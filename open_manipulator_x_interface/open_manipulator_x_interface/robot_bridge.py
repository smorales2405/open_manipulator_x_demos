"""
robot_bridge.py — Único dueño del puerto Dynamixel. Traduce la API ROS de la
interfaz a comandos de servo y publica el estado del robot.

Es un nodo independiente de la GUI. NO debe ejecutarse a la vez que el stack
oficial ros2_control (open_manipulator hardware.launch.py): ambos abrirían el
mismo /dev/ttyUSB0.

Modos
-----
- POSITION : torque ON. Sigue el último comando articular (con límite de
             velocidad por software) y/o reproduce una trayectoria.
- TEACH    : torque OFF en el brazo (queda backdrivable, "modo libre"). Solo
             lee y publica estado; el gripper mantiene torque para sostener.

Flag --sim (parámetro `sim:=true`): no abre el puerto; el estado publicado es un
eco del comando. Permite probar toda la GUI y RViz sin robot.

Interfaces ROS (solo tipos estándar)
------------------------------------
Pub  /joint_states                (sensor_msgs/JointState)   estado real (5 joints)
Sub  /interface/joint_command     (sensor_msgs/JointState)   target brazo [rad]
Sub  /interface/gripper_command   (std_msgs/Float64)         apertura gripper [m]
Sub  /interface/mode              (std_msgs/String)          "position" | "teach"
Sub  /interface/execute_trajectory(trajectory_msgs/JointTrajectory)  playback
Srv  /interface/go_home           (std_srvs/Trigger)         mover suave a 0 rad
Srv  /interface/torque            (std_srvs/SetBool)         habilita/inhibe torque brazo
Srv  /interface/stop              (std_srvs/Trigger)         congela en posición actual
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

from . import config


class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')
        self.declare_parameter('port_name', config.PORT)
        self.declare_parameter('sim', False)
        self.port_name = self.get_parameter('port_name').get_parameter_value().string_value
        self.sim = self.get_parameter('sim').get_parameter_value().bool_value

        # --- estado interno -------------------------------------------------
        self.mode = config.MODE_POSITION
        # target / cmd / measured en unidades "humanas": brazo rad, gripper m
        self.target_arm = [0.0, 0.0, 0.0, 0.0]
        self.target_grip_m = config.GRIPPER_CLOSED_M
        self.cmd_arm = [0.0, 0.0, 0.0, 0.0]
        self.cmd_grip_m = config.GRIPPER_CLOSED_M
        self.meas_arm = [0.0, 0.0, 0.0, 0.0]
        self.meas_grip_m = config.GRIPPER_CLOSED_M
        self.arm_torque = True

        # --- reproducción de trayectoria ------------------------------------
        self._traj = None          # dict: times[], arm[N][4], grip[N]
        self._traj_t0 = None

        # --- driver Dynamixel ----------------------------------------------
        self.driver = None
        if not self.sim:
            from .dxl_driver import DynamixelDriver
            self.driver = DynamixelDriver(port=self.port_name, baud=config.BAUDRATE)
            self.driver.connect()
            self.driver.setup_all_position()
            self.get_logger().info(f'Conectado a {self.port_name} (Dynamixel IDs '
                                   f'{self.driver.ids}).')
            self._read_measured()
            self.cmd_arm = list(self.meas_arm)
            self.target_arm = list(self.meas_arm)
            self.cmd_grip_m = self.meas_grip_m
            self.target_grip_m = self.meas_grip_m
        else:
            self.get_logger().warn('MODO SIMULACIÓN (--sim): no se abre el puerto; '
                                   'el estado es un eco del comando.')

        # --- ROS I/O --------------------------------------------------------
        self.pub_js = self.create_publisher(JointState, config.TOPIC_JOINT_STATES, 10)
        self.create_subscription(JointState, config.TOPIC_JOINT_COMMAND,
                                 self._on_joint_cmd, 10)
        self.create_subscription(Float64, config.TOPIC_GRIPPER_COMMAND,
                                 self._on_gripper_cmd, 10)
        self.create_subscription(String, config.TOPIC_MODE, self._on_mode, 10)
        self.create_subscription(JointTrajectory, config.TOPIC_EXECUTE_TRAJ,
                                 self._on_trajectory, 10)
        self.create_service(Trigger, config.SRV_GO_HOME, self._srv_go_home)
        self.create_service(SetBool, config.SRV_TORQUE, self._srv_torque)
        self.create_service(Trigger, config.SRV_STOP, self._srv_stop)

        self.dt = 1.0 / config.RATE_HZ
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(f'robot_bridge listo @ {config.RATE_HZ:.0f} Hz '
                               f'(modo {self.mode}).')

    # ====================================================================
    #  Lectura de estado del hardware
    # ====================================================================
    def _read_measured(self):
        if self.sim or self.driver is None:
            return
        ticks = self.driver.read_positions_ticks()
        if ticks is None:
            self.get_logger().warn('SyncRead falló; se mantiene el último estado.')
            return
        for i, name in enumerate(config.JOINT_NAMES):
            self.meas_arm[i] = config.arm_ticks_to_rad(name, ticks[config.DXL_IDS[name]])
        self.meas_grip_m = config.gripper_ticks_to_m(ticks[config.GRIPPER_ID])

    # ====================================================================
    #  Lazo principal
    # ====================================================================
    def _tick(self):
        # 1) Estado medido
        if self.sim:
            self.meas_arm = list(self.cmd_arm)
            self.meas_grip_m = self.cmd_grip_m
        else:
            self._read_measured()

        # 2) Trayectoria en curso -> actualiza target
        if self._traj is not None:
            self._update_from_trajectory()

        if self.mode == config.MODE_TEACH:
            # Brazo suelto: el target sigue al medido para no saltar al volver.
            self.target_arm = list(self.meas_arm)
            self.cmd_arm = list(self.meas_arm)
            # El gripper conserva su comando (mantiene torque).
            if not self.sim:
                self.driver.write_goal_ticks(
                    {config.GRIPPER_ID: config.gripper_m_to_ticks(self.cmd_grip_m)})
        else:
            # 3) Slew-limit (seguridad) y escritura de metas
            self._slew_toward_target()
            if not self.sim:
                goals = {}
                for i, name in enumerate(config.JOINT_NAMES):
                    goals[config.DXL_IDS[name]] = config.arm_rad_to_ticks(name, self.cmd_arm[i])
                goals[config.GRIPPER_ID] = config.gripper_m_to_ticks(self.cmd_grip_m)
                self.driver.write_goal_ticks(goals)

        # 4) Publicar estado
        self._publish_state()

    def _slew_toward_target(self):
        max_arm = config.MAX_JOINT_SPEED * self.dt
        for i in range(4):
            self.cmd_arm[i] += _clip(self.target_arm[i] - self.cmd_arm[i], max_arm)
            self.cmd_arm[i] = config.clamp_joint(config.JOINT_NAMES[i], self.cmd_arm[i])
        max_grip = config.MAX_GRIPPER_SPEED_M * self.dt
        self.cmd_grip_m += _clip(self.target_grip_m - self.cmd_grip_m, max_grip)

    def _publish_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(config.ALL_JOINT_NAMES) + ['gripper_right_joint']
        # gripper_right_joint imita al left (el URDF lo trata como mimic, pero lo
        # publicamos explícito por robustez en RViz).
        js.position = [self.meas_arm[0], self.meas_arm[1], self.meas_arm[2],
                       self.meas_arm[3], self.meas_grip_m, self.meas_grip_m]
        self.pub_js.publish(js)

    # ====================================================================
    #  Callbacks de comandos
    # ====================================================================
    def _on_joint_cmd(self, msg):
        if self.mode != config.MODE_POSITION:
            return
        self._cancel_traj()
        idx = {n: k for k, n in enumerate(msg.name)}
        for i, name in enumerate(config.JOINT_NAMES):
            if name in idx:
                self.target_arm[i] = config.clamp_joint(name, msg.position[idx[name]])

    def _on_gripper_cmd(self, msg):
        lo, hi = sorted(config.GRIPPER_PRISMATIC)
        self.target_grip_m = max(lo, min(hi, float(msg.data)))

    def _on_mode(self, msg):
        new_mode = msg.data.strip().lower()
        if new_mode not in (config.MODE_POSITION, config.MODE_TEACH):
            self.get_logger().warn(f'Modo desconocido: "{msg.data}"')
            return
        if new_mode == self.mode:
            return
        self._set_mode(new_mode)

    def _set_mode(self, new_mode):
        self._cancel_traj()
        if new_mode == config.MODE_TEACH:
            if not self.sim:
                for i in config.ARM_IDS:
                    self.driver.set_torque(i, False)
            self.arm_torque = False
            self.get_logger().info('Modo TEACH: brazo suelto (torque OFF). '
                                   'Sostén el brazo antes de moverlo.')
        else:
            # Volver a posición: alinear cmd/target al medido y re-habilitar torque.
            self._read_measured()
            self.cmd_arm = list(self.meas_arm)
            self.target_arm = list(self.meas_arm)
            if not self.sim:
                for i in config.ARM_IDS:
                    self.driver.set_torque(i, True)
            self.arm_torque = True
            self.get_logger().info('Modo POSITION: torque ON, manteniendo posición actual.')
        self.mode = new_mode

    # ====================================================================
    #  Servicios
    # ====================================================================
    def _srv_go_home(self, request, response):
        if self.mode != config.MODE_POSITION:
            self._set_mode(config.MODE_POSITION)
        self._start_trajectory(
            times=[0.0, config.HOME_TIME_S],
            arm=[list(self.meas_arm), [0.0, 0.0, 0.0, 0.0]],
            grip=[self.meas_grip_m, self.meas_grip_m])
        response.success = True
        response.message = 'Moviendo a posición cero (0 rad).'
        return response

    def _srv_torque(self, request, response):
        on = bool(request.data)
        if not self.sim:
            for i in config.ARM_IDS:
                self.driver.set_torque(i, on)
        self.arm_torque = on
        if not on:
            self.mode = config.MODE_TEACH
        response.success = True
        response.message = f'Torque del brazo {"ON" if on else "OFF"}.'
        return response

    def _srv_stop(self, request, response):
        self._cancel_traj()
        self._read_measured()
        self.cmd_arm = list(self.meas_arm)
        self.target_arm = list(self.meas_arm)
        self.target_grip_m = self.cmd_grip_m
        response.success = True
        response.message = 'Detenido: manteniendo posición actual.'
        return response

    # ====================================================================
    #  Trayectorias
    # ====================================================================
    def _on_trajectory(self, msg):
        if not msg.points:
            return
        if self.mode != config.MODE_POSITION:
            self._set_mode(config.MODE_POSITION)
        names = list(msg.joint_names)
        col = {n: k for k, n in enumerate(names)}
        times, arm, grip = [], [], []
        prev_arm = list(self.meas_arm)
        prev_grip = self.meas_grip_m
        # Punto inicial en t=0 con el estado actual: la interpolación arranca desde
        # la pose real (evita saltos/extrapolación en el primer tramo).
        times.append(0.0)
        arm.append(list(prev_arm))
        grip.append(prev_grip)
        for pt in msg.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            a = list(prev_arm)
            for i, jn in enumerate(config.JOINT_NAMES):
                if jn in col:
                    a[i] = config.clamp_joint(jn, pt.positions[col[jn]])
            g = prev_grip
            if config.GRIPPER_JOINT in col:
                lo, hi = sorted(config.GRIPPER_PRISMATIC)
                g = max(lo, min(hi, pt.positions[col[config.GRIPPER_JOINT]]))
            times.append(t)
            arm.append(a)
            grip.append(g)
            prev_arm, prev_grip = a, g
        self._start_trajectory(times, arm, grip)
        self.get_logger().info(f'Ejecutando trayectoria: {len(times)} puntos, '
                               f'{times[-1]:.1f} s.')

    def _start_trajectory(self, times, arm, grip):
        self._traj = {'times': times, 'arm': arm, 'grip': grip}
        self._traj_t0 = self._now_s()

    def _cancel_traj(self):
        self._traj = None
        self._traj_t0 = None

    def _update_from_trajectory(self):
        te = self._now_s() - self._traj_t0
        times = self._traj['times']
        arm = self._traj['arm']
        grip = self._traj['grip']
        glo, ghi = sorted(config.GRIPPER_PRISMATIC)
        if te >= times[-1]:
            self.target_arm = list(arm[-1])
            self.target_grip_m = max(glo, min(ghi, grip[-1]))
            self._cancel_traj()
            return
        # localizar segmento [k, k+1]
        k = 0
        while k < len(times) - 1 and times[k + 1] < te:
            k += 1
        t0, t1 = times[k], times[k + 1]
        s = 0.0 if t1 <= t0 else (te - t0) / (t1 - t0)
        self.target_arm = [arm[k][i] + s * (arm[k + 1][i] - arm[k][i]) for i in range(4)]
        self.target_grip_m = max(glo, min(ghi, grip[k] + s * (grip[k + 1] - grip[k])))

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def destroy_node(self):
        if self.driver is not None:
            try:
                self.driver.close()
            except Exception:
                pass
        super().destroy_node()


def _clip(v, lim):
    return max(-lim, min(lim, v))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RobotBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        print(f'[robot_bridge] ERROR: {exc}')
        print('Sugerencia: revisa el puerto/permisos (ls -l /dev/ttyUSB0) o '
              'lanza con sim:=true para probar sin robot.')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
