"""
master_slave_node.py — Núcleo de la teleoperación maestro-esclavo.

Único dueño de AMBOS puertos Dynamixel. Lee el brazo maestro (torque OFF,
backdrivable) y comanda el brazo esclavo (control de posición) para que lo siga
en tiempo real, con límite de velocidad por software y recorte a límites.

NO ejecutar a la vez que ros2_control ni que el robot_bridge del paquete
open_manipulator_x_interface: se pelearían por los puertos serie.

Modo simulación (`sim:=true`): no abre puertos. El maestro es virtual (movimiento
senoidal suave, o lo que llegue por /master/joint_command) y el esclavo hace eco
del comando. Permite probar todo (espejo, enable/E-STOP, panel) sin hardware.

Interfaces ROS
--------------
Pub  /master/joint_states  (sensor_msgs/JointState)
Pub  /slave/joint_states   (sensor_msgs/JointState)
Srv  /master_slave/enable  (std_srvs/SetBool)  activar/pausar espejo
Srv  /master_slave/stop    (std_srvs/Trigger)  E-STOP: pausa + congela esclavo
Sub  /master/joint_command (sensor_msgs/JointState)  solo en --sim
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from open_manipulator_x_interface import config as omx

from . import ms_config as ms


class MasterSlaveNode(Node):
    def __init__(self):
        super().__init__('master_slave_node')
        self.declare_parameter('master_port', ms.MASTER_PORT)
        self.declare_parameter('slave_port', ms.SLAVE_PORT)
        self.declare_parameter('sim', False)
        self.declare_parameter('enable_on_start', ms.ENABLE_ON_START)
        self.declare_parameter('mirror_gripper', ms.MIRROR_GRIPPER)

        gp = self.get_parameter
        self.master_port = gp('master_port').get_parameter_value().string_value
        self.slave_port = gp('slave_port').get_parameter_value().string_value
        self.sim = gp('sim').get_parameter_value().bool_value
        self.enabled = gp('enable_on_start').get_parameter_value().bool_value
        self.mirror_gripper = gp('mirror_gripper').get_parameter_value().bool_value

        # Estado del comando al esclavo (slew-limited): brazo rad + gripper m.
        self.cmd_arm = [0.0, 0.0, 0.0, 0.0]
        self.cmd_grip_m = omx.GRIPPER_CLOSED_M
        self.master_state = (list(self.cmd_arm), self.cmd_grip_m)
        self.slave_state = (list(self.cmd_arm), self.cmd_grip_m)
        self._sim_master_cmd = None         # override opcional en sim
        self._t0 = self._now()

        # --- drivers ------------------------------------------------------
        self.master = None
        self.slave = None
        if not self.sim:
            from open_manipulator_x_interface.dxl_driver import DynamixelDriver
            self.master = DynamixelDriver(self.master_port, ids=list(ms.MASTER_IDS.values()))
            self.master.connect()
            for i in ms.MASTER_IDS.values():
                self.master.set_torque(i, False)        # maestro libre
            self.slave = DynamixelDriver(self.slave_port, ids=list(ms.SLAVE_IDS.values()))
            self.slave.connect()
            self.slave.setup_all_position()             # esclavo posición, torque ON
            self.get_logger().info(
                f'Maestro en {self.master_port} (libre) | Esclavo en {self.slave_port} '
                f'(posición). IDs maestro={list(ms.MASTER_IDS.values())} '
                f'esclavo={list(ms.SLAVE_IDS.values())}.')
            # Semilla: el comando arranca en la pose actual del esclavo (sin saltos).
            arm, grip = self._read_arm(self.slave, ms.SLAVE_IDS)
            if arm is not None:
                self.cmd_arm, self.cmd_grip_m = arm, grip
        else:
            self.get_logger().warn('MODO SIMULACIÓN (--sim): maestro virtual, '
                                   'esclavo en eco. Sin puertos.')

        # --- ROS I/O ------------------------------------------------------
        self.pub_master = self.create_publisher(JointState, ms.TOPIC_MASTER_STATES, 10)
        self.pub_slave = self.create_publisher(JointState, ms.TOPIC_SLAVE_STATES, 10)
        self.create_service(SetBool, ms.SRV_ENABLE, self._srv_enable)
        self.create_service(Trigger, ms.SRV_STOP, self._srv_stop)
        if self.sim:
            self.create_subscription(JointState, ms.TOPIC_MASTER_CMD,
                                     self._on_sim_master_cmd, 10)

        self.dt = 1.0 / ms.RATE_HZ
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(f'master_slave_node listo @ {ms.RATE_HZ:.0f} Hz '
                               f'(espejo {"ON" if self.enabled else "OFF"}).')

    # ====================================================================
    #  Lazo principal
    # ====================================================================
    def _tick(self):
        # 1) Estado del maestro
        q_arm, grip_m = self._get_master()
        self.master_state = (q_arm, grip_m)

        # 2) Objetivo del esclavo
        if self.enabled:
            target_arm = [omx.clamp_joint(n, q_arm[i] * ms.JOINT_SCALE)
                          for i, n in enumerate(omx.JOINT_NAMES)]
            target_grip = grip_m if self.mirror_gripper else self.cmd_grip_m
        else:
            target_arm = list(self.cmd_arm)        # congelado
            target_grip = self.cmd_grip_m

        # 3) Slew-limit hacia el objetivo (suaviza el enganche; seguridad)
        max_arm = omx.MAX_JOINT_SPEED * self.dt
        for i in range(4):
            self.cmd_arm[i] += _clip(target_arm[i] - self.cmd_arm[i], max_arm)
            self.cmd_arm[i] = omx.clamp_joint(omx.JOINT_NAMES[i], self.cmd_arm[i])
        max_grip = omx.MAX_GRIPPER_SPEED_M * self.dt
        self.cmd_grip_m += _clip(target_grip - self.cmd_grip_m, max_grip)

        # 4) Escribir al esclavo
        if not self.sim:
            goals = {ms.SLAVE_IDS[n]: omx.arm_rad_to_ticks(n, self.cmd_arm[i])
                     for i, n in enumerate(omx.JOINT_NAMES)}
            if self.mirror_gripper:
                goals[ms.SLAVE_IDS['gripper']] = omx.gripper_motor_rad_to_ticks(
                    omx.gripper_m_to_motor_rad(self.cmd_grip_m))
            self.slave.write_goal_ticks(goals)
            arm, grip = self._read_arm(self.slave, ms.SLAVE_IDS)
            self.slave_state = (arm, grip) if arm is not None else self.slave_state
        else:
            self.slave_state = (list(self.cmd_arm), self.cmd_grip_m)

        # 5) Publicar
        self._publish(self.pub_master, *self.master_state)
        self._publish(self.pub_slave, *self.slave_state)

    # ====================================================================
    #  Maestro: hardware o simulación
    # ====================================================================
    def _get_master(self):
        if not self.sim:
            arm, grip = self._read_arm(self.master, ms.MASTER_IDS)
            if arm is None:
                return self.master_state           # mantiene el último válido
            return arm, grip
        if self._sim_master_cmd is not None:
            return self._sim_master_cmd
        # Maestro virtual: senoidal suave dentro de límites.
        t = self._now() - self._t0
        amp = [0.6, 0.4, 0.4, 0.5]
        arm = []
        for i, n in enumerate(omx.JOINT_NAMES):
            lo, hi = omx.joint_limits()[n]
            val = amp[i] * math.sin(2.0 * math.pi * 0.08 * t + i * 0.7)
            arm.append(max(lo, min(hi, val)))
        # gripper oscila cerrado<->abierto
        s = 0.5 * (1.0 + math.sin(2.0 * math.pi * 0.05 * t))
        grip = omx.gripper_percent_to_m(100.0 * s)
        return arm, grip

    def _read_arm(self, driver, id_map):
        ticks = driver.read_positions_ticks()
        if ticks is None:
            return None, None
        arm = [omx.arm_ticks_to_rad(n, ticks[id_map[n]]) for n in omx.JOINT_NAMES]
        grip = omx.gripper_motor_rad_to_m(
            omx.gripper_ticks_to_motor_rad(ticks[id_map['gripper']]))
        return arm, grip

    # ====================================================================
    #  Publicación / servicios
    # ====================================================================
    def _publish(self, pub, arm, grip_m):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(omx.ALL_JOINT_NAMES) + ['gripper_right_joint']
        js.position = [arm[0], arm[1], arm[2], arm[3], grip_m, grip_m]
        pub.publish(js)

    def _srv_enable(self, request, response):
        self.enabled = bool(request.data)
        if self.enabled:
            self.get_logger().info('Espejo HABILITADO: el esclavo sigue al maestro.')
        else:
            self.get_logger().info('Espejo PAUSADO: el esclavo se congela.')
        response.success = True
        response.message = f'Espejo {"ON" if self.enabled else "OFF"}.'
        return response

    def _srv_stop(self, request, response):
        self.enabled = False
        # Congela en la posición actual del esclavo.
        if not self.sim:
            arm, grip = self._read_arm(self.slave, ms.SLAVE_IDS)
            if arm is not None:
                self.cmd_arm, self.cmd_grip_m = arm, grip
        response.success = True
        response.message = 'E-STOP: espejo pausado, esclavo congelado.'
        return response

    def _on_sim_master_cmd(self, msg):
        idx = {n: k for k, n in enumerate(msg.name)}
        arm = [msg.position[idx[n]] if n in idx else 0.0 for n in omx.JOINT_NAMES]
        grip = msg.position[idx[omx.GRIPPER_JOINT]] if omx.GRIPPER_JOINT in idx \
            else self.cmd_grip_m
        self._sim_master_cmd = (arm, grip)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def destroy_node(self):
        for drv in (self.master, self.slave):
            if drv is not None:
                try:
                    drv.close()
                except Exception:
                    pass
        super().destroy_node()


def _clip(v, lim):
    return max(-lim, min(lim, v))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MasterSlaveNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        print(f'[master_slave_node] ERROR: {exc}')
        print('Sugerencia: revisa puertos/permisos de los DOS U2D2 '
              '(ls -l /dev/ttyUSB*) o lanza con sim:=true para probar sin robots.')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
