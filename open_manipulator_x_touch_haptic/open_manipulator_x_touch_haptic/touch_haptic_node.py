"""
touch_haptic_node.py — Núcleo de la teleoperación Geomagic Touch -> OM-X.

Único dueño del puerto Dynamixel del OpenMANIPULATOR-X. Lee el estado del Touch
(maestro cinemático, vía topics /phantom/*) y comanda el OM-X (control de
posición) para que lo siga en tiempo real, con límite de velocidad por software,
recorte a límites articulares y resolución de cinemática inversa para el modo
cartesiano. SOLO cinemática: no se usa ni se devuelve fuerza al Touch.

NO ejecutar a la vez que ros2_control, ni que el robot_bridge del paquete
open_manipulator_x_interface, ni que master_slave_node: se pelearían por el
puerto serie del OM-X.

Modos (conmutables con /touch_haptic/mode)
------------------------------------------
- joint     : mapea las articulaciones del Touch (waist, shoulder, elbow, pitch)
              a (joint1..joint4) del OM-X, de forma incremental desde un "engage".
- cartesian : mapea la posición XYZ del stylus a la posición del efector (IK).

En ambos, los 2 botones del stylus abren/cierran el gripper.

Engage (clutch)
---------------
Al HABILITAR (o al cambiar de modo) se captura la pose actual del Touch y del
robot como referencia. A partir de ahí se mapea el MOVIMIENTO relativo, de modo
que el robot no salta al engancharse y los offsets absolutos del Touch no importan.

Flag --sim (`sim:=true`): no abre el puerto del OM-X; el estado del robot es un eco
del comando. El Touch puede ser real o el virtual_touch de este paquete.

Interfaces ROS
--------------
Sub  /phantom/joint_states   (sensor_msgs/JointState)        articulaciones Touch
Sub  /phantom/pose           (geometry_msgs/PoseStamped) [m]  pose del stylus
Sub  /phantom/button         (omni_msgs/OmniButtonEvent)      botones del stylus
Sub  /touch_haptic/mode      (std_msgs/String)  "joint" | "cartesian"
Sub  /touch_haptic/gripper_cmd (std_msgs/String)  "open" | "close" (manual, panel)
Pub  /touch_haptic/robot_joint_states (sensor_msgs/JointState) estado del OM-X
Srv  /touch_haptic/enable    (std_srvs/SetBool)   habilita/pausa el espejo
Srv  /touch_haptic/stop      (std_srvs/Trigger)   E-STOP: pausa + congela
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from omni_msgs.msg import OmniButtonEvent

from open_manipulator_x_interface import config as omx
from open_manipulator_x_interface.kinematics import fkin, ik_step

from . import th_config as th


class TouchHapticNode(Node):
    def __init__(self):
        super().__init__('touch_haptic_node')
        self.declare_parameter('robot_port', th.ROBOT_PORT)
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('sim', False)
        self.declare_parameter('enable_on_start', th.ENABLE_ON_START)
        self.declare_parameter('mode', th.DEFAULT_MODE)
        self.declare_parameter('cart_scale', th.CART_SCALE)

        gp = self.get_parameter
        self.robot_port = gp('robot_port').get_parameter_value().string_value
        self.robot_id = gp('robot_id').get_parameter_value().integer_value
        self.sim = gp('sim').get_parameter_value().bool_value
        self.enabled = gp('enable_on_start').get_parameter_value().bool_value
        self.mode = gp('mode').get_parameter_value().string_value
        self.cart_scale = gp('cart_scale').get_parameter_value().double_value
        if self.mode not in (th.MODE_JOINT, th.MODE_CARTESIAN):
            self.mode = th.DEFAULT_MODE

        # Calibración del gripper según el ID del robot.
        omx.load_robot_config(self.robot_id)

        # --- estado del Touch (maestro) -----------------------------------
        self.touch_q = {}              # {nombre: rad} de /phantom/joint_states
        self.touch_pos = None          # [x,y,z] m de /phantom/pose
        self.grey = 0
        self.white = 0

        # --- estado / comando del robot (esclavo) -------------------------
        self.cmd_arm = [0.0, 0.0, 0.0, 0.0]
        self.cmd_grip_m = omx.GRIPPER_CLOSED_M
        self.target_grip_m = omx.GRIPPER_CLOSED_M
        self.meas_arm = [0.0, 0.0, 0.0, 0.0]
        self.meas_grip_m = omx.GRIPPER_CLOSED_M

        # --- referencias de engage (clutch) -------------------------------
        self.engage_touch_q = None     # dict {nombre: rad}
        self.engage_robot_q = None     # [q1..q4]
        self.engage_touch_pos = None   # [x,y,z]
        self.engage_ee = None          # [x,y,z] del efector

        # --- driver Dynamixel ---------------------------------------------
        self.driver = None
        if not self.sim:
            from open_manipulator_x_interface.dxl_driver import DynamixelDriver
            self.driver = DynamixelDriver(self.robot_port,
                                          ids=list(th.ROBOT_IDS.values()))
            self.driver.connect()
            self.driver.setup_all_position()
            self._read_measured()
            self.cmd_arm = list(self.meas_arm)
            self.cmd_grip_m = self.meas_grip_m
            self.target_grip_m = self.meas_grip_m
            self.get_logger().info(
                f'OM-X conectado en {self.robot_port} (IDs '
                f'{list(th.ROBOT_IDS.values())}).')
        else:
            self.get_logger().warn('MODO SIMULACIÓN (--sim): no se abre el puerto '
                                   'del OM-X; el estado del robot es un eco.')

        # --- ROS I/O ------------------------------------------------------
        self.create_subscription(JointState, th.TOPIC_PHANTOM_JOINTS,
                                 self._on_touch_joints, 10)
        self.create_subscription(PoseStamped, th.TOPIC_PHANTOM_POSE,
                                 self._on_touch_pose, 10)
        self.create_subscription(OmniButtonEvent, th.TOPIC_PHANTOM_BUTTON,
                                 self._on_button, 10)
        self.create_subscription(String, th.TOPIC_MODE, self._on_mode, 10)
        self.create_subscription(String, th.TOPIC_GRIPPER_CMD, self._on_grip_cmd, 10)
        self.pub_robot = self.create_publisher(JointState, th.TOPIC_ROBOT_STATES, 10)
        self.create_service(SetBool, th.SRV_ENABLE, self._srv_enable)
        self.create_service(Trigger, th.SRV_STOP, self._srv_stop)

        self.dt = 1.0 / th.RATE_HZ
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'touch_haptic_node listo @ {th.RATE_HZ:.0f} Hz '
            f'(modo {self.mode}, espejo {"ON" if self.enabled else "OFF"}).')

    # ====================================================================
    #  Entradas del Touch
    # ====================================================================
    def _on_touch_joints(self, msg):
        self.touch_q = {n: msg.position[i] for i, n in enumerate(msg.name)
                        if i < len(msg.position)}

    def _on_touch_pose(self, msg):
        p = msg.pose.position
        self.touch_pos = [p.x, p.y, p.z]

    def _on_button(self, msg):
        self.grey = int(msg.grey_button)
        self.white = int(msg.white_button)
        close_btn = self.grey if th.BUTTON_CLOSE == 'grey' else self.white
        open_btn = self.grey if th.BUTTON_OPEN == 'grey' else self.white
        if close_btn:
            self.target_grip_m = omx.GRIPPER_CLOSED_M
        elif open_btn:
            self.target_grip_m = omx.GRIPPER_OPEN_M

    def _on_grip_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'open':
            self.target_grip_m = omx.GRIPPER_OPEN_M
        elif cmd == 'close':
            self.target_grip_m = omx.GRIPPER_CLOSED_M

    def _on_mode(self, msg):
        new_mode = msg.data.strip().lower()
        if new_mode not in (th.MODE_JOINT, th.MODE_CARTESIAN):
            self.get_logger().warn(f'Modo desconocido: "{msg.data}"')
            return
        if new_mode == self.mode:
            return
        self.mode = new_mode
        # Re-anclar para que el cambio de modo no provoque saltos.
        if self.enabled:
            self._capture_engage()
        self.get_logger().info(f'Modo de teleoperación: {self.mode}.')

    # ====================================================================
    #  Lectura del robot
    # ====================================================================
    def _read_measured(self):
        if self.sim or self.driver is None:
            return
        ticks = self.driver.read_positions_ticks()
        if ticks is None:
            self.get_logger().warn('SyncRead falló; se mantiene el último estado.')
            return
        for i, name in enumerate(omx.JOINT_NAMES):
            self.meas_arm[i] = omx.arm_ticks_to_rad(name, ticks[th.ROBOT_IDS[name]])
        self.meas_grip_m = omx.gripper_ticks_to_m(ticks[th.ROBOT_IDS['gripper']])

    # ====================================================================
    #  Engage (clutch)
    # ====================================================================
    def _have_touch(self):
        have_joints = all(n in self.touch_q for n, _, _, _ in th.JOINT_MAP)
        return have_joints and self.touch_pos is not None

    def _capture_engage(self):
        """Captura la pose actual de Touch y robot como referencia del mapeo."""
        if not self._have_touch():
            return False
        self.engage_touch_q = dict(self.touch_q)
        self.engage_robot_q = list(self.meas_arm)
        self.engage_touch_pos = list(self.touch_pos)
        self.engage_ee = list(fkin(self.meas_arm)[:3])
        return True

    # ====================================================================
    #  Lazo principal
    # ====================================================================
    def _tick(self):
        # 1) Estado medido del robot
        if self.sim:
            self.meas_arm = list(self.cmd_arm)
            self.meas_grip_m = self.cmd_grip_m
        else:
            self._read_measured()

        # 2) Objetivo del brazo
        if self.enabled and self.engage_robot_q is not None and self._have_touch():
            if self.mode == th.MODE_JOINT:
                target_arm = self._target_joint()
            else:
                target_arm = self._target_cartesian()
        else:
            target_arm = list(self.cmd_arm)        # congelado

        # 3) Slew-limit hacia el objetivo (suavidad / seguridad) + clamp
        max_arm = omx.MAX_JOINT_SPEED * self.dt
        for i in range(4):
            self.cmd_arm[i] += _clip(target_arm[i] - self.cmd_arm[i], max_arm)
            self.cmd_arm[i] = omx.clamp_joint(omx.JOINT_NAMES[i], self.cmd_arm[i])
        max_grip = omx.MAX_GRIPPER_SPEED_M * self.dt
        self.cmd_grip_m += _clip(self.target_grip_m - self.cmd_grip_m, max_grip)

        # 4) Escribir al robot
        if not self.sim:
            goals = {th.ROBOT_IDS[n]: omx.arm_rad_to_ticks(n, self.cmd_arm[i])
                     for i, n in enumerate(omx.JOINT_NAMES)}
            goals[th.ROBOT_IDS['gripper']] = omx.gripper_m_to_ticks(self.cmd_grip_m)
            self.driver.write_goal_ticks(goals)

        # 5) Publicar estado del robot
        self._publish_robot()

    def _target_joint(self):
        """Mapeo incremental articular Touch -> OM-X (desde el engage)."""
        target = list(self.cmd_arm)
        for ph_name, om_name, sign, gain in th.JOINT_MAP:
            idx = omx.JOINT_NAMES.index(om_name)
            d = self.touch_q[ph_name] - self.engage_touch_q[ph_name]
            target[idx] = omx.clamp_joint(
                om_name, self.engage_robot_q[idx] + sign * gain * d)
        return target

    def _target_cartesian(self):
        """Mapeo de posición stylus -> efector, resuelto con IK incremental."""
        delta_touch = [self.touch_pos[i] - self.engage_touch_pos[i] for i in range(3)]
        robot_delta = th.map_touch_delta_to_robot(delta_touch)
        # Reescala si el panel cambió cart_scale en caliente (vía parámetro).
        if self.cart_scale != th.CART_SCALE and th.CART_SCALE != 0.0:
            robot_delta = [d * self.cart_scale / th.CART_SCALE for d in robot_delta]
        desired = [self.engage_ee[i] + robot_delta[i] for i in range(3)]
        cur_ee = fkin(self.cmd_arm)[:3]
        dpose = [desired[i] - cur_ee[i] for i in range(3)]
        q_new = ik_step(self.cmd_arm, dpose, hold_phi=th.CART_HOLD_PHI,
                        damping=th.IK_DAMPING, max_dq=th.IK_MAX_DQ)
        if q_new is None:
            return list(self.cmd_arm)              # fuera de alcance: mantener
        return [float(v) for v in q_new]

    def _publish_robot(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(omx.ALL_JOINT_NAMES) + ['gripper_right_joint']
        js.position = [self.meas_arm[0], self.meas_arm[1], self.meas_arm[2],
                       self.meas_arm[3], self.meas_grip_m, self.meas_grip_m]
        self.pub_robot.publish(js)

    # ====================================================================
    #  Servicios
    # ====================================================================
    def _srv_enable(self, request, response):
        on = bool(request.data)
        if on:
            if not self._capture_engage():
                response.success = False
                response.message = ('No hay datos del Touch todavía: lanza el driver '
                                    'Geomagic (o virtual_touch) antes de habilitar.')
                return response
            self.cmd_arm = list(self.meas_arm)     # arranca desde la pose real
            self.enabled = True
            self.get_logger().info('Espejo HABILITADO: el OM-X sigue al Touch.')
        else:
            self.enabled = False
            self.cmd_arm = list(self.meas_arm)
            self.get_logger().info('Espejo PAUSADO: el OM-X se congela.')
        response.success = True
        response.message = f'Espejo {"ON" if on else "OFF"} (modo {self.mode}).'
        return response

    def _srv_stop(self, request, response):
        self.enabled = False
        self._read_measured()
        self.cmd_arm = list(self.meas_arm)
        self.target_grip_m = self.cmd_grip_m
        response.success = True
        response.message = 'E-STOP: espejo pausado, robot congelado.'
        return response

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
        node = TouchHapticNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        print(f'[touch_haptic_node] ERROR: {exc}')
        print('Sugerencia: revisa el puerto/permisos del OM-X (ls -l /dev/ttyUSB*) '
              'o lanza con sim:=true para probar sin el robot.')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
