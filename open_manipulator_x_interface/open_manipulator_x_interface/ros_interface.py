"""
ros_interface.py — Capa ROS de la GUI (patrón "QNode").

Crea un nodo rclpy, lo gira en un hilo aparte y expone el estado del robot a Qt
mediante una señal (`joint_state_received`), de modo que la GUI se actualiza en
su propio hilo de forma segura. Los comandos se publican / llaman desde la GUI a
través de métodos de esta clase.
"""

import threading

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from PyQt5.QtCore import QObject, pyqtSignal
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from . import config


class RosInterface(QObject):
    # Emite un dict {nombre_joint: posición} cada vez que llega /joint_states.
    joint_state_received = pyqtSignal(dict)
    # Texto para la barra de estado (info/avisos).
    status_message = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        rclpy.init(args=None)
        self.node = Node('interface_gui')

        self.pub_cmd = self.node.create_publisher(
            JointState, config.TOPIC_JOINT_COMMAND, 10)
        self.pub_grip = self.node.create_publisher(
            Float64, config.TOPIC_GRIPPER_COMMAND, 10)
        self.pub_mode = self.node.create_publisher(
            String, config.TOPIC_MODE, 10)
        self.pub_preview = self.node.create_publisher(
            JointState, config.TOPIC_JOINT_STATES_PREVIEW, 10)
        self.pub_traj = self.node.create_publisher(
            JointTrajectory, config.TOPIC_EXECUTE_TRAJ, 10)

        self.cli_home = self.node.create_client(Trigger, config.SRV_GO_HOME)
        self.cli_torque = self.node.create_client(SetBool, config.SRV_TORQUE)
        self.cli_stop = self.node.create_client(Trigger, config.SRV_STOP)

        self.node.create_subscription(
            JointState, config.TOPIC_JOINT_STATES, self._on_joint_state, 10)

        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    # -- spin ------------------------------------------------------------
    def _spin(self):
        try:
            self._executor.spin()
        except Exception:
            pass

    def shutdown(self):
        try:
            self._executor.shutdown()
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

    # -- entrada ---------------------------------------------------------
    def _on_joint_state(self, msg):
        d = {n: msg.position[i] for i, n in enumerate(msg.name) if i < len(msg.position)}
        self.joint_state_received.emit(d)

    # -- salida: brazo y gripper ----------------------------------------
    def send_arm(self, q4):
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(config.JOINT_NAMES)
        msg.position = [float(v) for v in q4]
        self.pub_cmd.publish(msg)

    def send_gripper_m(self, meters):
        self.pub_grip.publish(Float64(data=float(meters)))

    def set_mode(self, mode):
        self.pub_mode.publish(String(data=mode))
        self.status_message.emit(f'Modo solicitado: {mode}')

    def publish_preview(self, q4, gripper_m):
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.name = list(config.ALL_JOINT_NAMES) + ['gripper_right_joint']
        msg.position = [float(q4[0]), float(q4[1]), float(q4[2]), float(q4[3]),
                        float(gripper_m), float(gripper_m)]
        self.pub_preview.publish(msg)

    def relay_preview(self, state):
        """Reenvía el estado real del robot al modelo preview de RViz, para que
        el modelo siga al robot cuando NO se está previsualizando."""
        grip = state.get(config.GRIPPER_JOINT, config.GRIPPER_CLOSED_M)
        q4 = [state.get(n, 0.0) for n in config.JOINT_NAMES]
        self.publish_preview(q4, grip)

    # -- salida: servicios ----------------------------------------------
    def go_home(self):
        self._call_trigger(self.cli_home, 'Ir a cero')

    def stop(self):
        self._call_trigger(self.cli_stop, 'Stop')

    def set_torque(self, on):
        if not self.cli_torque.service_is_ready():
            self.status_message.emit('Servicio de torque no disponible.')
            return
        req = SetBool.Request()
        req.data = bool(on)
        fut = self.cli_torque.call_async(req)
        fut.add_done_callback(
            lambda f: self.status_message.emit(
                f.result().message if f.result() else 'Torque: sin respuesta'))

    def _call_trigger(self, client, label):
        if not client.service_is_ready():
            self.status_message.emit(f'{label}: servicio no disponible '
                                     '(¿está corriendo robot_bridge?).')
            return
        fut = client.call_async(Trigger.Request())
        fut.add_done_callback(
            lambda f: self.status_message.emit(
                f.result().message if f.result() else f'{label}: sin respuesta'))

    # -- salida: trayectoria de waypoints -------------------------------
    def execute_trajectory(self, waypoints):
        """
        waypoints: lista de dicts con claves:
            'q'        -> [q1,q2,q3,q4]   [rad]
            'gripper'  -> apertura [m]
            'time'     -> tiempo (s) para llegar desde el waypoint anterior
        """
        traj = JointTrajectory()
        traj.joint_names = list(config.JOINT_NAMES) + [config.GRIPPER_JOINT]
        t_acc = 0.0
        for wp in waypoints:
            t_acc += max(0.1, float(wp.get('time', 2.0)))
            pt = JointTrajectoryPoint()
            pt.positions = [float(wp['q'][0]), float(wp['q'][1]),
                            float(wp['q'][2]), float(wp['q'][3]),
                            float(wp.get('gripper', config.GRIPPER_CLOSED_M))]
            sec = int(t_acc)
            pt.time_from_start = Duration(sec=sec, nanosec=int((t_acc - sec) * 1e9))
            traj.points.append(pt)
        self.pub_traj.publish(traj)
        self.status_message.emit(f'Trayectoria enviada: {len(waypoints)} waypoints.')
