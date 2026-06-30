"""
panel_ros.py — Capa ROS del panel Touch Haptic (patrón "QNode").

Gira un nodo rclpy en un hilo y entrega a Qt (mediante señales) el estado del
robot OM-X, las articulaciones y la pose del Touch, y los botones del stylus.
Expone los servicios enable / stop (E-STOP), el cambio de modo y la apertura/
cierre manual del gripper.
"""

import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from omni_msgs.msg import OmniButtonEvent

from . import th_config as th


class PanelRos(QObject):
    robot_state = pyqtSignal(dict)            # {nombre_joint: posición} del OM-X
    touch_joints = pyqtSignal(dict)           # {nombre_joint: posición} del Touch
    touch_pose = pyqtSignal(float, float, float)   # x, y, z [m] del stylus
    buttons = pyqtSignal(int, int)            # gris, blanco
    status_message = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        rclpy.init(args=None)
        self.node = Node('touch_haptic_panel')

        self.node.create_subscription(JointState, th.TOPIC_ROBOT_STATES,
                                      self._on_robot, 10)
        self.node.create_subscription(JointState, th.TOPIC_PHANTOM_JOINTS,
                                      self._on_touch_joints, 10)
        self.node.create_subscription(PoseStamped, th.TOPIC_PHANTOM_POSE,
                                      self._on_touch_pose, 10)
        self.node.create_subscription(OmniButtonEvent, th.TOPIC_PHANTOM_BUTTON,
                                      self._on_button, 10)
        self.pub_mode = self.node.create_publisher(String, th.TOPIC_MODE, 10)
        self.pub_grip = self.node.create_publisher(String, th.TOPIC_GRIPPER_CMD, 10)
        self.cli_enable = self.node.create_client(SetBool, th.SRV_ENABLE)
        self.cli_stop = self.node.create_client(Trigger, th.SRV_STOP)

        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

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
    def _on_robot(self, msg):
        self.robot_state.emit({n: msg.position[i] for i, n in enumerate(msg.name)
                               if i < len(msg.position)})

    def _on_touch_joints(self, msg):
        self.touch_joints.emit({n: msg.position[i] for i, n in enumerate(msg.name)
                                if i < len(msg.position)})

    def _on_touch_pose(self, msg):
        p = msg.pose.position
        self.touch_pose.emit(p.x, p.y, p.z)

    def _on_button(self, msg):
        self.buttons.emit(int(msg.grey_button), int(msg.white_button))

    # -- salida ----------------------------------------------------------
    def set_enabled(self, on):
        if not self.cli_enable.service_is_ready():
            self.status_message.emit('Servicio enable no disponible '
                                     '(¿está corriendo touch_haptic_node?).')
            return
        req = SetBool.Request()
        req.data = bool(on)
        fut = self.cli_enable.call_async(req)
        fut.add_done_callback(
            lambda f: self.status_message.emit(
                f.result().message if f.result() else 'enable: sin respuesta'))

    def estop(self):
        if not self.cli_stop.service_is_ready():
            self.status_message.emit('Servicio stop no disponible.')
            return
        fut = self.cli_stop.call_async(Trigger.Request())
        fut.add_done_callback(
            lambda f: self.status_message.emit(
                f.result().message if f.result() else 'stop: sin respuesta'))

    def set_mode(self, mode):
        self.pub_mode.publish(String(data=str(mode)))
        self.status_message.emit(f'Modo solicitado: {mode}')

    def gripper(self, action):
        self.pub_grip.publish(String(data=str(action)))
