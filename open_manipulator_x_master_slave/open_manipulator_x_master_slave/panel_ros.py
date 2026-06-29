"""
panel_ros.py — Capa ROS del panel maestro-esclavo (patrón "QNode").

Gira un nodo rclpy en un hilo y entrega los estados de maestro y esclavo a Qt
mediante una señal. Expone los servicios enable / stop (E-STOP) y el cambio del
parámetro mirror_gripper del nodo núcleo.
"""

import threading

import rclpy
from PyQt5.QtCore import QObject, pyqtSignal
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

from . import ms_config as ms

try:
    from rclpy.parameter_client import AsyncParameterClient
    _HAVE_PARAM_CLIENT = True
except ImportError:
    _HAVE_PARAM_CLIENT = False


class PanelRos(QObject):
    # Emite (master_dict, slave_dict) con {nombre_joint: posición}.
    states = pyqtSignal(dict, dict)
    status_message = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        rclpy.init(args=None)
        self.node = Node('master_slave_panel')
        self._master = {}
        self._slave = {}

        self.node.create_subscription(JointState, ms.TOPIC_MASTER_STATES,
                                      self._on_master, 10)
        self.node.create_subscription(JointState, ms.TOPIC_SLAVE_STATES,
                                      self._on_slave, 10)
        self.cli_enable = self.node.create_client(SetBool, ms.SRV_ENABLE)
        self.cli_stop = self.node.create_client(Trigger, ms.SRV_STOP)
        self._param_cli = (AsyncParameterClient(self.node, 'master_slave_node')
                           if _HAVE_PARAM_CLIENT else None)

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
    def _on_master(self, msg):
        self._master = {n: msg.position[i] for i, n in enumerate(msg.name)
                        if i < len(msg.position)}
        self.states.emit(self._master, self._slave)

    def _on_slave(self, msg):
        self._slave = {n: msg.position[i] for i, n in enumerate(msg.name)
                       if i < len(msg.position)}
        self.states.emit(self._master, self._slave)

    # -- salida ----------------------------------------------------------
    def set_enabled(self, on):
        if not self.cli_enable.service_is_ready():
            self.status_message.emit('Servicio enable no disponible '
                                     '(¿está corriendo master_slave_node?).')
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

    def set_mirror_gripper(self, on):
        if self._param_cli is None:
            self.status_message.emit('Cambio de mirror_gripper no disponible.')
            return
        self._param_cli.set_parameters([Parameter('mirror_gripper', value=bool(on))])
        self.status_message.emit(f'Espejar gripper: {"sí" if on else "no"}')
