"""
virtual_touch.py — Geomagic Touch VIRTUAL para pruebas sin hardware.

Publica los mismos tópicos que el driver Geomagic real (Geomagic_Touch_ROS2):
    /phantom/joint_states  (sensor_msgs/JointState)        6 articulaciones
    /phantom/pose          (geometry_msgs/PoseStamped) [m]  pose del stylus
    /phantom/button        (omni_msgs/OmniButtonEvent)      botones (gris/blanco)

Genera movimiento senoidal suave y alterna los botones periódicamente, de modo
que se puede verificar toda la cadena (touch_haptic_node + panel) sin el Touch.
NO ejecutar junto al driver real: ambos publicarían en /phantom/*.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from omni_msgs.msg import OmniButtonEvent

from . import th_config as th


class VirtualTouch(Node):
    def __init__(self):
        super().__init__('virtual_touch')
        self.declare_parameter('rate', 100.0)
        self.rate = self.get_parameter('rate').get_parameter_value().double_value or 100.0

        self.pub_joints = self.create_publisher(JointState, th.TOPIC_PHANTOM_JOINTS, 10)
        self.pub_pose = self.create_publisher(PoseStamped, th.TOPIC_PHANTOM_POSE, 10)
        self.pub_button = self.create_publisher(OmniButtonEvent, th.TOPIC_PHANTOM_BUTTON, 10)

        self.t = 0.0
        self.dt = 1.0 / self.rate
        self._grey = 0
        self._white = 0
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(f'virtual_touch publicando /phantom/* @ {self.rate:.0f} Hz '
                               '(maestro senoidal de prueba).')

    def _tick(self):
        self.t += self.dt
        w = 2.0 * math.pi * 0.08      # ~0.08 Hz, movimiento lento

        # Articulaciones (6): waist, shoulder, elbow, yaw, pitch, roll.
        amp = [0.6, 0.4, 0.4, 0.3, 0.5, 0.3]
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(th.PHANTOM_JOINT_NAMES)
        js.position = [amp[i] * math.sin(w * self.t + i * 0.7)
                       for i in range(len(th.PHANTOM_JOINT_NAMES))]
        self.pub_joints.publish(js)

        # Pose del stylus [m] alrededor de un centro (frame del driver).
        ps = PoseStamped()
        ps.header.stamp = js.header.stamp
        ps.header.frame_id = 'map'
        ps.pose.position.x = 0.05 * math.sin(w * self.t)          # derecha
        ps.pose.position.y = 0.05 * math.sin(w * self.t + 1.0)    # adelante
        ps.pose.position.z = 0.04 * math.sin(w * self.t + 2.0)    # arriba
        ps.pose.orientation.w = 1.0
        self.pub_pose.publish(ps)

        # Botones: alterna gris/blanco cada ~6 s para probar abrir/cerrar gripper.
        phase = int(self.t / 6.0) % 4
        grey = 1 if phase == 1 else 0
        white = 1 if phase == 3 else 0
        if grey != self._grey or white != self._white:
            ev = OmniButtonEvent()
            ev.grey_button = grey
            ev.white_button = white
            self.pub_button.publish(ev)
            self._grey, self._white = grey, white


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VirtualTouch()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
