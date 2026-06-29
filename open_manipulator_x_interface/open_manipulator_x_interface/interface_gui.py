"""
interface_gui.py — Punto de entrada de la interfaz gráfica.

Arranca QApplication, crea la capa ROS (RosInterface, que gira un nodo rclpy en
un hilo) y muestra la ventana principal. La GUI es un cliente puro: no abre el
puerto del robot (eso lo hace robot_bridge).
"""

import signal
import sys

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from .ros_interface import RosInterface
from .ui.main_window import MainWindow


def main(args=None):
    app = QApplication(sys.argv)

    ros = RosInterface()
    window = MainWindow(ros)
    window.show()

    # Permitir que Ctrl+C cierre la aplicación limpiamente.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    timer = QTimer()
    timer.start(200)
    timer.timeout.connect(lambda: None)   # devuelve el control al intérprete

    exit_code = app.exec_()
    ros.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
