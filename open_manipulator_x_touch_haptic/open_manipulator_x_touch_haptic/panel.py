"""
panel.py — Panel de control PyQt5 de la teleoperación Touch Haptic -> OM-X.

Ventana compacta: habilitar/pausar el espejo, E-STOP, selector de modo
(Articular / Cartesiano), abrir/cerrar gripper, estado de conexión (Touch y
robot) y telemetría (articulaciones del Touch mapeadas vs robot, en grados;
posición del efector en cm; gripper en %; botones del stylus).
"""

import math
import signal
import sys
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QApplication, QButtonGroup, QGridLayout, QGroupBox,
                             QHBoxLayout, QLabel, QPushButton, QRadioButton,
                             QVBoxLayout, QWidget)

from open_manipulator_x_interface import config as omx
from open_manipulator_x_interface.kinematics import fkin

from . import th_config as th
from .panel_ros import PanelRos

# Mapa OM-joint -> articulación del Touch (para la telemetría comparada).
_OM_FROM_TOUCH = {om: ph for ph, om, _, _ in th.JOINT_MAP}


class Panel(QWidget):
    def __init__(self, ros):
        super().__init__()
        self.ros = ros
        self.setWindowTitle('OpenMANIPULATOR-X — Touch Haptic')
        self.resize(580, 520)
        self._t_touch = 0.0
        self._t_robot = 0.0
        self._robot = {}
        self._touch = {}

        # --- estado / conexión --------------------------------------------
        self.lbl_touch_conn = QLabel('Touch: ●')
        self.lbl_robot_conn = QLabel('Robot: ●')
        self.lbl_enable = QLabel('Espejo: OFF')
        for w in (self.lbl_touch_conn, self.lbl_robot_conn, self.lbl_enable):
            w.setStyleSheet('font-weight: bold;')
        status = QHBoxLayout()
        status.addWidget(self.lbl_touch_conn)
        status.addWidget(self.lbl_robot_conn)
        status.addStretch(1)
        status.addWidget(self.lbl_enable)

        # --- modo ----------------------------------------------------------
        self.rb_joint = QRadioButton('Articular')
        self.rb_cart = QRadioButton('Cartesiano')
        self.rb_joint.setChecked(th.DEFAULT_MODE == th.MODE_JOINT)
        self.rb_cart.setChecked(th.DEFAULT_MODE == th.MODE_CARTESIAN)
        grp = QButtonGroup(self)
        grp.addButton(self.rb_joint)
        grp.addButton(self.rb_cart)
        self.rb_joint.toggled.connect(self._on_mode)
        mode_box = QGroupBox('Modo de teleoperación')
        ml = QHBoxLayout(mode_box)
        ml.addWidget(self.rb_joint)
        ml.addWidget(self.rb_cart)
        ml.addStretch(1)

        # --- botones grandes ----------------------------------------------
        self.btn_enable = QPushButton('▶  HABILITAR')
        self.btn_enable.setCheckable(True)
        self.btn_enable.setMinimumHeight(56)
        self.btn_enable.setStyleSheet('font-size: 16px; font-weight: bold;')
        self.btn_enable.toggled.connect(self._on_enable)

        self.btn_estop = QPushButton('■  E-STOP')
        self.btn_estop.setMinimumHeight(56)
        self.btn_estop.setStyleSheet(
            'font-size: 16px; font-weight: bold; background:#c0392b; color:white;')
        self.btn_estop.clicked.connect(self._on_estop)
        btns = QHBoxLayout()
        btns.addWidget(self.btn_enable, 2)
        btns.addWidget(self.btn_estop, 1)

        # --- gripper -------------------------------------------------------
        self.btn_open = QPushButton('Abrir gripper')
        self.btn_close = QPushButton('Cerrar gripper')
        self.btn_open.clicked.connect(lambda: self.ros.gripper('open'))
        self.btn_close.clicked.connect(lambda: self.ros.gripper('close'))
        self.lbl_btns = QLabel('Botones stylus:  gris ○   blanco ○')
        self.lbl_btns.setStyleSheet('font-family: monospace;')
        grip_box = QGroupBox('Gripper  (botones del stylus o manual)')
        gl = QHBoxLayout(grip_box)
        gl.addWidget(self.btn_open)
        gl.addWidget(self.btn_close)
        gl.addStretch(1)
        gl.addWidget(self.lbl_btns)

        # --- telemetría ----------------------------------------------------
        self._cells = {}
        grid = QGridLayout()
        for c, h in enumerate(['', 'Touch', 'Robot', 'Error']):
            lab = QLabel(h)
            lab.setStyleSheet('font-weight: bold;')
            grid.addWidget(lab, 0, c)
        for r, name in enumerate(omx.JOINT_NAMES, start=1):
            grid.addWidget(QLabel(f'J{r} ({_OM_FROM_TOUCH.get(name, "-")})'), r, 0)
            for c, col in enumerate(['t', 'r', 'e'], start=1):
                cell = QLabel('--')
                cell.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                cell.setStyleSheet('font-family: monospace; font-size: 14px;')
                grid.addWidget(cell, r, c)
                self._cells[(name, col)] = cell
        tele_box = QGroupBox('Telemetría articular  (Touch mapeado vs robot, en °)')
        tele_box.setLayout(grid)

        self.lbl_ee = QLabel('Efector robot (cm):  --        Stylus (cm):  --')
        self.lbl_ee.setStyleSheet('font-family: monospace;')
        self.lbl_grip = QLabel('Gripper robot:  -- %')
        self.lbl_grip.setStyleSheet('font-family: monospace;')

        # --- layout --------------------------------------------------------
        root = QVBoxLayout(self)
        root.addLayout(status)
        root.addWidget(mode_box)
        root.addLayout(btns)
        root.addWidget(grip_box)
        root.addWidget(tele_box)
        root.addWidget(self.lbl_ee)
        root.addWidget(self.lbl_grip)
        self.status_line = QLabel('Listo. Habilita para que el robot siga al Touch.')
        self.status_line.setStyleSheet('color:#555;')
        root.addWidget(self.status_line)

        # --- señales -------------------------------------------------------
        self.ros.robot_state.connect(self._on_robot)
        self.ros.touch_joints.connect(self._on_touch)
        self.ros.touch_pose.connect(self._on_pose)
        self.ros.buttons.connect(self._on_buttons)
        self.ros.status_message.connect(self.status_line.setText)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._check_conn)
        self._timer.start(400)

    # ------------------------------------------------------------------
    def _on_mode(self, _checked):
        mode = th.MODE_JOINT if self.rb_joint.isChecked() else th.MODE_CARTESIAN
        self.ros.set_mode(mode)

    def _on_enable(self, checked):
        self.ros.set_enabled(checked)
        self.btn_enable.setText('⏸  ACTIVO (pulsa para pausar)' if checked
                                else '▶  HABILITAR')
        self.lbl_enable.setText('Espejo: ON' if checked else 'Espejo: OFF')
        self.lbl_enable.setStyleSheet(
            'font-weight: bold; color:%s;' % ('#2e8b30' if checked else '#b06000'))

    def _on_estop(self):
        self.btn_enable.setChecked(False)
        self.ros.estop()
        self.status_line.setText('E-STOP activado: espejo pausado, robot congelado.')

    def _on_robot(self, state):
        self._robot = state
        self._t_robot = time.time()
        self._refresh()

    def _on_touch(self, state):
        self._touch = state
        self._t_touch = time.time()
        self._refresh()

    def _on_pose(self, x, y, z):
        self._stylus = (x, y, z)

    def _on_buttons(self, grey, white):
        g = '●' if grey else '○'
        w = '●' if white else '○'
        self.lbl_btns.setText(f'Botones stylus:  gris {g}   blanco {w}')

    def _refresh(self):
        for name in omx.JOINT_NAMES:
            rv = self._robot.get(name)
            tv = self._touch.get(_OM_FROM_TOUCH.get(name))
            self._set(name, 'r', rv)
            self._set(name, 't', tv)
            if rv is not None and tv is not None:
                self._cells[(name, 'e')].setText(f'{math.degrees(rv - tv):+6.1f}')
        # efector del robot + stylus
        ee = self._ee_str(self._robot)
        sty = getattr(self, '_stylus', None)
        sty_str = ('[%+5.1f,%+5.1f,%+5.1f]' % (sty[0]*100, sty[1]*100, sty[2]*100)
                   if sty else '--')
        self.lbl_ee.setText(f'Efector robot (cm):  {ee}        Stylus (cm):  {sty_str}')
        # gripper
        g = self._robot.get(omx.GRIPPER_JOINT)
        if g is not None:
            self.lbl_grip.setText(f'Gripper robot:  {omx.gripper_m_to_percent(g):3.0f} %')

    def _set(self, name, col, val):
        if val is not None:
            self._cells[(name, col)].setText(f'{math.degrees(val):+6.1f}')

    @staticmethod
    def _ee_str(state):
        q = [state.get(n) for n in omx.JOINT_NAMES]
        if any(v is None for v in q):
            return '--'
        x, y, z, _ = fkin(q)
        return f'[{x*100:+5.1f},{y*100:+5.1f},{z*100:+5.1f}]'

    def _check_conn(self):
        now = time.time()
        self._mark(self.lbl_touch_conn, 'Touch', now - self._t_touch < 1.2)
        self._mark(self.lbl_robot_conn, 'Robot', now - self._t_robot < 1.2)

    @staticmethod
    def _mark(label, name, ok):
        label.setText(f'{name}: ●')
        label.setStyleSheet('font-weight: bold; color:%s;'
                            % ('#2e8b30' if ok else '#b06000'))

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Space:
            self._on_estop()
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event):
        self.ros.shutdown()
        super().closeEvent(event)


def main(args=None):
    app = QApplication(sys.argv)
    ros = PanelRos()
    win = Panel(ros)
    win.show()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    keepalive = QTimer()
    keepalive.start(200)
    keepalive.timeout.connect(lambda: None)
    code = app.exec_()
    ros.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
