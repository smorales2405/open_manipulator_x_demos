"""
panel.py — Panel de control PyQt5 de la teleoperación maestro-esclavo.

Ventana compacta: habilitar/pausar el espejo, E-STOP, estado de conexión y una
tabla de telemetría que compara maestro vs esclavo (grados y error de
seguimiento) más la posición del efector de cada brazo (cm).
"""

import math
import signal
import sys
import time

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QApplication, QCheckBox, QGridLayout, QGroupBox,
                             QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget)

from open_manipulator_x_interface import config as omx
from open_manipulator_x_interface.kinematics import fkin

from .panel_ros import PanelRos


class Panel(QWidget):
    def __init__(self, ros):
        super().__init__()
        self.ros = ros
        self.setWindowTitle('OpenMANIPULATOR-X — Maestro / Esclavo')
        self.resize(560, 460)
        self._t_master = 0.0
        self._t_slave = 0.0

        # --- estado / conexión --------------------------------------------
        self.lbl_master_conn = QLabel('Maestro: ●')
        self.lbl_slave_conn = QLabel('Esclavo: ●')
        self.lbl_mirror = QLabel('Espejo: OFF')
        for w in (self.lbl_master_conn, self.lbl_slave_conn, self.lbl_mirror):
            w.setStyleSheet('font-weight: bold;')
        status = QHBoxLayout()
        status.addWidget(self.lbl_master_conn)
        status.addWidget(self.lbl_slave_conn)
        status.addStretch(1)
        status.addWidget(self.lbl_mirror)

        # --- botones grandes ----------------------------------------------
        self.btn_enable = QPushButton('▶  HABILITAR ESPEJO')
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

        self.chk_grip = QCheckBox('Espejar gripper')
        self.chk_grip.setChecked(True)
        self.chk_grip.toggled.connect(self.ros.set_mirror_gripper)

        # --- telemetría ----------------------------------------------------
        self._cells = {}
        grid = QGridLayout()
        headers = ['', 'Maestro', 'Esclavo', 'Error']
        for c, h in enumerate(headers):
            lab = QLabel(h)
            lab.setStyleSheet('font-weight: bold;')
            grid.addWidget(lab, 0, c)
        rows = list(omx.JOINT_NAMES) + ['gripper']
        for r, name in enumerate(rows, start=1):
            tag = 'Gripper' if name == 'gripper' else f'J{r}'
            grid.addWidget(QLabel(tag), r, 0)
            for c, col in enumerate(['m', 's', 'e'], start=1):
                cell = QLabel('--')
                cell.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
                cell.setStyleSheet('font-family: monospace; font-size: 14px;')
                grid.addWidget(cell, r, c)
                self._cells[(name, col)] = cell
        tele_box = QGroupBox('Telemetría  (articulaciones en °, gripper en %)')
        tele_box.setLayout(grid)

        self.lbl_ee = QLabel('Efector (cm):  maestro --   |   esclavo --')
        self.lbl_ee.setStyleSheet('font-family: monospace;')

        # --- layout --------------------------------------------------------
        root = QVBoxLayout(self)
        root.addLayout(status)
        root.addLayout(btns)
        root.addWidget(self.chk_grip)
        root.addWidget(tele_box)
        root.addWidget(self.lbl_ee)
        self.status_line = QLabel('Listo.')
        self.status_line.setStyleSheet('color:#555;')
        root.addWidget(self.status_line)

        # --- señales -------------------------------------------------------
        self.ros.states.connect(self._on_states)
        self.ros.status_message.connect(self.status_line.setText)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._check_conn)
        self._timer.start(400)

    # ------------------------------------------------------------------
    def _on_enable(self, checked):
        self.ros.set_enabled(checked)
        self.btn_enable.setText('⏸  ESPEJO ACTIVO (pulsa para pausar)' if checked
                                else '▶  HABILITAR ESPEJO')
        self.lbl_mirror.setText('Espejo: ON' if checked else 'Espejo: OFF')
        self.lbl_mirror.setStyleSheet(
            'font-weight: bold; color:%s;' % ('#2e8b30' if checked else '#b06000'))

    def _on_estop(self):
        self.btn_enable.setChecked(False)
        self.ros.estop()
        self.status_line.setText('E-STOP activado: espejo pausado, esclavo congelado.')

    def _on_states(self, master, slave):
        now = time.time()
        if master:
            self._t_master = now
        if slave:
            self._t_slave = now
        for i, name in enumerate(omx.JOINT_NAMES):
            mv = master.get(name)
            sv = slave.get(name)
            self._set(name, 'm', mv, deg=True)
            self._set(name, 's', sv, deg=True)
            if mv is not None and sv is not None:
                self._cells[(name, 'e')].setText(f'{math.degrees(sv - mv):+6.1f}')
        # gripper en %
        mg = master.get(omx.GRIPPER_JOINT)
        sg = slave.get(omx.GRIPPER_JOINT)
        self._set_pct('gripper', 'm', mg)
        self._set_pct('gripper', 's', sg)
        if mg is not None and sg is not None:
            self._cells[('gripper', 'e')].setText(
                f'{omx.gripper_m_to_percent(sg) - omx.gripper_m_to_percent(mg):+5.0f}')
        # efector
        self.lbl_ee.setText('Efector (cm):  maestro %s   |   esclavo %s'
                            % (self._ee_str(master), self._ee_str(slave)))

    def _set(self, name, col, val, deg=False):
        if val is None:
            return
        self._cells[(name, col)].setText(f'{math.degrees(val):+6.1f}' if deg
                                         else f'{val:+6.3f}')

    def _set_pct(self, name, col, meters):
        if meters is None:
            return
        self._cells[(name, col)].setText(f'{omx.gripper_m_to_percent(meters):5.0f}')

    @staticmethod
    def _ee_str(state):
        q = [state.get(n) for n in omx.JOINT_NAMES]
        if any(v is None for v in q):
            return '--'
        x, y, z, _ = fkin(q)
        return f'[{x*100:+5.1f},{y*100:+5.1f},{z*100:+5.1f}]'

    def _check_conn(self):
        now = time.time()
        self._mark(self.lbl_master_conn, 'Maestro', now - self._t_master < 1.2)
        self._mark(self.lbl_slave_conn, 'Esclavo', now - self._t_slave < 1.2)

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
