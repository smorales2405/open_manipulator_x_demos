"""
cartesian_tab.py — Pestaña "Cartesiano" (Requisito 3).

Mueve la posición X/Y/Z del efector final del robot real en pasos, con botones o
teclas, resolviendo la cinemática inversa (kinematics.ik_step) y manteniendo la
orientación phi. Incluye abrir/cerrar gripper por botón y por tecla.

Teclas (config.KEYMAP):  W/S=±X   A/D=±Y   Q/E=±Z   O/C=abrir/cerrar gripper.
"""

import math

import numpy as np

from PyQt5.QtWidgets import (QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout,
                             QLabel, QPushButton, QVBoxLayout, QWidget)

from .. import config
from ..kinematics import fkin, ik_step


class CartesianTab(QWidget):
    def __init__(self, ros, parent=None):
        super().__init__(parent)
        self.ros = ros
        self._q = np.zeros(4)          # semilla/target articular [rad]
        self._grip_m = config.GRIPPER_CLOSED_M

        # --- paso ----------------------------------------------------------
        self.step_spin = QDoubleSpinBox()
        self.step_spin.setRange(0.1, 5.0)
        self.step_spin.setSingleStep(0.1)
        self.step_spin.setValue(config.CART_STEP_M * 100.0)   # en cm
        self.step_spin.setSuffix(' cm')
        step_box = QHBoxLayout()
        step_box.addWidget(QLabel('Paso:'))
        step_box.addWidget(self.step_spin)
        step_box.addStretch(1)

        # --- botones de jog ------------------------------------------------
        grid = QGridLayout()
        self._mk_btn(grid, 'X +  (W)', 0, 1, (+1, 0, 0))
        self._mk_btn(grid, 'X −  (S)', 2, 1, (-1, 0, 0))
        self._mk_btn(grid, 'Y +  (A)', 1, 0, (0, +1, 0))
        self._mk_btn(grid, 'Y −  (D)', 1, 2, (0, -1, 0))
        self._mk_btn(grid, 'Z +  (Q)', 0, 3, (0, 0, +1))
        self._mk_btn(grid, 'Z −  (E)', 2, 3, (0, 0, -1))
        jog_box = QGroupBox('Jog del efector (robot real)')
        jog_box.setLayout(grid)

        # --- gripper -------------------------------------------------------
        self.btn_open = QPushButton('Abrir gripper  (O)')
        self.btn_close = QPushButton('Cerrar gripper  (C)')
        self.btn_open.clicked.connect(lambda: self._set_gripper(config.GRIPPER_OPEN_M))
        self.btn_close.clicked.connect(lambda: self._set_gripper(config.GRIPPER_CLOSED_M))
        grip_box = QGroupBox('Gripper')
        gl = QHBoxLayout(grip_box)
        gl.addWidget(self.btn_open)
        gl.addWidget(self.btn_close)

        # --- objetivo ------------------------------------------------------
        self.lbl_target = QLabel('—')
        self.lbl_target.setStyleSheet('font-family: monospace; font-size: 14px;')
        tgt_box = QGroupBox('Objetivo del efector')
        tl = QVBoxLayout(tgt_box)
        tl.addWidget(self.lbl_target)

        self.btn_sync = QPushButton('⟳  Tomar pose actual del robot como objetivo')
        self.btn_sync.clicked.connect(self.sync_to_robot)

        root = QVBoxLayout(self)
        root.addLayout(step_box)
        root.addWidget(jog_box)
        root.addWidget(grip_box)
        root.addWidget(tgt_box)
        root.addWidget(self.btn_sync)
        root.addStretch(1)
        self._refresh_target()

    def _mk_btn(self, grid, text, row, col, direction):
        b = QPushButton(text)
        b.setMinimumHeight(40)
        b.clicked.connect(lambda: self._jog(direction))
        grid.addWidget(b, row, col)

    # ------------------------------------------------------------------
    def _step_m(self):
        return self.step_spin.value() / 100.0

    def _jog(self, direction):
        step = self._step_m()
        d = [direction[0] * step, direction[1] * step, direction[2] * step]
        q_new = ik_step(self._q, d, hold_phi=True)
        if q_new is None:
            self.ros.status_message.emit('Cartesiano: objetivo fuera de alcance o '
                                         'límite articular; no se movió.')
            return
        self._q = np.asarray(q_new, dtype=float)
        self.ros.set_mode(config.MODE_POSITION)
        self.ros.send_arm(self._q.tolist())
        self._refresh_target()

    def _set_gripper(self, meters):
        self._grip_m = meters
        self.ros.send_gripper_m(meters)

    def _refresh_target(self):
        x, y, z, phi = fkin(self._q)
        self.lbl_target.setText(
            f'X = {x * 100:+6.1f} cm    Y = {y * 100:+6.1f} cm    '
            f'Z = {z * 100:+6.1f} cm    φ = {math.degrees(phi):+6.1f}°')

    # ------------------------------------------------------------------
    def on_state(self, state):
        # Mantiene una copia del estado por si el usuario sincroniza.
        self._last_state = dict(state)

    def sync_to_robot(self):
        st = getattr(self, '_last_state', {})
        q = [st.get(n, 0.0) for n in config.JOINT_NAMES]
        self._q = np.asarray(q, dtype=float)
        if config.GRIPPER_JOINT in st:
            self._grip_m = st[config.GRIPPER_JOINT]
        self._refresh_target()

    def handle_key(self, key_name):
        """Llamado por la ventana principal. key_name = 'W','S',... o 'Space'."""
        k = key_name.upper()
        m = config.KEYMAP
        mapping = {
            m['x+'].upper(): (+1, 0, 0), m['x-'].upper(): (-1, 0, 0),
            m['y+'].upper(): (0, +1, 0), m['y-'].upper(): (0, -1, 0),
            m['z+'].upper(): (0, 0, +1), m['z-'].upper(): (0, 0, -1),
        }
        if k in mapping:
            self._jog(mapping[k])
        elif k == m['grip_open'].upper():
            self._set_gripper(config.GRIPPER_OPEN_M)
        elif k == m['grip_close'].upper():
            self._set_gripper(config.GRIPPER_CLOSED_M)
