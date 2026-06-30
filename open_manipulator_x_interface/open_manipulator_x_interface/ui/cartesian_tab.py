"""
cartesian_tab.py — Pestaña "Cartesiano" (Requisito 3).

Mueve la posición X/Y/Z del efector final del robot real en pasos, con botones o
teclas, resolviendo la cinemática inversa (kinematics.ik_step) y manteniendo la
orientación phi. Incluye abrir/cerrar gripper por botón y por tecla.

Teclas (config.KEYMAP):  W/S=±X   A/D=±Y   Q/E=±Z   O/C=abrir/cerrar gripper.
Mantener presionada una tecla de movimiento produce jog continuo (no hay que
pulsar repetidamente); se pueden combinar varias para moverse en diagonal.
"""

import math

import numpy as np

from PyQt5.QtCore import Qt, QTimer
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

        # Jog continuo al mantener tecla: conjunto de teclas presionadas + timer.
        m = config.KEYMAP
        self._dir_map = {
            m['x+'].upper(): (+1, 0, 0), m['x-'].upper(): (-1, 0, 0),
            m['y+'].upper(): (0, +1, 0), m['y-'].upper(): (0, -1, 0),
            m['z+'].upper(): (0, 0, +1), m['z-'].upper(): (0, 0, -1),
        }
        self._held = set()
        self._jog_timer = QTimer(self)
        self._jog_timer.setInterval(config.CART_JOG_INTERVAL_MS)
        self._jog_timer.timeout.connect(self._continuous_jog)

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
        jog_box = QGroupBox('Movimiento del efector final')
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

        root = QVBoxLayout(self)
        root.addLayout(step_box)
        root.addWidget(jog_box)
        root.addWidget(grip_box)
        root.addWidget(tgt_box)
        root.addStretch(1)
        self._refresh_target()

    def _mk_btn(self, grid, text, row, col, direction):
        # Separar "X +  (W)" → coord "X +" (negrita) y tecla "(W)" (normal)
        if '(' in text:
            coord, rest = text.split('(', 1)
            key_str = f'({rest}'
        else:
            coord, key_str = text, ''
        coord = coord.strip()
        b = QPushButton()
        b.setMinimumHeight(48)
        lbl = QLabel(
            f'<b style="font-size:13px">{coord}</b>'
            f'&nbsp;&nbsp;<span style="font-size:11px; font-weight:normal;">{key_str}</span>')
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setAttribute(Qt.WA_TransparentForMouseEvents)
        inner = QVBoxLayout(b)
        inner.setContentsMargins(2, 2, 2, 2)
        inner.addWidget(lbl)
        b.clicked.connect(lambda: self._jog_once(direction))
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
        self.ros.send_arm(self._q.tolist())
        self._refresh_target()

    def _jog_once(self, direction):
        """Un único paso (clic de botón): asegura modo posición y mueve una vez."""
        self.ros.set_mode(config.MODE_POSITION)
        self._jog(direction)

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
        # Actualiza la semilla de IK con la posición real del robot mientras no
        # haya un jog activo, de modo que el jog siempre parte de donde está el robot.
        if not self._held:
            q = [state.get(n, 0.0) for n in config.JOINT_NAMES]
            self._q = np.asarray(q, dtype=float)
            if config.GRIPPER_JOINT in state:
                self._grip_m = state[config.GRIPPER_JOINT]
            self._refresh_target()

    # ------------------------------------------------------------------
    #  Jog continuo por teclado (mantener presionado)
    # ------------------------------------------------------------------
    def key_pressed(self, key_name):
        """Tecla presionada (sin auto-repeat). Devuelve True si la consumió."""
        k = key_name.upper()
        if k in self._dir_map:
            starting = not self._held
            self._held.add(k)
            if starting:
                # Primer paso inmediato + arranca el jog continuo mientras se mantenga.
                self.ros.set_mode(config.MODE_POSITION)
                self._continuous_jog()
                self._jog_timer.start()
            return True
        if k == config.KEYMAP['grip_open'].upper():
            self._set_gripper(config.GRIPPER_OPEN_M)
            return True
        if k == config.KEYMAP['grip_close'].upper():
            self._set_gripper(config.GRIPPER_CLOSED_M)
            return True
        return False

    def key_released(self, key_name):
        """Tecla soltada (release real, sin auto-repeat)."""
        self._held.discard(key_name.upper())
        if not self._held:
            self._jog_timer.stop()

    def stop_continuous(self):
        """Detiene todo jog continuo (al cambiar de pestaña o perder el foco)."""
        self._held.clear()
        self._jog_timer.stop()

    def _continuous_jog(self):
        if not self._held:
            self._jog_timer.stop()
            return
        sx = sy = sz = 0
        for k in self._held:
            dx, dy, dz = self._dir_map[k]
            sx += dx
            sy += dy
            sz += dz
        if sx or sy or sz:
            self._jog((sx, sy, sz))
