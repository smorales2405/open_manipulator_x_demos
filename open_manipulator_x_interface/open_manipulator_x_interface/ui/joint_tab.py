"""
joint_tab.py — Pestaña "Articular" (Requisitos 1 y 2).

Cinco sliders circulares (joint1..4 + gripper). Un conmutador permite intercalar:
  - EN VIVO        : cada cambio de slider comanda el robot real.
  - PREVISUALIZAR  : los sliders solo mueven el modelo en RViz (/joint_states_preview);
                     un botón "Enviar al robot real" comanda la configuración mostrada.
"""

import math
import time

from PyQt5.QtWidgets import (QButtonGroup, QGridLayout, QGroupBox, QHBoxLayout,
                             QLabel, QPushButton, QRadioButton, QVBoxLayout, QWidget)

from .. import config
from .circular_slider import CircularSlider, LinearSlider


class JointTab(QWidget):
    def __init__(self, ros, parent=None):
        super().__init__(parent)
        self.ros = ros
        self._last_state = {}
        self._slider_active_until = 0.0   # cooldown: no auto-sync mientras el usuario mueve sliders

        # --- sliders -------------------------------------------------------
        self.sliders = {}
        arm_row = QHBoxLayout()
        for i, name in enumerate(config.JOINT_NAMES):
            lo, hi = config.joint_limits()[name]
            notches = max(2, round(math.degrees(hi - lo) / 15.0))   # 1 marca cada 15°
            s = CircularSlider(
                f'J{i + 1}\n[{math.degrees(lo):.0f}°, {math.degrees(hi):.0f}°]',
                lo, hi, fmt=lambda v: f'{math.degrees(v):+.1f}°', notches=notches)
            s.valueChanged.connect(self._on_change)
            self.sliders[name] = s
            arm_row.addWidget(s)

        glo, ghi = sorted(config.GRIPPER_PRISMATIC)
        gs = LinearSlider('Gripper\ncerrado  ←      →  abierto', glo, ghi,
                          fmt=lambda m: f'{config.gripper_m_to_percent(m):.0f} %')
        gs.valueChanged.connect(self._on_change)
        self.sliders['gripper'] = gs

        gs.setMaximumWidth(200)
        grip_row = QHBoxLayout()
        grip_row.addStretch(1)
        grip_row.addWidget(gs)
        grip_row.addStretch(1)

        sliders_box = QGroupBox('Sliders articulares')
        sb_lay = QVBoxLayout(sliders_box)
        sb_lay.addLayout(arm_row)
        sb_lay.addLayout(grip_row)

        # --- modo en vivo / preview ---------------------------------------
        self.rb_live = QRadioButton('En vivo (comanda el robot real)')
        self.rb_preview = QRadioButton('Previsualizar (solo modelo RViz)')
        self.rb_live.setChecked(True)
        group = QButtonGroup(self)
        group.addButton(self.rb_live)
        group.addButton(self.rb_preview)
        self.rb_live.toggled.connect(self._on_mode_toggle)

        self.btn_send = QPushButton('➤  Enviar al robot real')
        self.btn_send.clicked.connect(self._send_to_robot)
        self.btn_send.setEnabled(False)
        self.btn_send.setStyleSheet('font-weight: bold; padding: 8px;')

        mode_box = QGroupBox('Modo')
        mode_lay = QGridLayout(mode_box)
        mode_lay.addWidget(self.rb_live, 0, 0)
        mode_lay.addWidget(self.rb_preview, 1, 0)
        mode_lay.addWidget(self.btn_send, 0, 1, 2, 1)   # ocupa las 2 filas

        self.hint = QLabel('En vivo: mover un slider mueve el robot. '
                           'Previsualizar: ajusta y pulsa «Enviar al robot real».')
        self.hint.setWordWrap(True)
        self.hint.setStyleSheet('color: #555;')

        root = QVBoxLayout(self)
        root.addWidget(sliders_box)
        root.addWidget(mode_box)
        root.addWidget(self.hint)
        root.addStretch(1)

    # ------------------------------------------------------------------
    def _arm_q(self):
        return [self.sliders[n].value() for n in config.JOINT_NAMES]

    def _gripper_m(self):
        return self.sliders['gripper'].value()

    def _is_live(self):
        return self.rb_live.isChecked()

    def _on_change(self, _value):
        self._slider_active_until = time.time() + 2.0   # bloquea auto-sync 2 s
        if self._is_live():
            self.ros.send_arm(self._arm_q())
            self.ros.send_gripper_m(self._gripper_m())
        else:
            self.ros.publish_preview(self._arm_q(), self._gripper_m())

    def _on_mode_toggle(self, live):
        self.btn_send.setEnabled(not live)
        if live:
            self.ros.set_mode(config.MODE_POSITION)
        else:
            # Mostrar de inmediato la configuración actual en el modelo preview.
            self.ros.publish_preview(self._arm_q(), self._gripper_m())

    def _send_to_robot(self):
        self.ros.set_mode(config.MODE_POSITION)
        self.ros.send_arm(self._arm_q())
        self.ros.send_gripper_m(self._gripper_m())

    # ------------------------------------------------------------------
    def on_state(self, state):
        self._last_state = dict(state)
        # En modo en vivo, sincroniza sliders con el robot salvo que el usuario
        # haya movido un slider recientemente (cooldown de 2 s).
        if self._is_live() and time.time() > self._slider_active_until:
            for name in config.JOINT_NAMES:
                if name in state:
                    self.sliders[name].set_value_silent(state[name])
            if config.GRIPPER_JOINT in state:
                self.sliders['gripper'].set_value_silent(state[config.GRIPPER_JOINT])

    def preview_zero(self):
        """Pone los sliders del brazo a 0 y actualiza SOLO el modelo de RViz
        (sin comandar el robot real). Usado por «Ir a cero» en previsualización."""
        for name in config.JOINT_NAMES:
            self.sliders[name].set_value_silent(0.0)
        self.ros.publish_preview([0.0, 0.0, 0.0, 0.0], self._gripper_m())

    def sliders_to_zero(self):
        """Mueve los sliders J1-J4 visualmente a 0° sin emitir comandos al robot.
        Bloquea el auto-sync durante el tiempo de homing para que no "persigan" al robot."""
        for name in config.JOINT_NAMES:
            self.sliders[name].set_value_silent(0.0)
        self._slider_active_until = time.time() + config.HOME_TIME_S + 0.5
