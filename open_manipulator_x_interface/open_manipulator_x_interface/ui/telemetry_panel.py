"""
telemetry_panel.py — Panel lateral fijo con las lecturas del robot.

Muestra los ángulos articulares en GRADOS sexagesimales, la apertura del gripper
en %, y la posición del efector final X/Y/Z en CENTÍMETROS (más phi en grados),
calculada con la cinemática directa (kinematics.fkin).
"""

import math

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QFrame, QGridLayout, QLabel, QVBoxLayout, QWidget)

from .. import config
from ..kinematics import fkin


class TelemetryPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumWidth(240)
        self._labels = {}

        root = QVBoxLayout(self)
        root.setAlignment(Qt.AlignTop)

        root.addWidget(self._header('ARTICULACIONES  [°]'))
        grid_j = QGridLayout()
        for i, name in enumerate(config.JOINT_NAMES):
            tag = QLabel(f'J{i + 1}')
            val = QLabel('   --')
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setStyleSheet('font-family: monospace; font-size: 15px;')
            grid_j.addWidget(tag, i, 0)
            grid_j.addWidget(val, i, 1)
            self._labels[name] = val
        root.addLayout(grid_j)

        grid_g = QGridLayout()
        tag = QLabel('Gripper')
        val = QLabel('   --')
        val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        val.setStyleSheet('font-family: monospace; font-size: 15px;')
        grid_g.addWidget(tag, 0, 0)
        grid_g.addWidget(val, 0, 1)
        self._labels['gripper'] = val
        root.addLayout(grid_g)

        root.addSpacing(10)
        root.addWidget(self._header('EFECTOR FINAL  [cm]'))
        grid_e = QGridLayout()
        for i, axis in enumerate(['X', 'Y', 'Z', 'φ']):
            tag = QLabel(axis)
            val = QLabel('   --')
            val.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val.setStyleSheet('font-family: monospace; font-size: 15px;')
            grid_e.addWidget(tag, i, 0)
            grid_e.addWidget(val, i, 1)
            self._labels[axis] = val
        root.addLayout(grid_e)

    @staticmethod
    def _header(text):
        lab = QLabel(text)
        lab.setStyleSheet('font-weight: bold; color: #3070b0;')
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        wrap = QWidget()
        v = QVBoxLayout(wrap)
        v.setContentsMargins(0, 0, 0, 0)
        v.addWidget(lab)
        v.addWidget(line)
        return wrap

    def update_state(self, state):
        """state: dict {nombre_joint: posición}. Brazo en rad, gripper en m."""
        q = []
        for i, name in enumerate(config.JOINT_NAMES):
            if name in state:
                q.append(state[name])
                self._labels[name].setText(f'{math.degrees(state[name]):+7.1f}')
            else:
                q.append(0.0)

        if config.GRIPPER_JOINT in state:
            pct = config.gripper_m_to_percent(state[config.GRIPPER_JOINT])
            self._labels['gripper'].setText(f'{pct:6.0f} %')

        if len(q) == 4:
            x, y, z, phi = fkin(q)
            self._labels['X'].setText(f'{x * 100.0:+7.1f}')
            self._labels['Y'].setText(f'{y * 100.0:+7.1f}')
            self._labels['Z'].setText(f'{z * 100.0:+7.1f}')
            self._labels['φ'].setText(f'{math.degrees(phi):+7.1f}°')
