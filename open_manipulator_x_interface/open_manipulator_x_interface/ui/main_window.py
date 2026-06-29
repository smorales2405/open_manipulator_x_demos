"""
main_window.py — Ventana única de la interfaz.

Disposición:
  - Barra superior: estado de conexión, modo, y botones globales
    (Ir a cero, Stop, Torque ON/OFF).
  - Centro: pestañas (Articular / Cartesiano / Teach & Waypoints) a la izquierda
    y panel de telemetría fijo a la derecha.
  - Barra inferior: mensajes de estado.
"""

import time

from PyQt5.QtCore import QEvent, Qt, QTimer
from PyQt5.QtWidgets import (QHBoxLayout, QLabel, QMainWindow, QPushButton,
                             QTabWidget, QVBoxLayout, QWidget)

from .. import config
from .cartesian_tab import CartesianTab
from .joint_tab import JointTab
from .teach_tab import TeachTab
from .telemetry_panel import TelemetryPanel


class MainWindow(QMainWindow):
    def __init__(self, ros):
        super().__init__()
        self.ros = ros
        self.setWindowTitle('OpenMANIPULATOR-X — Interfaz de Taller')
        self.resize(1080, 640)
        self._last_state_time = 0.0

        # --- barra superior ------------------------------------------------
        self.conn_label = QLabel('● Esperando…')
        self.conn_label.setStyleSheet('font-weight: bold; color: #b06000;')
        self.mode_label = QLabel('Modo: POSICIÓN')
        self.mode_label.setStyleSheet('font-weight: bold;')

        btn_home = QPushButton('⟲  Ir a cero')
        btn_home.clicked.connect(self._on_home)
        btn_stop = QPushButton('■  STOP')
        btn_stop.setStyleSheet('background:#c0392b; color:white; font-weight:bold;')
        btn_stop.clicked.connect(self.ros.stop)
        self.btn_torque = QPushButton('Torque: ON')
        self.btn_torque.setCheckable(True)
        self.btn_torque.setChecked(True)
        self.btn_torque.clicked.connect(self._on_torque)

        top = QHBoxLayout()
        top.addWidget(self.conn_label)
        top.addSpacing(20)
        top.addWidget(self.mode_label)
        top.addStretch(1)
        top.addWidget(btn_home)
        top.addWidget(self.btn_torque)
        top.addWidget(btn_stop)
        top_w = QWidget()
        top_w.setLayout(top)

        # --- pestañas + telemetría ----------------------------------------
        self.joint_tab = JointTab(ros)
        self.cartesian_tab = CartesianTab(ros)
        self.teach_tab = TeachTab(ros)
        self.tabs = QTabWidget()
        self.tabs.addTab(self.joint_tab, 'Articular')
        self.tabs.addTab(self.cartesian_tab, 'Cartesiano')
        self.tabs.addTab(self.teach_tab, 'Teach & Waypoints')
        self._all_tabs = [self.joint_tab, self.cartesian_tab, self.teach_tab]

        self.telemetry = TelemetryPanel()

        center = QHBoxLayout()
        center.addWidget(self.tabs, 3)
        center.addWidget(self.telemetry, 1)

        root = QVBoxLayout()
        root.addWidget(top_w)
        root.addLayout(center)
        central = QWidget()
        central.setLayout(root)
        self.setCentralWidget(central)
        self.statusBar().showMessage('Listo.')

        # --- señales -------------------------------------------------------
        self.ros.joint_state_received.connect(self._on_state)
        self.ros.status_message.connect(self.statusBar().showMessage)
        self.joint_tab.rb_live.toggled.connect(
            lambda live: self._set_mode_label('POSICIÓN' if live else 'PREVIEW'))
        self.teach_tab.btn_teach.toggled.connect(
            lambda on: self._set_mode_label('TEACH (libre)' if on else 'POSICIÓN'))
        # Al cambiar de pestaña, corta cualquier jog continuo del modo cartesiano.
        self.tabs.currentChanged.connect(
            lambda _i: self.cartesian_tab.stop_continuous())

        # --- watchdog de conexión -----------------------------------------
        self._conn_timer = QTimer(self)
        self._conn_timer.timeout.connect(self._check_conn)
        self._conn_timer.start(500)

    # ------------------------------------------------------------------
    def _on_state(self, state):
        self._last_state_time = time.time()
        self.telemetry.update_state(state)
        for tab in self._all_tabs:
            tab.on_state(state)
        # Si NO se está previsualizando, el modelo de RViz refleja el robot real.
        if not self.joint_tab.rb_preview.isChecked():
            self.ros.relay_preview(state)

    def _on_home(self):
        # En previsualización, «Ir a cero» solo mueve el modelo de RViz.
        if self.joint_tab.rb_preview.isChecked():
            self.joint_tab.preview_zero()
            self.statusBar().showMessage('Ir a cero: solo modelo (previsualización).')
        else:
            self.ros.go_home()
            self.joint_tab.sliders_to_zero()

    def _on_torque(self, checked):
        self.ros.set_torque(checked)
        self.btn_torque.setText('Torque: ON' if checked else 'Torque: OFF')
        if not checked:
            self._set_mode_label('TEACH (libre)')

    def _set_mode_label(self, text):
        self.mode_label.setText(f'Modo: {text}')

    def _check_conn(self):
        if time.time() - self._last_state_time < 1.5:
            self.conn_label.setText('● Conectado')
            self.conn_label.setStyleSheet('font-weight: bold; color: #2e8b30;')
        else:
            self.conn_label.setText('● Sin datos de /joint_states')
            self.conn_label.setStyleSheet('font-weight: bold; color: #b06000;')

    # ------------------------------------------------------------------
    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        key_name = 'Space' if event.key() == Qt.Key_Space else event.text().upper()
        if not key_name:
            return super().keyPressEvent(event)
        if key_name == config.KEYMAP['home'].upper():
            self._on_home()
            return
        if key_name == config.KEYMAP['stop'].upper():
            self.ros.stop()
            return
        if self.tabs.currentWidget() is self.cartesian_tab:
            if self.cartesian_tab.key_pressed(key_name):
                return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        # Ignora los 'release' del auto-repeat del SO; solo cuenta el real.
        if event.isAutoRepeat():
            return
        key_name = 'Space' if event.key() == Qt.Key_Space else event.text().upper()
        if key_name and self.tabs.currentWidget() is self.cartesian_tab:
            self.cartesian_tab.key_released(key_name)
            return
        super().keyReleaseEvent(event)

    def changeEvent(self, event):
        # Si la ventana pierde el foco con una tecla mantenida, detén el jog.
        if event.type() == QEvent.WindowDeactivate:
            self.cartesian_tab.stop_continuous()
        super().changeEvent(event)

    def closeEvent(self, event):
        self.cartesian_tab.stop_continuous()
        self.ros.shutdown()
        super().closeEvent(event)
