"""
teach_tab.py — Pestaña "Teach & Waypoints" (Requisito 4).

Flujo de taller:
  1) Activar "Modo libre" -> el brazo queda suelto (torque OFF, backdrivable).
  2) Mover el robot a mano y pulsar "Guardar waypoint" en cada pose deseada
     (se captura la configuración articular + acción del gripper).
  3) "Ejecutar trayectoria" -> el robot recupera torque y recorre todos los
     waypoints en orden, accionando el gripper en cada uno.
"""

import math

from PyQt5.QtWidgets import (QComboBox, QDoubleSpinBox, QFileDialog, QGroupBox,
                             QHBoxLayout, QLabel, QListWidget, QMessageBox,
                             QPushButton, QVBoxLayout, QWidget)

from .. import config

try:
    import yaml
    _HAVE_YAML = True
except ImportError:
    _HAVE_YAML = False


class TeachTab(QWidget):
    def __init__(self, ros, parent=None):
        super().__init__(parent)
        self.ros = ros
        self._last_state = {}
        self._waypoints = []          # lista de dicts {q, gripper, time}
        self._teach_on = False

        # --- modo libre ----------------------------------------------------
        self.btn_teach = QPushButton('🟠  Activar modo libre (Teach / torque OFF)')
        self.btn_teach.setCheckable(True)
        self.btn_teach.clicked.connect(self._toggle_teach)
        self.btn_teach.setStyleSheet('padding: 8px; font-weight: bold;')
        self.warn = QLabel('En modo libre el brazo queda SUELTO: sostenlo con la '
                           'mano antes de activarlo para que no caiga.')
        self.warn.setWordWrap(True)
        self.warn.setStyleSheet('color: #b06000;')
        teach_box = QGroupBox('1) Modo libre')
        tl = QVBoxLayout(teach_box)
        tl.addWidget(self.btn_teach)
        tl.addWidget(self.warn)

        # --- captura -------------------------------------------------------
        self.grip_combo = QComboBox()
        self.grip_combo.addItems(['Gripper: mantener', 'Gripper: abrir', 'Gripper: cerrar'])
        self.time_spin = QDoubleSpinBox()
        self.time_spin.setRange(0.3, 30.0)
        self.time_spin.setSingleStep(0.5)
        self.time_spin.setValue(2.5)
        self.time_spin.setSuffix(' s')
        self.btn_save = QPushButton('➕  Guardar waypoint')
        self.btn_save.clicked.connect(self._save_waypoint)
        cap_box = QGroupBox('2) Capturar waypoints')
        cl = QHBoxLayout(cap_box)
        cl.addWidget(self.grip_combo)
        cl.addWidget(QLabel('t al punto:'))
        cl.addWidget(self.time_spin)
        cl.addWidget(self.btn_save)

        # --- lista ---------------------------------------------------------
        self.list = QListWidget()
        btn_up = QPushButton('▲ Subir')
        btn_down = QPushButton('▼ Bajar')
        btn_del = QPushButton('🗑 Borrar')
        btn_clear = QPushButton('Limpiar')
        btn_up.clicked.connect(lambda: self._move(-1))
        btn_down.clicked.connect(lambda: self._move(+1))
        btn_del.clicked.connect(self._delete)
        btn_clear.clicked.connect(self._clear)
        list_btns = QHBoxLayout()
        for b in (btn_up, btn_down, btn_del, btn_clear):
            list_btns.addWidget(b)
        list_box = QGroupBox('Waypoints')
        ll = QVBoxLayout(list_box)
        ll.addWidget(self.list)
        ll.addLayout(list_btns)

        # --- ejecutar / archivo -------------------------------------------
        self.btn_exec = QPushButton('▶  Ejecutar trayectoria')
        self.btn_exec.setStyleSheet('padding: 8px; font-weight: bold;')
        self.btn_exec.clicked.connect(self._execute)
        btn_savef = QPushButton('💾 Guardar…')
        btn_loadf = QPushButton('📂 Cargar…')
        btn_savef.clicked.connect(self._save_file)
        btn_loadf.clicked.connect(self._load_file)
        exec_box = QGroupBox('3) Ejecutar')
        el = QHBoxLayout(exec_box)
        el.addWidget(self.btn_exec)
        el.addWidget(btn_savef)
        el.addWidget(btn_loadf)

        root = QVBoxLayout(self)
        root.addWidget(teach_box)
        root.addWidget(cap_box)
        root.addWidget(list_box)
        root.addWidget(exec_box)

    # ------------------------------------------------------------------
    def on_state(self, state):
        self._last_state = dict(state)

    def _toggle_teach(self, checked):
        self._teach_on = checked
        if checked:
            self.ros.set_mode(config.MODE_TEACH)
            self.btn_teach.setText('🟢  Modo libre ACTIVO — pulsa para fijar (torque ON)')
        else:
            self.ros.set_mode(config.MODE_POSITION)
            self.btn_teach.setText('🟠  Activar modo libre (Teach / torque OFF)')

    def _resolve_gripper(self):
        choice = self.grip_combo.currentIndex()
        if choice == 1:
            return config.GRIPPER_OPEN_M
        if choice == 2:
            return config.GRIPPER_CLOSED_M
        return self._last_state.get(config.GRIPPER_JOINT, config.GRIPPER_CLOSED_M)

    def _save_waypoint(self):
        if not all(n in self._last_state for n in config.JOINT_NAMES):
            QMessageBox.warning(self, 'Sin estado',
                                'Aún no llega /joint_states del robot.')
            return
        q = [self._last_state[n] for n in config.JOINT_NAMES]
        wp = {'q': q, 'gripper': self._resolve_gripper(), 'time': self.time_spin.value()}
        self._waypoints.append(wp)
        self._refresh_list()

    def _refresh_list(self):
        self.list.clear()
        for i, wp in enumerate(self._waypoints):
            degs = ' '.join(f'{math.degrees(v):+.0f}' for v in wp['q'])
            pct = config.gripper_m_to_percent(wp['gripper'])
            self.list.addItem(f'WP{i + 1}:  [{degs}]°   grip {pct:.0f}%   t={wp["time"]:.1f}s')

    def _selected(self):
        row = self.list.currentRow()
        return row if 0 <= row < len(self._waypoints) else None

    def _move(self, delta):
        i = self._selected()
        if i is None:
            return
        j = i + delta
        if 0 <= j < len(self._waypoints):
            self._waypoints[i], self._waypoints[j] = self._waypoints[j], self._waypoints[i]
            self._refresh_list()
            self.list.setCurrentRow(j)

    def _delete(self):
        i = self._selected()
        if i is not None:
            self._waypoints.pop(i)
            self._refresh_list()

    def _clear(self):
        self._waypoints.clear()
        self._refresh_list()

    def _execute(self):
        if len(self._waypoints) < 1:
            QMessageBox.information(self, 'Sin waypoints',
                                    'Guarda al menos un waypoint.')
            return
        if self._teach_on:
            self.btn_teach.setChecked(False)
            self._toggle_teach(False)
        self.ros.execute_trajectory(self._waypoints)

    # -- archivo --------------------------------------------------------
    def _save_file(self):
        if not _HAVE_YAML:
            QMessageBox.warning(self, 'Sin PyYAML', 'PyYAML no está disponible.')
            return
        path, _ = QFileDialog.getSaveFileName(self, 'Guardar waypoints',
                                              'waypoints.yaml', 'YAML (*.yaml)')
        if not path:
            return
        data = {'waypoints': [{'q': [float(v) for v in wp['q']],
                               'gripper': float(wp['gripper']),
                               'time': float(wp['time'])} for wp in self._waypoints]}
        with open(path, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        self.ros.status_message.emit(f'Waypoints guardados en {path}')

    def _load_file(self):
        if not _HAVE_YAML:
            QMessageBox.warning(self, 'Sin PyYAML', 'PyYAML no está disponible.')
            return
        path, _ = QFileDialog.getOpenFileName(self, 'Cargar waypoints',
                                              '', 'YAML (*.yaml)')
        if not path:
            return
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        self._waypoints = [{'q': list(wp['q']),
                            'gripper': float(wp.get('gripper', config.GRIPPER_CLOSED_M)),
                            'time': float(wp.get('time', 2.5))}
                           for wp in data.get('waypoints', [])]
        self._refresh_list()
