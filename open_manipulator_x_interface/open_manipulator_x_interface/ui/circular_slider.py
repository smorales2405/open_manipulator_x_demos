"""
circular_slider.py — Slider circular (QDial) con etiqueta numérica.

Genérico: trabaja en unidades arbitrarias [vmin, vmax] (rad para las
articulaciones, m para el gripper). Un formateador opcional decide cómo se
muestra el valor (p. ej. grados o porcentaje).
"""

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import QDial, QLabel, QSlider, QVBoxLayout, QWidget


class CircularSlider(QWidget):
    valueChanged = pyqtSignal(float)   # en las unidades de vmin/vmax

    # Pasos del dial por marca (notch). Da resolución fina dentro de cada marca,
    # manteniendo una línea visible por marca (p. ej. cada 15°).
    SUB = 30

    def __init__(self, name, vmin, vmax, fmt=None, notches=12, parent=None):
        super().__init__(parent)
        self.vmin = float(vmin)
        self.vmax = float(vmax)
        self._fmt = fmt or (lambda v: f'{v:.3f}')

        self.title = QLabel(name)
        self.title.setAlignment(Qt.AlignCenter)
        self.title.setStyleSheet('font-weight: bold;')

        # range = notches * SUB, con singleStep == pageStep == SUB hace que QDial
        # dibuje exactamente una marca por cada SUB pasos (una marca por notch).
        self.steps = max(self.SUB, int(notches) * self.SUB)
        self.dial = QDial()
        self.dial.setRange(0, self.steps)
        self.dial.setSingleStep(self.SUB)
        self.dial.setPageStep(self.SUB)
        self.dial.setNotchesVisible(True)
        self.dial.setWrapping(False)
        self.dial.setFixedSize(110, 110)
        self.dial.valueChanged.connect(self._on_dial)

        self.value_label = QLabel()
        self.value_label.setAlignment(Qt.AlignCenter)
        self.value_label.setStyleSheet('font-size: 14px;')

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.addWidget(self.title)
        lay.addWidget(self.dial, alignment=Qt.AlignCenter)
        lay.addWidget(self.value_label)

        self.set_value_silent(0.0 if self.vmin <= 0.0 <= self.vmax else self.vmin)

    # -- conversión ------------------------------------------------------
    def _to_value(self, step):
        return self.vmin + (step / self.steps) * (self.vmax - self.vmin)

    def _to_step(self, value):
        value = max(self.vmin, min(self.vmax, value))
        return int(round((value - self.vmin) / (self.vmax - self.vmin) * self.steps))

    # -- eventos ---------------------------------------------------------
    def _on_dial(self, step):
        v = self._to_value(step)
        self.value_label.setText(self._fmt(v))
        self.valueChanged.emit(v)

    def value(self):
        return self._to_value(self.dial.value())

    def set_value_silent(self, value):
        """Mueve el dial sin emitir valueChanged (para sincronizar con el robot)."""
        blocked = self.dial.blockSignals(True)
        self.dial.setValue(self._to_step(value))
        self.dial.blockSignals(blocked)
        self.value_label.setText(self._fmt(self._to_value(self.dial.value())))

    def setEnabled(self, enabled):  # noqa: N802 (Qt API)
        super().setEnabled(enabled)
        self.dial.setEnabled(enabled)


class LinearSlider(QWidget):
    """Slider lineal horizontal con etiqueta numérica. Misma API que CircularSlider."""
    valueChanged = pyqtSignal(float)
    STEPS = 1000

    def __init__(self, name, vmin, vmax, fmt=None, parent=None):
        super().__init__(parent)
        self.vmin = float(vmin)
        self.vmax = float(vmax)
        self._fmt = fmt or (lambda v: f'{v:.3f}')

        self.title = QLabel(name)
        self.title.setAlignment(Qt.AlignCenter)
        self.title.setStyleSheet('font-weight: bold;')
        self.title.setWordWrap(True)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, self.STEPS)
        self.slider.valueChanged.connect(self._on_slider)

        self.value_label = QLabel()
        self.value_label.setAlignment(Qt.AlignCenter)
        self.value_label.setStyleSheet('font-size: 14px;')

        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        lay.addWidget(self.title)
        lay.addWidget(self.slider)
        lay.addWidget(self.value_label)

        self.set_value_silent(0.0 if vmin <= 0.0 <= vmax else vmin)

    def _to_value(self, step):
        return self.vmin + (step / self.STEPS) * (self.vmax - self.vmin)

    def _to_step(self, value):
        value = max(self.vmin, min(self.vmax, value))
        return int(round((value - self.vmin) / (self.vmax - self.vmin) * self.STEPS))

    def _on_slider(self, step):
        v = self._to_value(step)
        self.value_label.setText(self._fmt(v))
        self.valueChanged.emit(v)

    def value(self):
        return self._to_value(self.slider.value())

    def set_value_silent(self, value):
        blocked = self.slider.blockSignals(True)
        self.slider.setValue(self._to_step(value))
        self.slider.blockSignals(blocked)
        self.value_label.setText(self._fmt(self._to_value(self.slider.value())))

    def setEnabled(self, enabled):  # noqa: N802
        super().setEnabled(enabled)
        self.slider.setEnabled(enabled)
