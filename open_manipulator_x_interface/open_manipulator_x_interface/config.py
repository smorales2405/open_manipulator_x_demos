"""
config.py — Fuente única de verdad de la interfaz.

Aquí viven los LÍMITES ARTICULARES, los IDs/constantes Dynamixel y los
parámetros de la GUI. Para un taller, edita SOLO este archivo para ajustar
límites, pasos de jog, teclas, etc.

Convención de signos
--------------------
La cinemática directa (kinematics.fkin) usa la MISMA convención que
`hw_fkin_node.cpp` del paquete open_manipulator_x_torque_control:

    q[rad] = ENCODER_SIGN * (tick - ZERO_TICK) * POS_UNIT_RAD

con ENCODER_SIGN = (+1, -1, -1, -1) y ZERO_TICK = 2048. Como ENCODER_SIGN es
±1, la conversión inversa (rad -> tick) usa exactamente el mismo factor. Así, q=0
en todas las articulaciones corresponde al tick 2048 (posición "cero" del robot)
y la FK coincide con la del laboratorio.
"""

import math

# ---------------------------------------------------------------------------
# Articulaciones del brazo (4 GDL) + gripper
# ---------------------------------------------------------------------------
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']
GRIPPER_JOINT = 'gripper_left_joint'
ALL_JOINT_NAMES = JOINT_NAMES + [GRIPPER_JOINT]

# IDs Dynamixel (XM430-W350). Brazo 11-14, gripper (dxl5) = 15.
DXL_IDS = {'joint1': 11, 'joint2': 12, 'joint3': 13, 'joint4': 14, 'gripper': 15}
ARM_IDS = [DXL_IDS[n] for n in JOINT_NAMES]
GRIPPER_ID = DXL_IDS['gripper']

# ---------------------------------------------------------------------------
# LÍMITES ARTICULARES [rad]  (seed = open_manipulator_x.urdf.xacro)
#   >>> EDITA AQUÍ para cambiar los límites del taller <<<
# ---------------------------------------------------------------------------
JOINT_LIMITS = {
    'joint1': (-math.pi, math.pi),   # ±180°
    'joint2': (-1.50, 1.50),         # ≈ ±86°
    'joint3': (-1.50, 1.40),
    'joint4': (-1.70, 1.97),
}

# Límites "taller" más conservadores (opcional). Activar con USE_WORKSHOP_LIMITS.
USE_WORKSHOP_LIMITS = False
WORKSHOP_JOINT_LIMITS = {
    'joint1': (-2.6, 2.6),
    'joint2': (-1.0, 1.3),
    'joint3': (-1.2, 1.2),
    'joint4': (-1.5, 1.7),
}


def joint_limits():
    """Devuelve los límites activos (normales o de taller)."""
    return WORKSHOP_JOINT_LIMITS if USE_WORKSHOP_LIMITS else JOINT_LIMITS


def clamp_joint(name, value):
    lo, hi = joint_limits()[name]
    return max(lo, min(hi, value))


# ---------------------------------------------------------------------------
# Gripper: motor revoluto (rad) <-> apertura prismática (m)
#   Mapeo lineal tomado del ros2_control oficial (open_manipulator_x_system):
#     prismático -0.010 m (cerrado) <-> motor  0.92 rad
#     prismático  0.019 m (abierto) <-> motor -1.52 rad
#   NOTA: verifica/calibra en el robot real antes del taller. Si el gripper
#   se mueve al revés, intercambia OPEN/CLOSE o cambia el signo.
# ---------------------------------------------------------------------------
GRIPPER_PRISMATIC = (-0.010, 0.019)        # (cerrado, abierto) [m]
GRIPPER_MOTOR_RANGE = (0.92, -1.52)        # (cerrado, abierto) [rad motor]
GRIPPER_CLOSED_M = GRIPPER_PRISMATIC[0]
GRIPPER_OPEN_M = GRIPPER_PRISMATIC[1]


def gripper_m_to_motor_rad(m):
    """Apertura prismática [m] -> ángulo del motor del gripper [rad]."""
    m0, m1 = GRIPPER_PRISMATIC
    r0, r1 = GRIPPER_MOTOR_RANGE
    m = max(min(m, max(m0, m1)), min(m0, m1))
    return r0 + (m - m0) * (r1 - r0) / (m1 - m0)


def gripper_motor_rad_to_m(rad):
    """Ángulo del motor del gripper [rad] -> apertura prismática [m]."""
    m0, m1 = GRIPPER_PRISMATIC
    r0, r1 = GRIPPER_MOTOR_RANGE
    return m0 + (rad - r0) * (m1 - m0) / (r1 - r0)


def gripper_m_to_percent(m):
    """Apertura [m] -> porcentaje 0 (cerrado) .. 100 (abierto)."""
    m0, m1 = GRIPPER_PRISMATIC
    pct = 100.0 * (m - m0) / (m1 - m0)
    return max(0.0, min(100.0, pct))


def gripper_percent_to_m(pct):
    m0, m1 = GRIPPER_PRISMATIC
    pct = max(0.0, min(100.0, pct))
    return m0 + (pct / 100.0) * (m1 - m0)


# ---------------------------------------------------------------------------
# Dynamixel: conversión rad <-> tick  (XM430: 4096 ticks/rev, cero en 2048)
# ---------------------------------------------------------------------------
POS_UNIT_RAD = 2.0 * math.pi / 4096.0       # rad por tick
VEL_UNIT_RAD_S = 0.229 * 2.0 * math.pi / 60.0
ZERO_TICK = 2048
# Convención de la FK (hw_fkin_node.cpp): {+1,-1,-1,-1} para joint1..joint4
ENCODER_SIGN = {'joint1': +1.0, 'joint2': -1.0, 'joint3': -1.0, 'joint4': -1.0}
GRIPPER_ENCODER_SIGN = +1.0


def _wrapped_tick_diff(raw, zero=ZERO_TICK):
    d = raw - zero
    while d > 2048:
        d -= 4096
    while d < -2048:
        d += 4096
    return d


def arm_ticks_to_rad(name, ticks):
    return ENCODER_SIGN[name] * _wrapped_tick_diff(int(ticks)) * POS_UNIT_RAD


def arm_rad_to_ticks(name, rad):
    t = int(round(ZERO_TICK + ENCODER_SIGN[name] * rad / POS_UNIT_RAD))
    return max(0, min(4095, t))


def gripper_ticks_to_motor_rad(ticks):
    return GRIPPER_ENCODER_SIGN * _wrapped_tick_diff(int(ticks)) * POS_UNIT_RAD


def gripper_motor_rad_to_ticks(rad):
    t = int(round(ZERO_TICK + GRIPPER_ENCODER_SIGN * rad / POS_UNIT_RAD))
    return max(0, min(4095, t))


# Tabla de control (XM430-W350), de hw_sinusoidal_torque_node.cpp
ADDR = {
    'OPERATING_MODE': 11,
    'TORQUE_ENABLE': 64,
    'POSITION_D_GAIN': 80,
    'POSITION_I_GAIN': 82,
    'POSITION_P_GAIN': 84,
    'PROFILE_ACC': 108,
    'PROFILE_VEL': 112,
    'GOAL_POSITION': 116,
    'PRESENT_VELOCITY': 128,
    'PRESENT_POSITION': 132,
}
LEN_GOAL_POSITION = 4
LEN_PRESENT_POSITION = 4
LEN_PRESENT_VELOCITY = 4

POSITION_CONTROL_MODE = 3
CURRENT_CONTROL_MODE = 0
TORQUE_ENABLE_VAL = 1
TORQUE_DISABLE_VAL = 0

# Ganancias y perfiles por defecto (mismos valores del ros2_control oficial)
POSITION_P_GAIN = 800
POSITION_I_GAIN = 100
POSITION_D_GAIN = 100
PROFILE_VELOCITY = 200     # 0 = velocidad máxima. >0 = perfil trapezoidal suave.
PROFILE_ACCELERATION = 50

# ---------------------------------------------------------------------------
# Conexión y temporización
# ---------------------------------------------------------------------------
PORT = '/dev/ttyUSB0'
BAUDRATE = 1_000_000
PROTOCOL_VERSION = 2.0
RATE_HZ = 50.0

# Límite de velocidad articular por software (seguridad). El puente nunca mueve
# una articulación más rápido que esto, sin importar el comando recibido.
MAX_JOINT_SPEED = 1.5          # rad/s (brazo)
MAX_GRIPPER_SPEED_M = 0.04     # m/s   (gripper)
HOME_TIME_S = 3.0              # duración del movimiento "ir a cero"

# ---------------------------------------------------------------------------
# Jog cartesiano y teclas
# ---------------------------------------------------------------------------
CART_STEP_M = 0.005            # paso por pulsación [m] (5 mm)
GRIPPER_STEP_M = 0.004         # paso del gripper por pulsación [m]

# Tópicos / servicios (solo interfaces estándar)
TOPIC_JOINT_STATES = '/joint_states'
TOPIC_JOINT_STATES_PREVIEW = '/joint_states_preview'
TOPIC_JOINT_COMMAND = '/interface/joint_command'
TOPIC_GRIPPER_COMMAND = '/interface/gripper_command'
TOPIC_MODE = '/interface/mode'
TOPIC_EXECUTE_TRAJ = '/interface/execute_trajectory'
SRV_GO_HOME = '/interface/go_home'
SRV_TORQUE = '/interface/torque'
SRV_STOP = '/interface/stop'

# Teclas de jog cartesiano (Qt.Key_*). Editable.
KEYMAP = {
    'x+': 'W', 'x-': 'S',
    'y+': 'A', 'y-': 'D',
    'z+': 'Q', 'z-': 'E',
    'grip_open': 'O', 'grip_close': 'C',
    'home': 'H', 'stop': 'Space',
}

MODE_POSITION = 'position'
MODE_TEACH = 'teach'
