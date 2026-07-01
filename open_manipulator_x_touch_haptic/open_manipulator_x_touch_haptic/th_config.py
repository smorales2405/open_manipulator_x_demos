"""
th_config.py — Configuración de la teleoperación Touch Haptic -> OpenMANIPULATOR-X.

Reutiliza las constantes y conversiones de open_manipulator_x_interface
(config.py): límites articulares, signos de encoder, mapeo de gripper,
conversiones tick<->rad y la cinemática. Aquí solo se definen el puerto/IDs del
robot y, sobre todo, EL MAPEO entre el Geomagic Touch (maestro) y el robot.

>>> EDITA AQUÍ si el robot se mueve al revés (signos) o quieres más/menos
    sensibilidad (ganancias / escala). Toda la "orientación" del mapeo vive en
    este archivo, para que sea fácil de ajustar en el taller. <<<

Convención de ejes
------------------
El driver Geomagic (Geomagic_Touch_ROS2/omni_state.cpp) ya transforma la pose del
stylus a un frame cómodo y la publica en /phantom/pose [m]:
    x = derecha,  y = adelante (lejos del usuario),  z = arriba
La base del OpenMANIPULATOR-X (kinematics.fkin) usa:
    x = adelante,  y = izquierda,  z = arriba
"""

import math

from open_manipulator_x_interface import config as omx

# ---------------------------------------------------------------------------
# Robot esclavo (OpenMANIPULATOR-X)
# ---------------------------------------------------------------------------
ROBOT_PORT = '/dev/ttyUSB0'           # puerto U2D2 del OM-X
ROBOT_IDS = dict(omx.DXL_IDS)         # {joint1:11, ..., gripper:15}
RATE_HZ = omx.RATE_HZ                 # 50 Hz
ENABLE_ON_START = False               # seguridad: arranca PAUSADO

# ---------------------------------------------------------------------------
# Tópicos del driver Geomagic (omni_name = "phantom")
# ---------------------------------------------------------------------------
TOPIC_PHANTOM_JOINTS = '/phantom/joint_states'   # [waist,shoulder,elbow,yaw,pitch,roll]
TOPIC_PHANTOM_POSE = '/phantom/pose'             # geometry_msgs/PoseStamped [m]
TOPIC_PHANTOM_BUTTON = '/phantom/button'         # omni_msgs/OmniButtonEvent

# Nombres de las 6 articulaciones del Touch (orden de /phantom/joint_states).
PHANTOM_JOINT_NAMES = ['waist', 'shoulder', 'elbow', 'yaw', 'pitch', 'roll']

# ---------------------------------------------------------------------------
# MODO ARTICULAR — mapeo ABSOLUTO Touch -> OM-X
#
#   Fórmulas directas (no relativas al engage):
#     joint1 = waist_TH                          límite estándar (±π/2)
#     joint2 = π/2  − shoulder_TH                límite ±π/4 (±45°)
#     joint3 = −elbow_TH                         límite ±π/4 (±45°)
#     joint4 = pitch_TH + 3π/4                   límite ±π/4 (±45°)
#
#   Si el Touch supera el rango que mantiene el OM-X dentro del límite, esa
#   articulación se queda en su límite (clamp independiente) mientras las
#   demás siguen moviéndose. Al volver dentro del rango, la articulación
#   retoma el seguimiento automáticamente.
#
#   JOINT_MAP solo se usa para la telemetría del panel (qué par Touch↔robot
#   se muestra en cada fila). Para el mapeo real, usa map_touch_joints_to_robot().
# ---------------------------------------------------------------------------
JOINT_MAP = [
    ('waist',    'joint1', +1.0, 1.0),
    ('shoulder', 'joint2', -1.0, 1.0),
    ('elbow',    'joint3', -1.0, 1.0),
    ('pitch',    'joint4', +1.0, 1.0),   # J5 del Touch -> joint4 del OM-X
]

# Constantes de la transformación articular
_J2_SHOULDER_REF = math.pi / 2          # offset de referencia shoulder: 90°
_J4_PITCH_REF    = 3.0 * math.pi / 4    # offset de referencia pitch:    135°

# Límites interiores para J2, J3, J4 en modo articular (±grados → ±rad).
# J1 usa los límites estándar del hardware (±90° via omx.clamp_joint).
# >>> Edita estos valores para ajustar el rango de cada articulación. <<<
JOINT_MODE_LIMITS = {
    'joint2': math.radians(45.0),   # ±45°
    'joint3': math.radians(45.0),   # ±45°
    'joint4': math.radians(60.0),   # ±60°
}


def map_touch_joints_to_robot(touch_q):
    """Mapeo absoluto Touch -> OM-X para el modo articular.

    Devuelve dict {om_name: rad} SIN clamp. El nodo aplica el clamp por
    articulación: ±JOINT_MODE_LIMITS para J2/J3/J4, límites estándar para J1.
    """
    return {
        'joint1': touch_q.get('waist', 0.0),
        'joint2': _J2_SHOULDER_REF - touch_q.get('shoulder', 0.0),
        'joint3': -touch_q.get('elbow', 0.0),
        'joint4': touch_q.get('pitch', 0.0) + _J4_PITCH_REF,
    }


def _clamp_joint_mode(name, value):
    """Clampa J2/J3/J4 a su límite del modo articular (JOINT_MODE_LIMITS)."""
    lim = JOINT_MODE_LIMITS[name]
    return max(-lim, min(lim, value))


def articular_joint_targets(touch_q):
    """Objetivos articulares [q1,q2,q3,q4] (rad) del modo Articular, ya clampeados.

    J1 usa los límites estándar del hardware (omx.clamp_joint); J2/J3/J4 usan
    JOINT_MODE_LIMITS. Es la ÚNICA fuente del mapeo articular: la usan tanto el
    modo Articular (posición de las 4 articulaciones) como el modo Cartesiano
    (para derivar phi), de modo que la orientación se comanda idéntica en ambos.
    """
    raw = map_touch_joints_to_robot(touch_q)
    return [
        omx.clamp_joint('joint1', raw['joint1']),
        _clamp_joint_mode('joint2', raw['joint2']),
        _clamp_joint_mode('joint3', raw['joint3']),
        _clamp_joint_mode('joint4', raw['joint4']),
    ]


def articular_phi(touch_q):
    """Orientación phi = J2 + J3 + J4 con el MISMO mapeo/escalamiento/límites
    del modo Articular. Es la orientación que comanda el modo Cartesiano."""
    q = articular_joint_targets(touch_q)
    return q[1] + q[2] + q[3]

# ---------------------------------------------------------------------------
# MODO CARTESIANO — posición del stylus -> posición del efector (+ orientación)
#
#   POSICIÓN: cada eje del robot (x,y,z) se toma de un eje del Touch con signo:
#       robot_x  <-  +touch_y   (adelante)
#       robot_y  <-  -touch_x   (izquierda = -derecha)
#       robot_z  <-  +touch_z   (arriba)
#   p_om = p_engage + CART_SCALE * (mapeo de (p_touch - p_touch_engage))
#   >>> Si un eje va al revés, cambia su signo aquí. <<<
#
#   ORIENTACIÓN (para parecerse al modo Articular): con CART_INCLUDE_ORIENTATION
#   se comanda phi = J2+J3+J4 usando EXACTAMENTE la fórmula, escalamiento y
#   límites del modo Articular (articular_phi()). Como el OM-X es de 4 GDL,
#   (x,y,z,phi) es un sistema cuadrado: se resuelven a la vez con el Jacobiano
#   4x4, sin conflicto ni acople entre posición y orientación.
#
#   Con CART_APPLY_JOINT_MODE_LIMITS, el modo Cartesiano respeta además los
#   MISMOS límites articulares (JOINT_MODE_LIMITS) que el Articular, para que el
#   espacio de trabajo de ambos modos coincida.
# ---------------------------------------------------------------------------
CART_AXES = (('y', +1.0), ('x', -1.0), ('z', +1.0))   # robot (x, y, z) <- touch
CART_SCALE = 2.0                      # sensibilidad Touch->robot (m/m)
CART_INCLUDE_ORIENTATION = True       # comanda phi = J2+J3+J4 (fórmula articular)
CART_APPLY_JOINT_MODE_LIMITS = True   # usa los mismos límites que el modo articular
CART_HOLD_PHI = False                 # (solo si CART_INCLUDE_ORIENTATION=False)
IK_DAMPING = 0.05                     # amortiguación DLS del ik_step
IK_MAX_DQ = 0.20                      # paso articular máx. por iteración [rad]

_AXIS_INDEX = {'x': 0, 'y': 1, 'z': 2}


def map_touch_delta_to_robot(delta_touch):
    """Convierte un desplazamiento del stylus (np3 [dx,dy,dz] en frame Touch) a un
    desplazamiento del efector (np3 en frame robot) según CART_AXES y CART_SCALE."""
    out = [0.0, 0.0, 0.0]
    for i, (axis, sign) in enumerate(CART_AXES):
        out[i] = CART_SCALE * sign * float(delta_touch[_AXIS_INDEX[axis]])
    return out


# ---------------------------------------------------------------------------
# Gripper — botones del stylus
#   El stylus tiene 2 botones: gris (grey_button) y blanco (white_button).
#   >>> Intercambia si abren/cierran al revés. <<<
# ---------------------------------------------------------------------------
BUTTON_CLOSE = 'grey'                 # 'grey' | 'white'  -> cierra el gripper
BUTTON_OPEN = 'white'                 # 'grey' | 'white'  -> abre el gripper

# ---------------------------------------------------------------------------
# Tópicos / servicios propios del paquete
# ---------------------------------------------------------------------------
TOPIC_ROBOT_STATES = '/touch_haptic/robot_joint_states'   # estado del OM-X
TOPIC_MODE = '/touch_haptic/mode'                         # std_msgs/String
TOPIC_GRIPPER_CMD = '/touch_haptic/gripper_cmd'           # std_msgs/String 'open'|'close'
SRV_ENABLE = '/touch_haptic/enable'                       # std_srvs/SetBool
SRV_STOP = '/touch_haptic/stop'                           # std_srvs/Trigger (E-STOP)

# Modos
MODE_JOINT = 'joint'
MODE_CARTESIAN = 'cartesian'
DEFAULT_MODE = MODE_JOINT
