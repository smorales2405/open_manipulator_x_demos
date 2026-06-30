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
# MODO ARTICULAR — mapeo Touch -> OM-X
#   Cada fila: (articulación del Touch, articulación del OM-X, signo, ganancia)
#   q_om = q_engage + signo * ganancia * (q_touch - q_touch_engage)
#   Se captura una referencia ("engage") al habilitar, así NO hay saltos y no
#   importan los offsets absolutos del Touch.
#   >>> Si una articulación va al revés, cambia su SIGNO. <<<
# ---------------------------------------------------------------------------
JOINT_MAP = [
    ('waist',    'joint1', +1.0, 1.0),
    ('shoulder', 'joint2', +1.0, 1.0),
    ('elbow',    'joint3', +1.0, 1.0),
    ('pitch',    'joint4', +1.0, 1.0),   # J5 del Touch -> joint4 del OM-X
]

# ---------------------------------------------------------------------------
# MODO CARTESIANO — mapeo posición stylus -> posición efector
#   Cada eje del robot (x,y,z) se toma de un eje del Touch con un signo:
#       robot_x  <-  +touch_y   (adelante)
#       robot_y  <-  -touch_x   (izquierda = -derecha)
#       robot_z  <-  +touch_z   (arriba)
#   p_om = p_engage + CART_SCALE * (mapeo de (p_touch - p_touch_engage))
#   >>> Si un eje va al revés, cambia su signo aquí. <<<
# ---------------------------------------------------------------------------
CART_AXES = (('y', +1.0), ('x', -1.0), ('z', +1.0))   # robot (x, y, z) <- touch
CART_SCALE = 2.0                      # sensibilidad Touch->robot (m/m)
CART_HOLD_PHI = False                 # solo posición (sin orientación)
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
