"""
ms_config.py — Configuración de la aplicación maestro-esclavo.

Reutiliza las constantes y conversiones del paquete open_manipulator_x_interface
(config.py) para no duplicar: límites, signos de encoder, mapeo de gripper,
conversiones tick<->rad, tabla de control, etc. Aquí solo se definen los puertos,
los IDs de cada brazo y unos pocos parámetros propios.

>>> EDITA AQUÍ los puertos / IDs si tu montaje difiere <<<
"""

from open_manipulator_x_interface import config as omx

# Puertos U2D2 (convención ROBOTIS: esclavo en USB0, maestro en USB1).
SLAVE_PORT = '/dev/ttyUSB0'
MASTER_PORT = '/dev/ttyUSB1'

# IDs Dynamixel de cada brazo. Por decisión del taller, AMBOS de fábrica (11-15);
# se distinguen por el puerto. Si re-asignaste el maestro (convención ROBOTIS),
# cámbialo a {joint1:21, joint2:22, joint3:23, joint4:24, gripper:25}.
SLAVE_IDS = dict(omx.DXL_IDS)        # {joint1:11, joint2:12, joint3:13, joint4:14, gripper:15}
MASTER_IDS = dict(omx.DXL_IDS)

# Comportamiento
RATE_HZ = omx.RATE_HZ                 # 50 Hz
MIRROR_GRIPPER = True                 # el esclavo copia la apertura del gripper
ENABLE_ON_START = False               # seguridad: el espejo arranca PAUSADO
JOINT_SCALE = 1.0                     # escala maestro->esclavo (1.0 = 1:1)

# Tópicos / servicios
TOPIC_MASTER_STATES = '/master/joint_states'
TOPIC_SLAVE_STATES = '/slave/joint_states'
TOPIC_MASTER_CMD = '/master/joint_command'   # solo en --sim: maestro virtual opcional
SRV_ENABLE = '/master_slave/enable'
SRV_STOP = '/master_slave/stop'
