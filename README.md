# open_manipulator_x_demos

Aplicaciones de **taller** en ROS 2 Humble para el **OpenMANIPULATOR-X** real
(brazo serial de 4 GDL + gripper, de ROBOTIS). Este repositorio agrupa dos
paquetes pensados para usarse en un taller, ambos comunicándose con el robot a
través de un **puente Dynamixel SDK** propio (sin `ros2_control`):

- **`open_manipulator_x_interface`** — Interfaz gráfica (PyQt5) para operar **un**
  brazo: control articular con sliders circulares (en vivo y con previsualización
  en RViz antes de enviar), jog cartesiano X/Y/Z + gripper por teclas/botones, y
  modo *teach* para grabar y reproducir waypoints.
- **`open_manipulator_x_master_slave`** — Teleoperación **maestro-esclavo** con
  **dos** brazos: el maestro se mueve libremente a mano (torque OFF) y el esclavo
  replica sus movimientos en tiempo real. Reescritura para ROS 2 de la
  [app de ROBOTIS](https://github.com/ROBOTIS-GIT/open_manipulator_applications).

> `open_manipulator_x_master_slave` **reutiliza** módulos de
> `open_manipulator_x_interface` (`dxl_driver`, `config`, `kinematics`), por lo que
> ambos paquetes se construyen juntos.

> **Los detalles de uso y los comandos de ejecución de cada paquete** están en sus
> respectivos README:
> [`open_manipulator_x_interface/README.md`](open_manipulator_x_interface/README.md) ·
> [`open_manipulator_x_master_slave/README.md`](open_manipulator_x_master_slave/README.md).
> Este README cubre solo la instalación, preparación del workspace y construcción.

---

## Tabla de contenido

1. [Descripción](#1-descripción)
2. [Prerrequisitos](#2-prerrequisitos)
3. [Instalación de ROS 2 y dependencias](#3-instalación-de-ros-2-y-dependencias)
4. [Creación del workspace y clonación de repositorios](#4-creación-del-workspace-y-clonación-de-repositorios)
5. [Estructura del repositorio](#5-estructura-del-repositorio)
6. [Construcción de los paquetes](#6-construcción-de-los-paquetes)

---

## 1. Descripción

| Paquete | Para qué sirve | Robots |
|---|---|---|
| `open_manipulator_x_interface` | GUI de taller: sliders articulares (en vivo / preview RViz), jog cartesiano, gripper, teach + waypoints, "ir a cero", telemetría en grados y cm. | 1 |
| `open_manipulator_x_master_slave` | Maestro-esclavo: el maestro libre (torque OFF) y el esclavo lo sigue en tiempo real (brazo + gripper). Panel PyQt con habilitar/E-STOP. | 2 |

Ambos paquetes son **ament_python** (Python 3 + PyQt5 + rclpy) y hablan con los
servos Dynamixel XM430-W350 a través del **Dynamixel SDK** (un nodo puente que es
el único dueño del puerto serie). Pueden ejecutarse **sin hardware** gracias a un
modo de simulación (`sim:=true`), útil para preparar el taller.

---

## 2. Prerrequisitos

- **Ubuntu 22.04 (Jammy)**
- **ROS 2 Humble**
- **Python 3.10**
- Para uso real: **uno o dos OpenMANIPULATOR-X** y su(s) interfaz(es) **U2D2**
  (USB-RS485). El maestro-esclavo necesita **dos U2D2** (uno por brazo).
  Sin hardware, todo funciona en modo simulación.

---

## 3. Instalación de ROS 2 y dependencias

### 3.1. ROS 2 Humble

```bash
# Locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Fuente apt de ROS 2
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update && sudo apt install -y curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# ROS 2 Humble Desktop + herramientas de desarrollo
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools

# Sourcing automático
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 3.2. Dependencias de estos paquetes (GUI + ROS)

```bash
sudo apt install -y \
  python3-pyqt5 \
  python3-numpy \
  ros-humble-rviz2 \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  python3-colcon-common-extensions \
  python3-rosdep
```

- **PyQt5 / NumPy**: usados por las interfaces gráficas y la cinemática.
- **rviz2 / robot_state_publisher / xacro**: para la previsualización del modelo
  en `open_manipulator_x_interface`.
- El **Dynamixel SDK** (`dynamixel_sdk`) y la descripción del robot
  (`open_manipulator_x_description`) se obtienen **clonando los repositorios de
  ROBOTIS** en el workspace (siguiente sección).

### 3.3. Acceso al puerto USB (solo hardware real)

```bash
sudo usermod -a -G dialout $USER   # cerrar sesión y volver a entrar para aplicar
```

---

## 4. Creación del workspace y clonación de repositorios

Estos paquetes asumen el workspace **`~/open_manx_ws`**. Junto a este repositorio
se clonan los repos de **ROBOTIS** que aportan el Dynamixel SDK y la descripción
(URDF/meshes) del OpenMANIPULATOR-X:

```bash
mkdir -p ~/open_manx_ws/src
cd ~/open_manx_ws/src

# Repos de ROBOTIS (rama humble)
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/open_manipulator.git
git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git

# Este repositorio
git clone https://github.com/smorales2405/open_manipulator_x_demos.git
```

¿Por qué cada repo?

| Repositorio | Aporta | Lo usa |
|---|---|---|
| `DynamixelSDK` | `dynamixel_sdk` (Python) | el puente de ambos paquetes |
| `open_manipulator` | `open_manipulator_x_description` (URDF/meshes) | la previsualización RViz del interface |
| `dynamixel_hardware_interface`, `dynamixel_interfaces` | dependencias para construir `open_manipulator` | colcon |

Resolver el resto de dependencias declaradas con **rosdep**:

```bash
cd ~/open_manx_ws
sudo rosdep init   # solo la primera vez (ignora el error si ya estaba)
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

---

## 5. Estructura del repositorio

```
open_manipulator_x_demos/
├── README.md                         # este archivo (instalación / build)
├── open_manipulator_x_interface/     # GUI de taller (1 brazo)
│   ├── package.xml · setup.py · setup.cfg
│   ├── open_manipulator_x_interface/
│   │   ├── config.py                 # ÚNICA fuente de límites/constantes/IDs
│   │   ├── kinematics.py             # FK + IK analíticas
│   │   ├── dxl_driver.py             # envoltura Dynamixel SDK
│   │   ├── robot_bridge.py           # nodo puente (dueño del puerto; modos position/teach)
│   │   ├── ros_interface.py          # capa ROS de la GUI (spin en hilo + señales Qt)
│   │   ├── interface_gui.py          # punto de entrada de la GUI
│   │   └── ui/                        # ventana + slider circular + pestañas + telemetría
│   ├── launch/interface.launch.py
│   ├── rviz/interface_preview.rviz
│   ├── config/joint_limits.yaml
│   └── README.md                     # uso y comandos del paquete
└── open_manipulator_x_master_slave/  # teleoperación maestro-esclavo (2 brazos)
    ├── package.xml · setup.py · setup.cfg
    ├── open_manipulator_x_master_slave/
    │   ├── ms_config.py              # puertos, IDs, flags (reusa interface.config)
    │   ├── master_slave_node.py      # núcleo: 2 puertos, espejo, servicios, sim
    │   ├── panel_ros.py              # capa ROS del panel
    │   └── panel.py                  # panel PyQt (habilitar/E-STOP/telemetría)
    ├── launch/master_slave.launch.py
    └── README.md                     # uso y comandos del paquete
```

---

## 6. Construcción de los paquetes

Desde la raíz del workspace:

```bash
cd ~/open_manx_ws
colcon build --symlink-install
source install/setup.bash

# Sourcing automático (opcional)
echo 'source ~/open_manx_ws/install/setup.bash' >> ~/.bashrc
```

> `colcon` resuelve el orden de construcción automáticamente: como
> `open_manipulator_x_master_slave` depende de `open_manipulator_x_interface`,
> este último se compila primero.

Para reconstruir **solo** estos dos paquetes (y sus dependencias) sin compilar
todo el repositorio `open_manipulator` (MoveIt, GUI oficial, etc.):

```bash
cd ~/open_manx_ws
colcon build --symlink-install \
    --packages-up-to open_manipulator_x_interface open_manipulator_x_master_slave
source install/setup.bash
```

Una vez construidos, consulta el README de cada paquete para ejecutarlos:
[interface](open_manipulator_x_interface/README.md) ·
[master_slave](open_manipulator_x_master_slave/README.md).
