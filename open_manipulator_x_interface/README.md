# open_manipulator_x_interface

Interfaz gráfica de **taller** (PyQt5 + ROS 2 Humble) para operar el
**OpenMANIPULATOR-X real** de forma simple y ordenada.

Permite:

1. **Control articular en vivo** del robot real con *sliders circulares* (4 ejes
   + gripper), respetando límites articulares.
2. Los mismos sliders en modo **previsualización**: mueven un modelo en RViz y,
   con un botón, se envía la configuración al robot. Se intercala 1 ⇄ 2 con un
   conmutador.
3. **Jog cartesiano** X/Y/Z del efector con botones o teclas, más abrir/cerrar
   gripper.
4. Modo **Teach**: el brazo queda suelto (torque OFF) para moverlo a mano y
   **grabar waypoints**, que luego se recorren como **trayectoria** (incluida la
   acción del gripper).

Extras: botón **Ir a cero** (0 rad), **STOP**, lecturas de articulaciones en
**grados** y del efector en **centímetros**.

---

## Arquitectura

```
 interface_gui (PyQt5)  ──comandos ROS──▶  robot_bridge (Dynamixel SDK)  ──▶ robot real
        │  ▲                                      │
        │  └──────────── /joint_states ───────────┘
        ▼
 /joint_states_preview ──▶ robot_state_publisher ──▶ RViz (modelo preview)
```

- **`robot_bridge`**: único dueño de `/dev/ttyUSB0`. Modos `position` (torque ON,
  sigue comandos con límite de velocidad) y `teach` (torque OFF, backdrivable).
  Reproduce trayectorias y ofrece `go_home` / `stop` / `torque`.
- **`interface_gui`**: cliente puro. No toca el puerto.

> ⚠️ **No** ejecutes esta interfaz a la vez que el stack oficial
> (`open_manipulator_x_bringup hardware.launch.py` / ros2_control): ambos
> abrirían el mismo puerto serie.

### Interfaces ROS (solo tipos estándar)

| Nombre | Tipo | Uso |
|---|---|---|
| `/joint_states` | sensor_msgs/JointState | estado real |
| `/joint_states_preview` | sensor_msgs/JointState | modelo RViz |
| `/interface/joint_command` | sensor_msgs/JointState | target brazo [rad] |
| `/interface/gripper_command` | std_msgs/Float64 | apertura gripper [m] |
| `/interface/mode` | std_msgs/String | `position` / `teach` |
| `/interface/execute_trajectory` | trajectory_msgs/JointTrajectory | playback waypoints |
| `/interface/go_home` | std_srvs/Trigger | ir a 0 rad |
| `/interface/torque` | std_srvs/SetBool | torque brazo ON/OFF |
| `/interface/stop` | std_srvs/Trigger | congelar posición |

---

## Compilación

```bash
cd ~/open_manx_ws
colcon build --packages-select open_manipulator_x_interface
source install/setup.bash
```

Dependencias: `python3-pyqt5`, `dynamixel_sdk`, `numpy`, `robot_state_publisher`,
`rviz2`, `xacro`, `open_manipulator_x_description` (ya presentes en este
workspace).

---

## Uso

### Sin robot (preparar el taller)

```bash
ros2 launch open_manipulator_x_interface interface.launch.py sim:=true
```

El `robot_bridge` hace eco de los comandos a `/joint_states`, así puedes probar
toda la GUI y ver el modelo en RViz sin hardware.

### Con robot real

```bash
# Asegúrate de tener permiso sobre el puerto (p. ej. grupo dialout) y de que
# NINGÚN otro proceso (ros2_control) esté usando /dev/ttyUSB0.
ros2 launch open_manipulator_x_interface interface.launch.py port_name:=/dev/ttyUSB0
```

### Teclas (pestaña Cartesiano)

| Tecla | Acción | | Tecla | Acción |
|---|---|---|---|---|
| `W`/`S` | +X / −X | | `O` | abrir gripper |
| `A`/`D` | +Y / −Y | | `C` | cerrar gripper |
| `Q`/`E` | +Z / −Z | | `H` | ir a cero |
| `Espacio` | STOP |  |  |  |

(Se editan en `config.py → KEYMAP`.)

---

## Personalización (todo en `config.py`)

- **Límites articulares**: `JOINT_LIMITS` (o activa `USE_WORKSHOP_LIMITS` para un
  set más conservador).
- **Velocidad máxima / suavidad**: `MAX_JOINT_SPEED`, `PROFILE_VELOCITY`.
- **Pasos de jog**: `CART_STEP_M`, `GRIPPER_STEP_M`.
- **Teclas**: `KEYMAP`.
- **Gripper**: calibración del motor en `GRIPPER_CLOSED_DEG` / `GRIPPER_OPEN_DEG`
  (por defecto 40° / 160°, medidos con Dynamixel Wizard).
  > Si el gripper se mueve al revés o no llega, ajusta esos dos ángulos.
- **Sentido de giro**: `ENCODER_SIGN` (si una articulación gira al revés que el
  modelo de RViz, invierte su signo).

---

## Seguridad

- Los límites se aplican **doblemente** (en la GUI y en el puente).
- El puente **limita la velocidad** de cada articulación por software.
- Al entrar a **Teach** el brazo queda suelto: **sostenlo** antes de activarlo.
- **STOP** congela el robot en su posición actual en cualquier momento.

---

## Notas de implementación

- La cinemática (`kinematics.py`) es un *port* directo de la FK analítica de
  `open_manipulator_x_torque_control/.../hw_fkin_node.cpp`, así la pose mostrada
  coincide con la de los laboratorios. La IK del jog es DLS (mínimos cuadrados
  amortiguados) sembrada en la pose actual.
- Conversión rad↔tick y signos de encoder `{+1,−1,−1,−1}` también provienen de
  ese nodo (cero en tick 2048).
