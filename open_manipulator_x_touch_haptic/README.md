# open_manipulator_x_touch_haptic

Teleoperación **cinemática** del **OpenMANIPULATOR-X** usando el **Geomagic Touch**
(3D Systems) como maestro, en ROS 2 Humble. El stylus del Touch guía al robot en
dos modos y sus dos botones abren/cierran el gripper. **Sin realimentación
háptica**: solo se lee el Touch y se comanda el robot (cinemática pura). Se apoya
en `open_manipulator_x_interface` (Dynamixel SDK, cinemática, límites) y en el
driver [`Geomagic_Touch_ROS2`](https://github.com/IvoD1998/Geomagic_Touch_ROS2)
(`omni_msgs`, topics `/phantom/*`).

## Cómo funciona

```
 touch_haptic_panel (PyQt5)         touch_haptic_node  (dueño del puerto del OM-X)
   Habilitar / E-STOP / modo  ──▶   lee /phantom/* (Touch)  ──┐
   gripper / telemetría             escribe al OM-X (USB0) ◀──┘  (el robot sigue al Touch)
```

El Geomagic Touch tiene **6 GDL**: los 3 primeros (J1, J2, J3) con capacidad de
fuerza y los 3 del stylus (J4, J5, J6) sin fuerza. Aquí **solo se leen** sus
posiciones (no se aplica fuerza).

### Modos (conmutables desde el panel)

- **Articular** — mapea articulaciones del Touch → OM-X:
  `waist→joint1`, `shoulder→joint2`, `elbow→joint3`, `pitch (J5)→joint4`.
- **Cartesiano** — mapea la posición **X, Y, Z del stylus** (centro de J5) a la
  posición deseada del **efector**, resuelta con cinemática inversa
  (`open_manipulator_x_interface/kinematics.ik_step`, solo posición, sin
  orientación).

En ambos modos se usa un **"engage" (clutch)**: al habilitar (o cambiar de modo)
se captura la pose actual de Touch y robot como referencia y se mapea el
**movimiento relativo**. Así el robot **no salta** y los offsets del Touch no
importan. Todo respeta los **límites articulares** de
`open_manipulator_x_interface/config/joint_limits.yaml`.

### Gripper

Los **dos botones del stylus** lo controlan: **gris = cerrar**, **blanco = abrir**
(configurable). La apertura/cierre usa la calibración por robot de
`open_manipulator_x_interface/config/robot_gripper_cal.yaml` (parámetro `robot_id`).

## Requisitos previos

1. **Drivers del Touch instalados**: OpenHaptics + Touch Device Driver
   (3D Systems), y el paquete del workspace **`Geomagic_Touch_ROS2`** compilado
   (aporta `omni_msgs` y el nodo `omni_state`).
2. Paquete **`open_manipulator_x_interface`** del mismo repo (dependencia directa).

## Compilación

```bash
cd ~/open_manx_ws
colcon build --packages-up-to open_manipulator_x_touch_haptic
source install/setup.bash
```

## Uso

### 1) Lanzar el driver del Touch (en otra terminal)

```bash
ros2 launch omni_common omni_state.launch.py
```

Comprueba que publica: `ros2 topic list | grep phantom` →
`/phantom/joint_states`, `/phantom/pose`, `/phantom/button`.

### 2) Lanzar la teleoperación + panel

```bash
# Verifica el puerto del OM-X (ls -l /dev/ttyUSB*) y el grupo dialout.
ros2 launch open_manipulator_x_touch_haptic touch_haptic.launch.py \
    robot_port:=/dev/ttyUSB0 robot_id:=1
```

En el panel: elige **modo** (Articular / Cartesiano), pulsa **HABILITAR** y mueve
el stylus; el OM-X sigue al Touch. **E-STOP** (botón o tecla `Espacio`) congela el
robot. Usa los botones del stylus (o los del panel) para el gripper.

> También puedes incluir el driver desde el mismo launch con `launch_driver:=true`.

### Sin hardware (prueba completa offline)

```bash
ros2 launch open_manipulator_x_touch_haptic touch_haptic.launch.py \
    sim:=true virtual_touch:=true
```

`virtual_touch` publica un `/phantom/*` sintético (maestro senoidal) y el nodo
hace **eco** del robot. Habilita el espejo y verás la telemetría seguir al Touch
virtual en ambos modos; los botones virtuales accionan el gripper.

## Control por línea de comandos (sin panel)

```bash
ros2 topic pub --once /touch_haptic/mode std_msgs/msg/String '{data: cartesian}'
ros2 service call /touch_haptic/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /touch_haptic/stop   std_srvs/srv/Trigger          # E-STOP
ros2 topic pub --once /touch_haptic/gripper_cmd std_msgs/msg/String '{data: open}'
```

## Ajuste del mapeo (orientación / sensibilidad)

Si el robot se mueve **al revés** en algún eje o articulación, o quieres más/menos
sensibilidad, edita **`open_manipulator_x_touch_haptic/th_config.py`** — toda la
"orientación" del mapeo está centralizada ahí:

- **`JOINT_MAP`** — pares `(touch, om, signo, ganancia)` del modo articular.
  Cambia el **signo** si una articulación va invertida; la **ganancia** escala.
- **`CART_AXES`** — qué eje del Touch alimenta cada eje del robot y con qué signo
  (def. `robot_x←+touch_y`, `robot_y←−touch_x`, `robot_z←+touch_z`).
- **`CART_SCALE`** — sensibilidad cartesiana Touch→robot (m/m).
- **`BUTTON_CLOSE` / `BUTTON_OPEN`** — qué botón (gris/blanco) abre y cuál cierra.

## Seguridad

- El robot está **siempre** limitado en velocidad y recortado a los límites
  articulares (`open_manipulator_x_interface/config.py`).
- Arranca **pausado** (`enable_on_start:=false`); al habilitar **no salta**
  (engage/clutch).
- **E-STOP congela** el robot en su sitio (mantiene torque → no cae).
- En modo cartesiano, si una pose objetivo queda **fuera de alcance**, el robot
  **mantiene** la última posiblе (la IK no fuerza soluciones inválidas).
- ⚠️ **No** lances esto a la vez que `ros2_control`, el `robot_bridge` del paquete
  `open_manipulator_x_interface`, ni `master_slave_node`: se pelearían por el
  puerto serie del OM-X.

## Parámetros del nodo

`robot_port`, `robot_id`, `sim`, `mode` (`joint`|`cartesian`), `enable_on_start`,
`cart_scale`. El resto del mapeo vive en `th_config.py`.
