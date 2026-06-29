# open_manipulator_x_master_slave

Teleoperación **maestro-esclavo** (leader-follower) para **dos OpenMANIPULATOR-X**
en ROS 2 Humble. El brazo **maestro** se mueve libremente a mano (torque OFF) y el
brazo **esclavo** replica sus movimientos en tiempo real (articulaciones +
gripper). Reescritura para ROS 2 de la
[app de ROBOTIS](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_applications/),
apoyada en el paquete `open_manipulator_x_interface` (Dynamixel SDK, conversiones,
cinemática).

## Cómo funciona

```
 master_slave_panel (PyQt5)        master_slave_node  (dueño de AMBOS puertos)
   Habilitar / E-STOP / estado  ──▶  maestro (USB1, torque OFF)  ──lee──┐
                                      esclavo (USB0, posición)  ◀─escribe┘  (sigue al maestro)
```

- **`master_slave_node`** abre los dos puertos: deja el maestro **suelto** y
  comanda el esclavo en **control de posición** a 50 Hz, con **límite de
  velocidad** y **recorte a límites articulares** por software.
- **`master_slave_panel`** es un cliente puro: habilita/pausa el espejo, E-STOP,
  y muestra la comparación maestro vs esclavo.

## Hardware

- **Dos U2D2**, uno por brazo. Por defecto: **maestro → `/dev/ttyUSB1`**,
  **esclavo → `/dev/ttyUSB0`** (convención ROBOTIS). Baudrate 1 Mbps.
- IDs Dynamixel: por defecto **ambos brazos 11–15** (de fábrica), distinguidos por
  el puerto. Si re-asignaste el maestro a 21–25, edítalo en
  `open_manipulator_x_master_slave/ms_config.py`.

## Compilación

```bash
cd ~/open_manx_ws
colcon build --packages-select open_manipulator_x_interface open_manipulator_x_master_slave
source install/setup.bash
```

## Uso

### Sin robots (demo / preparar el taller)

```bash
ros2 launch open_manipulator_x_master_slave master_slave.launch.py sim:=true
```

El maestro virtual se mueve solo (senoidal). Pulsa **HABILITAR ESPEJO** y verás el
esclavo seguirlo en la telemetría; **E-STOP** lo congela.

### Con los dos robots reales

```bash
# Verifica qué puerto es cada brazo (ls -l /dev/ttyUSB*) y los permisos (grupo dialout).
ros2 launch open_manipulator_x_master_slave master_slave.launch.py \
    master_port:=/dev/ttyUSB1 slave_port:=/dev/ttyUSB0
```

1. Sostén el brazo **maestro** (queda suelto).
2. Pulsa **HABILITAR ESPEJO**: el esclavo se acerca suave a la pose del maestro y
   luego lo sigue.
3. Mueve el maestro a mano; el esclavo replica. **E-STOP** (botón o tecla
   `Espacio`) pausa y congela el esclavo.

## Control por línea de comandos (sin panel)

```bash
ros2 service call /master_slave/enable std_srvs/srv/SetBool '{data: true}'   # activar
ros2 service call /master_slave/stop   std_srvs/srv/Trigger                  # E-STOP
```

## Seguridad

- El esclavo está **siempre** limitado en velocidad y recortado a los límites
  articulares (definidos en `open_manipulator_x_interface/config.py`).
- Arranca **pausado** (`enable_on_start:=false`); al habilitar, el esclavo **no
  salta**: se acerca suave a la pose del maestro.
- **E-STOP congela** el esclavo en su sitio (no suelta torque → no cae).
- ⚠️ **No** lances esto a la vez que `ros2_control` o el `robot_bridge` del paquete
  `open_manipulator_x_interface`: se pelearían por los puertos serie.

## Parámetros (en `ms_config.py`)

`SLAVE_PORT`, `MASTER_PORT`, `SLAVE_IDS`, `MASTER_IDS`, `RATE_HZ`,
`MIRROR_GRIPPER`, `ENABLE_ON_START`, `JOINT_SCALE`.
