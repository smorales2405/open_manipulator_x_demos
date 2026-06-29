"""
dxl_driver.py — Envoltura mínima del Dynamixel SDK para el OpenMANIPULATOR-X.

Encapsula GroupSyncRead/Write y los comandos de configuración (modo de
operación, torque, ganancias, perfiles) sobre los servos XM430-W350 (IDs 11-15).
Las direcciones de la tabla de control y los factores de conversión provienen de
`hw_fkin_node.cpp` / `hw_sinusoidal_torque_node.cpp` del paquete torque_control.

Trabaja en TICKS; la conversión tick<->rad la hace el nodo puente con
config.arm_*_ticks/rad y config.gripper_*.
"""

from dynamixel_sdk import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
    GroupSyncRead,
    GroupSyncWrite,
    PacketHandler,
    PortHandler,
)

from . import config

A = config.ADDR


class DynamixelError(RuntimeError):
    pass


class DynamixelDriver:
    def __init__(self, port=config.PORT, baud=config.BAUDRATE, ids=None):
        self.port_name = port
        self.baud = baud
        self.ids = list(ids) if ids is not None else (config.ARM_IDS + [config.GRIPPER_ID])
        self.port = PortHandler(port)
        self.packet = PacketHandler(config.PROTOCOL_VERSION)
        self._sr_pos = None
        self._sr_vel = None
        self._sw_pos = None

    # -- conexión --------------------------------------------------------
    def connect(self):
        if not self.port.openPort():
            raise DynamixelError(f'No se pudo abrir el puerto {self.port_name}')
        if not self.port.setBaudRate(self.baud):
            raise DynamixelError(f'No se pudo fijar baudrate {self.baud}')

        self._sr_pos = GroupSyncRead(
            self.port, self.packet, A['PRESENT_POSITION'], config.LEN_PRESENT_POSITION)
        self._sr_vel = GroupSyncRead(
            self.port, self.packet, A['PRESENT_VELOCITY'], config.LEN_PRESENT_VELOCITY)
        for i in self.ids:
            if not self._sr_pos.addParam(i):
                raise DynamixelError(f'SyncRead(pos) addParam falló para ID {i}')
            if not self._sr_vel.addParam(i):
                raise DynamixelError(f'SyncRead(vel) addParam falló para ID {i}')
        self._sw_pos = GroupSyncWrite(
            self.port, self.packet, A['GOAL_POSITION'], config.LEN_GOAL_POSITION)

    def close(self):
        try:
            for i in self.ids:
                self.set_torque(i, False)
        except Exception:
            pass
        self.port.closePort()

    # -- escritura de registros simples ----------------------------------
    def _w1(self, dxl_id, addr, value):
        rc, err = self.packet.write1ByteTxRx(self.port, dxl_id, addr, value & 0xFF)
        self._check(rc, err, dxl_id, f'write1@{addr}')

    def _w2(self, dxl_id, addr, value):
        rc, err = self.packet.write2ByteTxRx(self.port, dxl_id, addr, value & 0xFFFF)
        self._check(rc, err, dxl_id, f'write2@{addr}')

    def _w4(self, dxl_id, addr, value):
        rc, err = self.packet.write4ByteTxRx(self.port, dxl_id, addr, value & 0xFFFFFFFF)
        self._check(rc, err, dxl_id, f'write4@{addr}')

    def _check(self, rc, err, dxl_id, what):
        if rc != COMM_SUCCESS:
            raise DynamixelError(
                f'[ID {dxl_id}] {what}: {self.packet.getTxRxResult(rc)}')
        if err != 0:
            raise DynamixelError(
                f'[ID {dxl_id}] {what}: {self.packet.getRxPacketError(err)}')

    # -- comandos de alto nivel ------------------------------------------
    def set_torque(self, dxl_id, on):
        self._w1(dxl_id, A['TORQUE_ENABLE'],
                 config.TORQUE_ENABLE_VAL if on else config.TORQUE_DISABLE_VAL)

    def set_operating_mode(self, dxl_id, mode):
        # El modo solo se puede cambiar con el torque deshabilitado.
        self.set_torque(dxl_id, False)
        self._w1(dxl_id, A['OPERATING_MODE'], mode)

    def setup_position_control(self, dxl_id, with_gains=True):
        """Deja un servo en Position Control Mode, con ganancias/perfil y torque ON."""
        self.set_operating_mode(dxl_id, config.POSITION_CONTROL_MODE)
        if with_gains:
            self._w2(dxl_id, A['POSITION_P_GAIN'], config.POSITION_P_GAIN)
            self._w2(dxl_id, A['POSITION_I_GAIN'], config.POSITION_I_GAIN)
            self._w2(dxl_id, A['POSITION_D_GAIN'], config.POSITION_D_GAIN)
            self._w4(dxl_id, A['PROFILE_ACC'], config.PROFILE_ACCELERATION)
            self._w4(dxl_id, A['PROFILE_VEL'], config.PROFILE_VELOCITY)
        self.set_torque(dxl_id, True)

    def setup_all_position(self):
        for i in self.ids:
            self.setup_position_control(i, with_gains=(i != config.GRIPPER_ID))

    # -- lectura/escritura de estado -------------------------------------
    @staticmethod
    def _to_signed32(v):
        v &= 0xFFFFFFFF
        return v - 0x100000000 if v > 0x7FFFFFFF else v

    def read_positions_ticks(self):
        if self._sr_pos.txRxPacket() != COMM_SUCCESS:
            return None
        out = {}
        for i in self.ids:
            if not self._sr_pos.isAvailable(i, A['PRESENT_POSITION'], config.LEN_PRESENT_POSITION):
                return None
            out[i] = self._to_signed32(
                self._sr_pos.getData(i, A['PRESENT_POSITION'], config.LEN_PRESENT_POSITION))
        return out

    def read_velocities_ticks(self):
        if self._sr_vel.txRxPacket() != COMM_SUCCESS:
            return None
        out = {}
        for i in self.ids:
            if not self._sr_vel.isAvailable(i, A['PRESENT_VELOCITY'], config.LEN_PRESENT_VELOCITY):
                return None
            out[i] = self._to_signed32(
                self._sr_vel.getData(i, A['PRESENT_VELOCITY'], config.LEN_PRESENT_VELOCITY))
        return out

    def write_goal_ticks(self, id_to_ticks):
        self._sw_pos.clearParam()
        for dxl_id, ticks in id_to_ticks.items():
            t = max(0, min(4095, int(ticks)))
            param = [DXL_LOBYTE(DXL_LOWORD(t)), DXL_HIBYTE(DXL_LOWORD(t)),
                     DXL_LOBYTE(DXL_HIWORD(t)), DXL_HIBYTE(DXL_HIWORD(t))]
            if not self._sw_pos.addParam(dxl_id, bytes(param)):
                return False
        return self._sw_pos.txPacket() == COMM_SUCCESS
