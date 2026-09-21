"""A fake DH5 on a fake serial port.

These tests drive the real `DH5ModbusAPI` against a simulated device, so
they cover frame building, CRC, register addressing and response parsing -
not just the layers above. Nothing here needs hardware.
"""

import struct
import sys
from pathlib import Path

import pytest

# Import the project from src/ rather than from an install,
# so the tests describe the working tree.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from dh5 import registers as reg  # noqa: E402
from dh5.hand import DH5Hand  # noqa: E402
from dh5.modbus import DH5ModbusAPI  # noqa: E402


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


class FakeDH5:
    """A DH5 that answers Modbus frames from an in-memory register file.

    Motion is instantaneous: reading a position feedback register returns
    whatever was last written to the matching position setpoint. Speed
    feedback always reads 0, exactly as a real stationary axis does - which
    is what makes the setpoint/feedback regression test meaningful.
    """

    def __init__(self, modbus_id: int = 1, sensor_points: int = reg.HUALICHUANG_POINTS):
        self.modbus_id = modbus_id
        self.registers = {}
        self.writes = []       # [(address, [values])] in order
        self.exception_code = None  # set to force the next response to fail

        for axis in range(reg.NUM_AXES):
            self.registers[reg.SETPOINT_BASE["position"] + axis] = 0
            self.registers[reg.SETPOINT_BASE["force"] + axis] = 50
            self.registers[reg.SETPOINT_BASE["speed"] + axis] = 50
            self.registers[reg.AXIS_STATUS_BASE_REGISTER + axis] = 1  # reached position
            self.registers[reg.FEEDBACK_BASE["speed"] + axis] = 0     # standing still
            self.registers[reg.FEEDBACK_BASE["current"] + axis] = 0

        self.registers[reg.INITIALIZE_STATUS_REGISTER] = 0x0555  # 0b01 per axis
        self.registers[reg.CURRENT_FAULT_REGISTER] = 0
        self.registers[reg.SENSOR_POINTS_PER_FINGER_REGISTER] = sensor_points

    # -- register access ------------------------------------------------
    def read(self, address: int) -> int:
        position_feedback = reg.FEEDBACK_BASE["position"]
        if position_feedback <= address < position_feedback + reg.NUM_AXES:
            # Instantaneous motion: feedback mirrors the setpoint.
            return self.registers[reg.SETPOINT_BASE["position"] + (address - position_feedback)]
        return self.registers.get(address, 0)

    def setpoints(self, kind: str):
        base = reg.SETPOINT_BASE[kind]
        return [self.registers[base + axis] for axis in range(reg.NUM_AXES)]

    def set_position_percent(self, axis: int, percent: float) -> None:
        """Place an axis, as the caller would see it through feedback."""
        low, high = reg.AXIS_LIMITS[axis]
        raw = int(round(low + (high - low) * percent / 100.0))
        self.registers[reg.SETPOINT_BASE["position"] + (axis - 1)] = raw

    # -- protocol -------------------------------------------------------
    def handle(self, frame: bytes) -> bytes:
        assert crc16(frame[:-2]) == struct.unpack("<H", frame[-2:])[0], "request CRC mismatch"
        assert frame[0] == self.modbus_id

        function_code = frame[1]
        address = struct.unpack(">H", frame[2:4])[0]

        if self.exception_code is not None:
            body = bytes([self.modbus_id, function_code | 0x80, self.exception_code])
            self.exception_code = None
            return self._with_crc(body)

        if function_code == reg.READ_HOLDING_REGISTERS:
            count = struct.unpack(">H", frame[4:6])[0]
            payload = b"".join(struct.pack(">H", self.read(address + i) & 0xFFFF) for i in range(count))
            return self._with_crc(bytes([self.modbus_id, function_code, len(payload)]) + payload)

        if function_code == reg.WRITE_SINGLE_REGISTER:
            value = struct.unpack(">H", frame[4:6])[0]
            self.registers[address] = value
            self.writes.append((address, [value]))
            return self._with_crc(frame[:6])

        if function_code == reg.WRITE_MULTIPLE_REGISTERS:
            count = struct.unpack(">H", frame[4:6])[0]
            values = [struct.unpack(">H", frame[7 + 2 * i:9 + 2 * i])[0] for i in range(count)]
            for offset, value in enumerate(values):
                self.registers[address + offset] = value
            self.writes.append((address, values))
            return self._with_crc(frame[:6])

        raise AssertionError(f"Unsupported function code 0x{function_code:02X}")

    def _with_crc(self, body: bytes) -> bytes:
        return bytes(body) + struct.pack("<H", crc16(bytes(body)))


class FakeSerial:
    """Just enough of `serial.Serial` for the transport to talk to."""

    def __init__(self, device: FakeDH5):
        self.device = device
        self.is_open = True
        self._buffer = b""

    def write(self, frame):
        self._buffer += self.device.handle(bytes(frame))
        return len(frame)

    def read(self, size=1):
        chunk, self._buffer = self._buffer[:size], self._buffer[size:]
        return chunk

    def reset_input_buffer(self):
        self._buffer = b""

    def close(self):
        self.is_open = False


@pytest.fixture
def device():
    return FakeDH5()


@pytest.fixture
def api(device):
    api = DH5ModbusAPI(port="FAKE")
    api.serial_connection = FakeSerial(device)
    return api


@pytest.fixture
def hand(api):
    # poll_interval 0 keeps the wait loops from sleeping during tests.
    return DH5Hand(api=api, poll_interval=0)
