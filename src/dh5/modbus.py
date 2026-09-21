"""Modbus RTU transport for the DH5 hand.

This is the vendor `DH5ModbusAPI` class, kept method-for-method compatible
with the original `DH5.py` so existing code (the Tkinter GUI, the ROS2
controller node) keeps working, with the following fixes:

* `set_axis_force()` addressed the wrong register for axes 2-6
  (`+ (axis - 1) * 0x10` instead of `+ (axis - 1)`).
* `set_speed()` / `set_force()` spun forever in their validation loops
  (`i` was never incremented).
* Every command blocked for the full serial timeout, because it always
  asked for 256 bytes and waited for them. Responses are now read at their
  exact expected length, which takes a round trip instead of a second.
* Modbus exception responses (function code | 0x80) were reported as a
  generic "invalid response"; they now surface the device's exception code.
* `open_connection()` / `close_connection()` returned `None` on some paths.

See `dh5.registers` for where the addresses come from.
"""

import logging
import struct
import threading
from typing import Sequence

import serial

from . import registers as reg

logger = logging.getLogger(__name__)

# Modbus exception codes the DH5 may return in a 0x80-flagged response.
MODBUS_EXCEPTION_NAMES = {
    0x01: "illegal function",
    0x02: "illegal data address",
    0x03: "illegal data value",
    0x04: "slave device failure",
    0x06: "slave device busy",
}


class DH5ModbusAPI:
    """Serial Modbus RTU client for a DH5 six-axis hand.

    All traffic funnels through `send_modbus_command()`, which is guarded by
    a re-entrant lock so a background polling thread (the GUI's feedback
    loop, a ROS2 timer) cannot interleave its frame with a write from
    another thread and desynchronise the bus.
    """

    SUCCESS = 0
    ERROR_CONNECTION_FAILED = 1
    ERROR_INVALID_RESPONSE = 2
    ERROR_CRC_CHECK_FAILED = 3
    ERROR_INVALID_COMMAND = 4

    def __init__(self, port='COM6', modbus_id=1, baud_rate=115200, stop_bits=1, parity='N', timeout=1.0):
        self.port = port
        self.modbus_id = modbus_id
        self.baud_rate = baud_rate
        self.stop_bits = stop_bits
        self.parity = parity
        self.timeout = timeout
        self.serial_connection = None
        self._lock = threading.RLock()

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------
    def open_connection(self):
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                stopbits=self.stop_bits,
                parity=self.parity,
                timeout=self.timeout,
            )
        except Exception as exc:
            logger.error("Failed to open serial connection on %s: %s", self.port, exc)
            return f"Failed to open serial connection: {exc}"

        if not self.serial_connection.is_open:
            logger.error("Serial port %s did not report itself open.", self.port)
            return self.ERROR_CONNECTION_FAILED

        logger.info("Serial connection opened on %s at %s baud.", self.port, self.baud_rate)
        return self.SUCCESS

    def close_connection(self):
        with self._lock:
            if self.serial_connection and self.serial_connection.is_open:
                self.serial_connection.close()
                logger.info("Serial connection on %s closed.", self.port)
            return self.SUCCESS

    @property
    def is_connected(self) -> bool:
        return bool(self.serial_connection and self.serial_connection.is_open)

    # ------------------------------------------------------------------
    # Frame construction / parsing
    # ------------------------------------------------------------------
    def send_modbus_command(self, function_code, register_address, data=None, data_length=None):
        """Build, send and parse one Modbus RTU frame.

        Returns a list of 16-bit register values for reads (0x03), or
        `SUCCESS` for writes (0x06 / 0x10). Errors come back as one of the
        `ERROR_*` constants or a descriptive string, matching the original
        vendor behaviour.
        """
        if not self.is_connected:
            return self.ERROR_CONNECTION_FAILED

        try:
            if function_code == reg.READ_HOLDING_REGISTERS:
                count = data_length or 1
                message = self._build_request(function_code, register_address, data_length=count)
                expected = 5 + 2 * count  # id + fc + bytecount + payload + crc
            elif function_code == reg.WRITE_SINGLE_REGISTER:
                message = self._build_request(function_code, register_address, value=data)
                expected = 8  # echo of the request
            elif function_code == reg.WRITE_MULTIPLE_REGISTERS:
                message = self._build_request(
                    function_code, register_address, values=data, data_length=data_length
                )
                expected = 8  # id + fc + address + count + crc
            else:
                return self.ERROR_INVALID_COMMAND

            with self._lock:
                # Drop anything stale left over from a timed-out previous
                # exchange, so we don't parse the tail of an old frame.
                self.serial_connection.reset_input_buffer()
                self.serial_connection.write(message)
                response = self._read_response(expected)

            return self._parse_response(response, function_code)
        except Exception as exc:
            logger.exception("Modbus command 0x%02X on 0x%04X failed.", function_code, register_address)
            return f"Error: {exc}"

    def _read_response(self, expected_length: int) -> bytes:
        """Read exactly the number of bytes this response should contain.

        The original implementation asked for 256 bytes, which meant every
        single command waited out the full serial timeout before returning.
        Reading the two-byte header first lets us detect a Modbus exception
        frame (which is always 5 bytes) and size the rest of the read
        correctly.
        """
        header = self.serial_connection.read(2)
        if len(header) < 2:
            return header

        function_code = header[1]
        if function_code & 0x80:
            remaining = 3  # exception code + CRC
        else:
            remaining = expected_length - 2

        return header + self.serial_connection.read(max(remaining, 0))

    def _build_request(self, function_code, register_address, data_length=1, value=None, values=None):
        request = bytearray()
        request.append(self.modbus_id)
        request.append(function_code)
        request += struct.pack('>H', register_address)

        if function_code == reg.READ_HOLDING_REGISTERS:
            request += struct.pack('>H', data_length)
        elif function_code == reg.WRITE_SINGLE_REGISTER:
            request += struct.pack('>H', value)
        elif function_code == reg.WRITE_MULTIPLE_REGISTERS:
            register_count = data_length if data_length else len(values)
            request += struct.pack('>H', register_count)
            request.append(register_count * 2)  # byte count
            for val in values:
                request += struct.pack('>H', val)

        request += struct.pack('<H', self._calculate_crc(request))
        return bytes(request)

    @staticmethod
    def _calculate_crc(data) -> int:
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def _parse_response(self, response, function_code):
        if len(response) < 5:
            logger.debug("Incomplete response (%d bytes): %r", len(response), response)
            return self.ERROR_INVALID_RESPONSE

        crc_received = (response[-1] << 8) | response[-2]
        if crc_received != self._calculate_crc(response[:-2]):
            logger.warning("CRC mismatch on response %r", response)
            return self.ERROR_CRC_CHECK_FAILED

        returned_code = response[1]
        if returned_code & 0x80:
            code = response[2]
            name = MODBUS_EXCEPTION_NAMES.get(code, "unknown")
            logger.error("Device returned Modbus exception 0x%02X (%s).", code, name)
            return f"Modbus exception 0x{code:02X} ({name})"

        if returned_code != function_code:
            logger.warning("Unexpected function code 0x%02X (wanted 0x%02X).", returned_code, function_code)
            return self.ERROR_INVALID_RESPONSE

        if function_code == reg.READ_HOLDING_REGISTERS:
            byte_count = response[2]
            payload = response[3:3 + byte_count]
            if len(payload) != byte_count:
                return self.ERROR_INVALID_RESPONSE
            return [
                struct.unpack('>H', payload[i:i + 2])[0]
                for i in range(0, len(payload), 2)
            ]
        return self.SUCCESS

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------
    def set_config(self, modbus_id=None, baud_rate=None, stop_bits=None, parity=None):
        if modbus_id:
            self.modbus_id = modbus_id
        if baud_rate:
            self.baud_rate = baud_rate
        if stop_bits:
            self.stop_bits = stop_bits
        if parity:
            self.parity = parity

    def set_uart_config(self, modbus_id=None, baud_rate=None, stop_bits=None, parity=None):
        uart_registers = [modbus_id, baud_rate, stop_bits, parity]
        return self.send_modbus_command(
            function_code=reg.WRITE_MULTIPLE_REGISTERS,
            register_address=reg.UART_CONFIG_REGISTER,
            data=uart_registers,
            data_length=len(uart_registers),
        )

    def set_save_param(self, flag=1):
        return self.send_modbus_command(
            function_code=reg.WRITE_SINGLE_REGISTER,
            register_address=reg.SAVE_PARAMETERS_REGISTER,
            data=flag,
        )

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------
    def initialize(self, mode):
        """Initialize all 6 axes with one mode (0b01 close, 0b10 open,
        0b11 find total stroke)."""
        if mode not in reg.INIT_MODES:
            return self.ERROR_INVALID_COMMAND

        data = 0
        for axis in range(reg.NUM_AXES):
            data |= (mode << (axis * 2))
        return self.send_modbus_command(
            function_code=reg.WRITE_SINGLE_REGISTER,
            register_address=reg.INITIALIZE_COMMAND_REGISTER,
            data=data,
        )

    def initialize_axis(self, axis, mode):
        """Initialize one axis. The other axes receive 0b00 in their bit
        pair, which the protocol reads as 'no action'."""
        if not 1 <= axis <= reg.NUM_AXES:
            return self.ERROR_INVALID_COMMAND
        if mode not in reg.INIT_MODES:
            return self.ERROR_INVALID_COMMAND

        init_status = mode << ((axis - 1) * 2)
        return self.send_modbus_command(
            function_code=reg.WRITE_SINGLE_REGISTER,
            register_address=reg.INITIALIZE_COMMAND_REGISTER,
            data=init_status,
        )

    def check_initialization(self):
        """Return {'axis_F1': 'initialized' | 'initializing' | 'not
        initialized', ...} for all six axes."""
        response = self.send_modbus_command(
            function_code=reg.READ_HOLDING_REGISTERS,
            register_address=reg.INITIALIZE_STATUS_REGISTER,
            data_length=1,
        )
        if isinstance(response, list) and response:
            init_status = response[0]
            labels = {0b01: "initialized", 0b10: "initializing"}
            return {
                f"axis_F{axis + 1}": labels.get((init_status >> (axis * 2)) & 0b11, "not initialized")
                for axis in range(reg.NUM_AXES)
            }
        return self.ERROR_INVALID_RESPONSE

    # ------------------------------------------------------------------
    # Whole-hand setpoints (all six axes in one frame)
    # ------------------------------------------------------------------
    def set_position(self, position: Sequence[int]):
        if len(position) != reg.NUM_AXES:
            return self.ERROR_INVALID_COMMAND
        return self.send_modbus_command(
            function_code=reg.WRITE_MULTIPLE_REGISTERS,
            register_address=reg.SETPOINT_BASE["position"],
            data=list(position),
            data_length=reg.NUM_AXES,
        )

    def set_speed(self, speed: Sequence[int]):
        if len(speed) != reg.NUM_AXES:
            return self.ERROR_INVALID_COMMAND
        if any(not 1 <= value <= 100 for value in speed):
            return self.ERROR_INVALID_COMMAND
        return self.send_modbus_command(
            function_code=reg.WRITE_MULTIPLE_REGISTERS,
            register_address=reg.SETPOINT_BASE["speed"],
            data=list(speed),
            data_length=reg.NUM_AXES,
        )

    def set_force(self, force: Sequence[int]):
        if len(force) != reg.NUM_AXES:
            return self.ERROR_INVALID_COMMAND
        if any(not 20 <= value <= 100 for value in force):
            return self.ERROR_INVALID_COMMAND
        return self.send_modbus_command(
            function_code=reg.WRITE_MULTIPLE_REGISTERS,
            register_address=reg.SETPOINT_BASE["force"],
            data=list(force),
            data_length=reg.NUM_AXES,
        )

    def read_block(self, register_address: int, count: int = reg.NUM_AXES):
        """Read `count` consecutive holding registers in one frame."""
        return self.send_modbus_command(
            function_code=reg.READ_HOLDING_REGISTERS,
            register_address=register_address,
            data_length=count,
        )

    def get_seted_position(self):
        return self.read_block(reg.SETPOINT_BASE["position"])

    def get_seted_speed(self):
        return self.read_block(reg.SETPOINT_BASE["speed"])

    def get_seted_force(self):
        return self.read_block(reg.SETPOINT_BASE["force"])

    def get_position_fd(self):
        return self.read_block(reg.FEEDBACK_BASE["position"])

    def get_speed_fd(self):
        return self.read_block(reg.FEEDBACK_BASE["speed"])

    def get_current_fd(self):
        return self.read_block(reg.FEEDBACK_BASE["current"])

    # ------------------------------------------------------------------
    # Per-axis setpoints
    # ------------------------------------------------------------------
    def set_axis_position(self, axis, position):
        return self.write_axis_setpoint("position", axis, position)

    def set_axis_speed(self, axis, speed):
        return self.write_axis_setpoint("speed", axis, speed)

    def set_axis_force(self, axis, force):
        # The original used `0x0107 + (axis - 1) * 0x10`, which put axis 2's
        # force at 0x0117 instead of 0x0108 and so on for every axis above 1.
        return self.write_axis_setpoint("force", axis, force)

    def write_axis_setpoint(self, kind: str, axis: int, value):
        if not 1 <= axis <= reg.NUM_AXES:
            return self.ERROR_INVALID_COMMAND
        return self.send_modbus_command(
            function_code=reg.WRITE_SINGLE_REGISTER,
            register_address=reg.setpoint_register(kind, axis),
            data=int(value),
        )

    # ------------------------------------------------------------------
    # Per-axis feedback
    # ------------------------------------------------------------------
    def get_axis_position(self, axis):
        return self._read_axis_feedback("position", axis)

    def get_axis_speed(self, axis):
        return self._read_axis_feedback("speed", axis)

    def get_axis_current(self, axis):
        return self._read_axis_feedback("current", axis)

    def _read_axis_feedback(self, kind: str, axis: int):
        if not 1 <= axis <= reg.NUM_AXES:
            return self.ERROR_INVALID_COMMAND
        return self.send_modbus_command(
            function_code=reg.READ_HOLDING_REGISTERS,
            register_address=reg.feedback_register(kind, axis),
            data_length=1,
        )

    # ------------------------------------------------------------------
    # Faults
    # ------------------------------------------------------------------
    def get_cur_faults(self):
        return self.send_modbus_command(
            function_code=reg.READ_HOLDING_REGISTERS,
            register_address=reg.CURRENT_FAULT_REGISTER,
            data_length=1,
        )

    def get_history_faults(self):
        return self.send_modbus_command(
            function_code=reg.READ_HOLDING_REGISTERS,
            register_address=reg.HISTORY_FAULT_REGISTER,
            data_length=reg.HISTORY_FAULT_COUNT,
        )

    def reset_faults(self):
        return self.send_modbus_command(
            function_code=reg.WRITE_SINGLE_REGISTER,
            register_address=reg.CLEAR_FAULTS_REGISTER,
            data=1,
        )

    def reset_history_faults(self):
        return self.send_modbus_command(
            function_code=reg.WRITE_SINGLE_REGISTER,
            register_address=reg.CLEAR_HISTORY_FAULTS_REGISTER,
            data=1,
        )

    def restart_system(self):
        return self.send_modbus_command(
            function_code=reg.WRITE_SINGLE_REGISTER,
            register_address=reg.RESTART_SYSTEM_REGISTER,
            data=1,
        )

