"""Transport-level behaviour: addressing, framing, and error responses."""

import pytest

from dh5 import registers as reg
from dh5.modbus import DH5ModbusAPI


class TestAxisAddressing:
    @pytest.mark.parametrize("axis", range(1, 7))
    def test_force_setpoint_is_one_register_per_axis(self, api, device, axis):
        """Regression: the original computed `0x0107 + (axis - 1) * 0x10`,
        which put axis 2's force at 0x0117 instead of 0x0108."""
        api.set_axis_force(axis, 77)
        address, _ = device.writes[-1]
        assert address == reg.SETPOINT_BASE["force"] + (axis - 1)
        assert device.registers[address] == 77

    @pytest.mark.parametrize("axis", range(1, 7))
    def test_every_axis_force_lands_in_its_own_register(self, api, device, axis):
        for target in range(1, 7):
            api.set_axis_force(target, 20 + target)
        assert device.setpoints("force") == [21, 22, 23, 24, 25, 26]

    @pytest.mark.parametrize("axis", range(1, 7))
    def test_position_and_speed_addressing(self, api, device, axis):
        api.set_axis_position(axis, 123)
        assert device.registers[reg.SETPOINT_BASE["position"] + axis - 1] == 123
        api.set_axis_speed(axis, 45)
        assert device.registers[reg.SETPOINT_BASE["speed"] + axis - 1] == 45

    @pytest.mark.parametrize("axis", [0, 7, -1])
    def test_out_of_range_axis_is_rejected(self, api, axis):
        assert api.set_axis_force(axis, 50) == api.ERROR_INVALID_COMMAND
        assert api.set_axis_position(axis, 50) == api.ERROR_INVALID_COMMAND
        assert api.get_axis_position(axis) == api.ERROR_INVALID_COMMAND


class TestBlockValidation:
    def test_whole_hand_setters_reject_wrong_lengths(self, api):
        assert api.set_position([1, 2, 3]) == api.ERROR_INVALID_COMMAND
        assert api.set_speed([50] * 3) == api.ERROR_INVALID_COMMAND
        assert api.set_force([50] * 7) == api.ERROR_INVALID_COMMAND

    def test_speed_validation_terminates(self, api):
        """Regression: `while i < len(speed)` never incremented `i`, so an
        out-of-range value hung the process instead of returning."""
        assert api.set_speed([0, 50, 50, 50, 50, 50]) == api.ERROR_INVALID_COMMAND
        assert api.set_speed([101] * 6) == api.ERROR_INVALID_COMMAND

    def test_force_validation_terminates(self, api):
        assert api.set_force([19] * 6) == api.ERROR_INVALID_COMMAND
        assert api.set_force([101] * 6) == api.ERROR_INVALID_COMMAND

    def test_valid_blocks_are_written_in_one_frame(self, api, device):
        assert api.set_speed([10, 20, 30, 40, 50, 60]) == api.SUCCESS
        address, values = device.writes[-1]
        assert address == reg.SETPOINT_BASE["speed"]
        assert values == [10, 20, 30, 40, 50, 60]


class TestResponseHandling:
    def test_read_returns_one_value_per_register(self, api, device):
        device.registers[reg.SETPOINT_BASE["speed"]] = 42
        assert api.read_block(reg.SETPOINT_BASE["speed"], 6)[0] == 42
        assert len(api.read_block(reg.SETPOINT_BASE["speed"], 6)) == 6

    def test_modbus_exception_is_reported_with_its_code(self, api, device):
        device.exception_code = 0x02  # illegal data address
        result = api.send_modbus_command(reg.READ_HOLDING_REGISTERS, 0x9999, data_length=1)
        assert isinstance(result, str)
        assert "0x02" in result and "illegal data address" in result

    def test_truncated_response_is_not_parsed_as_data(self, api):
        api.serial_connection._buffer = b""

        class Mute:
            is_open = True

            def reset_input_buffer(self):
                pass

            def write(self, frame):
                return len(frame)

            def read(self, size=1):
                return b""

        api.serial_connection = Mute()
        assert api.send_modbus_command(
            reg.READ_HOLDING_REGISTERS, reg.CURRENT_FAULT_REGISTER, data_length=1
        ) == api.ERROR_INVALID_RESPONSE

    def test_commands_fail_cleanly_when_disconnected(self):
        api = DH5ModbusAPI(port="FAKE")
        assert api.is_connected is False
        assert api.get_cur_faults() == api.ERROR_CONNECTION_FAILED

    def test_unknown_function_code_is_rejected(self, api):
        assert api.send_modbus_command(0x42, 0x0100, data=1) == api.ERROR_INVALID_COMMAND


class TestInitialization:
    @pytest.mark.parametrize("mode", [0b01, 0b10, 0b11])
    def test_initialize_sets_the_mode_for_every_axis(self, api, device, mode):
        api.initialize(mode)
        expected = sum(mode << (axis * 2) for axis in range(6))
        assert device.registers[reg.INITIALIZE_COMMAND_REGISTER] == expected

    def test_initialize_rejects_an_invalid_mode(self, api):
        assert api.initialize(0) == api.ERROR_INVALID_COMMAND
        assert api.initialize(4) == api.ERROR_INVALID_COMMAND

    @pytest.mark.parametrize("axis", range(1, 7))
    def test_single_axis_initialization_leaves_other_axes_at_no_action(self, api, device, axis):
        api.initialize_axis(axis, 0b10)
        written = device.registers[reg.INITIALIZE_COMMAND_REGISTER]
        assert (written >> ((axis - 1) * 2)) & 0b11 == 0b10
        for other in range(1, 7):
            if other != axis:
                assert (written >> ((other - 1) * 2)) & 0b11 == 0b00

    def test_check_initialization_decodes_each_axis(self, api, device):
        device.registers[reg.INITIALIZE_STATUS_REGISTER] = 0x0555
        assert api.check_initialization() == {f"axis_F{i}": "initialized" for i in range(1, 7)}

        device.registers[reg.INITIALIZE_STATUS_REGISTER] = 0b10  # axis 1 busy, rest untouched
        status = api.check_initialization()
        assert status["axis_F1"] == "initializing"
        assert status["axis_F2"] == "not initialized"
