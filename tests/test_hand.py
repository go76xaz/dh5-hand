"""`DH5Hand`: percent moves, batch writes, and the setpoint/feedback split."""

import pytest

from dh5 import registers as reg
from dh5.hand import DH5Hand, format_faults


class TestSetpointVersusFeedback:
    """The central regression suite.

    A block write has to supply a value for every axis in the block. If
    those filler values come from the feedback registers, an axis that is
    standing still reports speed 0 and gets silently re-commanded to 0.
    """

    def test_setting_one_speed_uses_a_single_register_write(self, hand, device):
        """One axis needs no block write at all, so no filler value is read
        and the question of which register it came from cannot arise."""
        for axis in range(reg.NUM_AXES):
            device.registers[reg.SETPOINT_BASE["speed"] + axis] = 60

        hand.set_speeds({2: 25})

        address, values = device.writes[-1]
        assert (address, len(values)) == (reg.SETPOINT_BASE["speed"] + 1, 1)
        assert device.setpoints("speed") == [60, 25, 60, 60, 60, 60]

    def test_setting_several_speeds_fills_the_rest_from_the_setpoints(self, hand, device):
        """The regression proper: a block write must supply all six values,
        and the four this call does not name have to come from the setpoint
        registers. Feedback speed reads 0 for a stationary axis, so the old
        implementation commanded those four to a standstill."""
        for axis in range(reg.NUM_AXES):
            device.registers[reg.SETPOINT_BASE["speed"] + axis] = 60
        assert all(device.registers[reg.FEEDBACK_BASE["speed"] + a] == 0
                   for a in range(reg.NUM_AXES)), "fake axes must read as stationary"

        hand.set_speeds({1: 10, 6: 90})

        address, values = device.writes[-1]
        assert (address, len(values)) == (reg.SETPOINT_BASE["speed"], reg.NUM_AXES)
        assert device.setpoints("speed") == [10, 60, 60, 60, 60, 90]

    def test_setting_one_force_leaves_the_others_alone(self, hand, device):
        for axis in range(reg.NUM_AXES):
            device.registers[reg.SETPOINT_BASE["force"] + axis] = 80
        hand.set_forces({3: 30, 4: 40})
        assert device.setpoints("force") == [80, 80, 30, 40, 80, 80]

    def test_moving_two_axes_does_not_disturb_the_rest(self, hand, device):
        for axis in range(1, 7):
            device.set_position_percent(axis, 100)
        before = device.setpoints("position")

        hand.move({2: 50, 3: 50}, wait=False)
        after = device.setpoints("position")

        assert after[0] == before[0] and after[3:] == before[3:]
        assert after[1] != before[1] and after[2] != before[2]

    def test_read_setpoint_and_read_feedback_hit_different_registers(self, hand, device):
        device.registers[reg.SETPOINT_BASE["speed"]] = 77
        device.registers[reg.FEEDBACK_BASE["speed"]] = 3
        assert hand.read_setpoint("speed", 1) == 77
        assert hand.read_feedback("speed", 1) == 3

    def test_force_has_no_feedback_register(self, hand):
        with pytest.raises(ValueError):
            hand.read_feedback("force", 1)


class TestMoves:
    def test_single_axis_move_uses_one_write(self, hand, device):
        hand.move({2: 100}, wait=False)
        address, values = device.writes[-1]
        assert address == reg.SETPOINT_BASE["position"] + 1
        assert len(values) == 1

    def test_multi_axis_move_uses_one_block_write(self, hand, device):
        hand.move({2: 100, 5: 100}, wait=False)
        address, values = device.writes[-1]
        assert address == reg.SETPOINT_BASE["position"]
        assert len(values) == reg.NUM_AXES, "all six axes must travel in a single frame"

    def test_percent_is_mapped_through_each_axis_own_limits(self, hand, device):
        hand.move({1: 100, 2: 100}, wait=False)
        assert device.registers[reg.SETPOINT_BASE["position"]] == reg.AXIS_LIMITS[1][1]
        assert device.registers[reg.SETPOINT_BASE["position"] + 1] == reg.AXIS_LIMITS[2][1]

    def test_move_result_unpacks_as_the_legacy_triple(self, hand):
        result, positions, statuses = hand.move({2: 40}, wait=False)
        assert positions == {2: pytest.approx(reg.AXIS_LIMITS[2][1] * 0.4, abs=1)}
        assert statuses is None

    def test_waiting_reports_the_final_status_of_each_axis(self, hand):
        _, _, statuses = hand.move({2: 40, 3: 40}, wait=True)
        assert statuses == {2: 1, 3: 1}

    def test_move_axis_waits_by_default(self, hand):
        """Regression: with the old `wait=False` default, two calls in a row
        overwrote each other and the first target was never reached."""
        _, _, statuses = hand.move_axis(2, 60)
        assert statuses is not None

    @pytest.mark.parametrize("axis", [0, 7, "2"])
    def test_invalid_axis_is_rejected(self, hand, axis):
        with pytest.raises(ValueError):
            hand.move({axis: 50})

    def test_empty_move_is_rejected(self, hand):
        with pytest.raises(ValueError):
            hand.move({})

    def test_wait_for_axes_gives_up_rather_than_hanging(self, hand, device):
        for axis in range(reg.NUM_AXES):
            device.registers[reg.AXIS_STATUS_BASE_REGISTER + axis] = 0  # never arrives
        statuses = hand.wait_for_axes([1, 2], poll_interval=0, timeout=0.05)
        assert statuses == {1: 0, 2: 0}


class TestSensors:
    def test_points_per_finger_identifies_the_sensor_type(self, hand):
        assert hand.sensor_points_per_finger() == reg.HUALICHUANG_POINTS

    def test_reading_a_finger_returns_one_float_per_point(self, hand):
        assert len(hand.read_finger("thumb", num_points=3)) == 3

    def test_all_fingers_are_readable(self, hand):
        readings = hand.read_all_sensors(num_points=3)
        assert set(readings) == set(reg.FINGERS)

    @pytest.mark.parametrize("finger", ["pinky", "", "THUMBS"])
    def test_unknown_finger_is_rejected(self, hand, finger):
        with pytest.raises(ValueError):
            hand.read_finger(finger)

    def test_finger_names_are_case_insensitive(self, hand):
        assert hand.read_finger("THUMB", num_points=3) == hand.read_finger("thumb", num_points=3)

    @pytest.mark.parametrize("num_points", [0, 17, -1])
    def test_point_count_must_fit_the_block(self, hand, num_points):
        with pytest.raises(ValueError):
            hand.read_finger("thumb", num_points=num_points)

    def test_full_reading_carries_raw_and_derived_values(self, hand):
        reading = hand.read_finger_full("index")
        assert set(reading) == {"mx", "my", "fz", "fx", "fy"}


class TestFaultFormatting:
    def test_ints_render_as_hex(self):
        assert format_faults(0x0A) == "0x0A"

    def test_dicts_and_lists_are_mapped_elementwise(self):
        assert format_faults({1: 10}) == {1: "0x0A"}
        assert format_faults([1, 2]) == ["0x01", "0x02"]

    def test_non_hex_strings_pass_through_untouched(self):
        assert format_faults("connection lost") == "connection lost"

    def test_booleans_are_not_mangled_into_hex(self):
        assert format_faults(True) is True


class TestLifecycle:
    def test_context_manager_closes_the_port(self, api):
        with DH5Hand(api=api, poll_interval=0) as hand:
            assert hand.api.is_connected
        assert not api.is_connected

    def test_initialize_refuses_when_disconnected(self):
        hand = DH5Hand(port="FAKE", poll_interval=0)
        assert hand.initialize() is False

    def test_initialize_succeeds_when_all_axes_report_ready(self, hand):
        assert hand.initialize(reg.INIT_MODE_OPEN, timeout=1, poll_interval=0) is True

    def test_initialize_times_out_when_an_axis_never_finishes(self, hand, device):
        device.registers[reg.INITIALIZE_STATUS_REGISTER] = 0b10  # axis 1 still initializing
        assert hand.initialize(reg.INIT_MODE_OPEN, timeout=0.05, poll_interval=0) is False
