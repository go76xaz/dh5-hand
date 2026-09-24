"""The command registry: parsing, dispatch, and help that cannot drift."""

import pytest

from dh5 import cli, gestures
from dh5 import registers as reg
from dh5.cli import UsageError, parse_axes, parse_axis_values


class TestParsing:
    def test_single_axis(self):
        assert parse_axes("3") == [3]

    def test_comma_list(self):
        assert parse_axes("1,3,5") == [1, 3, 5]

    def test_comma_list_tolerates_spaces(self):
        assert parse_axes("1, 3 ,5") == [1, 3, 5]

    def test_all_expands_to_every_axis(self):
        assert parse_axes("all") == [1, 2, 3, 4, 5, 6]

    @pytest.mark.parametrize("arg", ["0", "7", "x", "", "1,9", "-1"])
    def test_invalid_axis_arguments_are_usage_errors(self, arg):
        with pytest.raises(UsageError):
            parse_axes(arg)

    def test_axis_value_pairs(self):
        assert parse_axis_values(["2", "100", "3", "50"]) == {2: 100.0, 3: 50.0}

    def test_fractional_values_survive(self):
        assert parse_axis_values(["2", "58.5"]) == {2: 58.5}

    def test_all_sets_every_axis_to_the_same_value(self):
        assert parse_axis_values(["all", "10"]) == {axis: 10.0 for axis in range(1, 7)}

    def test_all_is_case_insensitive(self):
        assert parse_axis_values(["ALL", "10"]) == {axis: 10.0 for axis in range(1, 7)}

    def test_all_rejects_a_non_numeric_value(self):
        with pytest.raises(UsageError):
            parse_axis_values(["all", "fast"])

    @pytest.mark.parametrize("args", [
        [],                      # nothing
        ["2"],                   # odd length
        ["2", "100", "3"],       # odd length
        ["9", "100"],            # axis out of range
        ["2", "fast"],           # non-numeric value
        ["all"],                 # 'all' with no value
        ["all", "10", "20"],     # 'all' with extra arguments
    ])
    def test_malformed_pairs_are_usage_errors(self, args):
        with pytest.raises(UsageError):
            parse_axis_values(args)


class TestRegistry:
    def test_every_command_in_the_help_text_can_be_dispatched(self):
        """Regression: `getspeed` was documented in the old HELP_TEXT but
        had no branch, so typing it printed 'Unknown command'."""
        text = cli.help_text()
        for command in cli.COMMANDS.values():
            assert command.usage in text
            assert command.name in cli.COMMANDS

    def test_getspeed_exists(self):
        assert "getspeed" in cli.COMMANDS

    def test_help_lists_the_exit_commands(self):
        text = cli.help_text()
        for name in cli.EXIT_COMMANDS:
            assert name in text

    def test_every_gesture_became_a_command(self):
        for name in gestures.GESTURES:
            assert name in cli.COMMANDS, f"{name} should be dispatchable without hand-written code"

    def test_no_command_is_registered_twice_under_one_name(self):
        seen = {}
        for name, command in cli.COMMANDS.items():
            seen.setdefault(command.name, set()).add(name)
        assert len(cli.COMMANDS) == sum(len(names) for names in seen.values())

    def test_every_command_documents_itself(self):
        for command in cli.COMMANDS.values():
            assert command.summary and command.usage


class TestHandlers:
    def run(self, name, hand, args):
        cli.COMMANDS[name].handler(hand, args)

    def test_setpos_moves_a_single_axis(self, hand, device):
        self.run("setpos", hand, ["2", "100"])
        assert device.registers[reg.SETPOINT_BASE["position"] + 1] == reg.AXIS_LIMITS[2][1]

    def test_setpos_moves_several_axes_in_one_frame(self, hand, device):
        self.run("setpos", hand, ["2", "100", "5", "100"])
        address, values = device.writes[-1]
        assert address == reg.SETPOINT_BASE["position"]
        assert len(values) == reg.NUM_AXES

    def test_setspeed_leaves_unnamed_axes_alone(self, hand, device):
        for axis in range(reg.NUM_AXES):
            device.registers[reg.SETPOINT_BASE["speed"] + axis] = 60
        self.run("setspeed", hand, ["2", "25"])
        assert device.setpoints("speed") == [60, 25, 60, 60, 60, 60]

    def test_setforce_leaves_unnamed_axes_alone(self, hand, device):
        for axis in range(reg.NUM_AXES):
            device.registers[reg.SETPOINT_BASE["force"] + axis] = 80
        self.run("setforce", hand, ["1", "35"])
        assert device.setpoints("force") == [35, 80, 80, 80, 80, 80]

    def test_setspeed_all_sets_every_axis(self, hand, device):
        self.run("setspeed", hand, ["all", "10"])
        assert device.setpoints("speed") == [10] * reg.NUM_AXES

    def test_setpos_all_moves_every_axis(self, hand, device):
        self.run("setpos", hand, ["all", "50"])
        assert device.setpoints("position") == [
            round((reg.AXIS_LIMITS[axis][1]) * 0.5) for axis in range(1, reg.NUM_AXES + 1)
        ]

    @pytest.mark.parametrize("name", ["getpos", "getspeed", "getforce", "getcurrent", "getstatus"])
    def test_readers_accept_all(self, name, hand):
        self.run(name, hand, ["all"])

    @pytest.mark.parametrize("name", ["getpos", "getspeed", "getforce", "getcurrent", "getstatus"])
    def test_readers_need_exactly_one_argument(self, name, hand):
        with pytest.raises(UsageError):
            self.run(name, hand, [])
        with pytest.raises(UsageError):
            self.run(name, hand, ["1", "2"])

    def test_getsensors_reads_one_finger(self, hand):
        self.run("getsensors", hand, ["1"])

    def test_getsensors_reads_every_finger(self, hand):
        self.run("getsensors", hand, ["all"])

    def test_getsensors_raw_mode(self, hand):
        self.run("getsensors", hand, ["1", "raw"])

    @pytest.mark.parametrize("args", [["pinky"], ["thumb"], ["6"], ["1", "sideways"], ["1", "watch", "soon"]])
    def test_getsensors_rejects_bad_arguments(self, hand, args):
        with pytest.raises(UsageError):
            self.run("getsensors", hand, args)

    @pytest.mark.parametrize("name", ["faults", "reset", "history", "checkinit"])
    def test_no_argument_commands_reject_arguments(self, name, hand):
        self.run(name, hand, [])
        with pytest.raises(UsageError):
            self.run(name, hand, ["extra"])

    def test_initialize_axis_validates_its_mode(self, hand):
        self.run("initialize_axis", hand, ["1", "2"])
        with pytest.raises(UsageError):
            self.run("initialize_axis", hand, ["1", "9"])
        with pytest.raises(UsageError):
            self.run("initialize_axis", hand, ["1"])


class TestGestureCommands:
    def open_everything(self, device):
        for axis in range(1, 7):
            device.set_position_percent(axis, 100)

    def test_a_gesture_runs_with_no_arguments(self, hand, device):
        self.open_everything(device)
        cli.COMMANDS["round_grip"].handler(hand, [])

    def test_pinch_accepts_a_width(self, hand, device):
        self.open_everything(device)
        cli.COMMANDS["two_finger_pinch"].handler(hand, ["10"])

    def test_pinch_accepts_a_width_and_a_variant(self, hand, device):
        self.open_everything(device)
        cli.COMMANDS["two_finger_pinch"].handler(hand, ["10", "axis3"])

    @pytest.mark.parametrize("args", [["wide"], ["10", "axis9"], ["10", "axis2", "extra"], ["99"]])
    def test_pinch_rejects_bad_arguments(self, hand, device, args):
        self.open_everything(device)
        with pytest.raises(UsageError):
            cli.COMMANDS["two_finger_pinch"].handler(hand, args)

    def test_a_fixed_gesture_rejects_arguments(self, hand, device):
        self.open_everything(device)
        with pytest.raises(UsageError):
            cli.COMMANDS["open_hand"].handler(hand, ["nonsense"])

    def test_usage_line_shows_width_and_variants(self):
        usage = cli.COMMANDS["two_finger_pinch"].usage
        assert "width" in usage and "axis2" in usage and "axis3" in usage
