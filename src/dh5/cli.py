"""The interactive `DH5>` prompt.

Commands live in a registry rather than an if/elif chain, so the help text
is generated from the same objects that dispatch. A command cannot be
documented but unimplemented (as `getspeed` was), or implemented but
undocumented, because there is only one list.

Gesture commands are not written out here at all - they are generated from
`dh5.gestures.GESTURES`, so a new entry in that table becomes a new prompt
command with correct help and argument parsing for free.
"""

import logging
import shlex
from dataclasses import dataclass
from typing import Callable, Dict, List, Sequence

from . import gestures
from . import registers as reg
from .hand import DH5Hand, format_faults

logger = logging.getLogger(__name__)

PROMPT = "DH5> "
EXIT_COMMANDS = ("close", "exit", "quit")


@dataclass(frozen=True)
class Command:
    """One prompt command: how to call it, what it does, and who handles it."""

    name: str
    usage: str
    summary: str
    handler: Callable[[DH5Hand, List[str]], None]
    aliases: Sequence[str] = ()

    @property
    def names(self):
        return (self.name, *self.aliases)


class UsageError(Exception):
    """Raised by a handler when its arguments do not parse. The loop turns
    this into the command's usage line rather than a traceback."""


# --------------------------------------------------------------------------
# Argument parsing
# --------------------------------------------------------------------------
def parse_axes(arg: str, num_axes: int = reg.NUM_AXES) -> List[int]:
    """'3' -> [3];  '1,3,5' -> [1, 3, 5];  'all' -> [1..num_axes]."""
    if arg == "all":
        return list(range(1, num_axes + 1))

    axes = []
    for part in arg.split(","):
        part = part.strip()
        if not part.isdigit() or not (1 <= int(part) <= num_axes):
            raise UsageError(f"Invalid axis {part!r}. Use 1-{num_axes}, a list like 1,3,5, or 'all'.")
        axes.append(int(part))
    if not axes:
        raise UsageError("No axis given.")
    return axes


def parse_axis_values(args: Sequence[str], num_axes: int = reg.NUM_AXES) -> Dict[int, float]:
    """Parse a flat '<axis> <value> [<axis> <value> ...]' sequence, or
    'all <value>' to apply the same value to every axis."""
    if len(args) == 2 and args[0].lower() == "all":
        try:
            value = float(args[1])
        except ValueError:
            raise UsageError(f"Invalid value {args[1]!r}. Must be a number.") from None
        return {axis: value for axis in range(1, num_axes + 1)}

    if not args or len(args) % 2 != 0:
        raise UsageError("Expected pairs of <axis> <value>, or 'all <value>'.")

    axis_values: Dict[int, float] = {}
    for axis_str, value_str in zip(args[::2], args[1::2]):
        if not axis_str.isdigit() or not (1 <= int(axis_str) <= num_axes):
            raise UsageError(f"Invalid axis {axis_str!r}. Must be 1-{num_axes}.")
        try:
            axis_values[int(axis_str)] = float(value_str)
        except ValueError:
            raise UsageError(f"Invalid value {value_str!r}. Must be a number.") from None
    return axis_values


def _require_none(args):
    if args:
        raise UsageError("This command takes no arguments.")


def _require_one(args):
    if len(args) != 1:
        raise UsageError("Expected exactly one argument.")


# --------------------------------------------------------------------------
# Command registry
# --------------------------------------------------------------------------
COMMANDS: Dict[str, Command] = {}
_ORDER: List[Command] = []


def command(name: str, usage: str, summary: str, aliases: Sequence[str] = ()):
    """Register a prompt command. The same object supplies dispatch and help."""

    def register(handler):
        cmd = Command(name=name, usage=usage, summary=summary, handler=handler, aliases=aliases)
        for key in cmd.names:
            COMMANDS[key] = cmd
        _ORDER.append(cmd)
        return handler

    return register


# --------------------------------------------------------------------------
# Setting values
# --------------------------------------------------------------------------
def _report_write(hand, what: str, result) -> bool:
    """Say plainly whether a write was accepted. The library returns
    `SUCCESS` (0) or an error code / message, which is meaningless to read
    raw at the prompt."""
    if result == hand.api.SUCCESS:
        logger.info("OK: %s", what)
        return True
    logger.error("FAILED: %s - device/bus returned %r", what, result)
    return False


def _check_range(name: str, values: Dict[int, float], low: float, high: float) -> None:
    for axis, value in values.items():
        if not low <= value <= high:
            raise UsageError(f"{name} for axis {axis} is {value:g}; must be {low:g}-{high:g}.")


@command("setpos", "setpos <axis> <percent> [<axis> <percent> ...] | setpos all <percent>",
         "Move axes to percent positions (0-100). Several pairs move together in one frame; "
         "'all <percent>' moves every axis to the same position.")
def _setpos(hand, args):
    targets = parse_axis_values(args, hand.num_axes)
    _check_range("Position", targets, 0, 100)
    move = hand.move(targets)
    if not _report_write(hand, f"setpos {targets}", move.result):
        return
    for axis, status in sorted((move.statuses or {}).items()):
        if status == 2:
            logger.warning("Axis %d stalled before reaching its target.", axis)
        elif status == 0:
            logger.warning("Axis %d is still moving (timed out waiting).", axis)
        else:
            logger.info("Axis %d reached position.", axis)


@command("setspeed", "setspeed <axis> <value> [<axis> <value> ...] | setspeed all <value>",
         "Set axis speed (1-100). Axes not listed keep their current setpoint; "
         "'all <value>' sets every axis to the same speed.")
def _setspeed(hand, args):
    values = parse_axis_values(args, hand.num_axes)
    _check_range("Speed", values, 1, 100)
    ints = {axis: int(v) for axis, v in values.items()}
    _report_write(hand, f"setspeed {ints}", hand.set_speeds(ints))


@command("setforce", "setforce <axis> <value> [<axis> <value> ...] | setforce all <value>",
         "Set axis force (20-100). Axes not listed keep their current setpoint; "
         "'all <value>' sets every axis to the same force.")
def _setforce(hand, args):
    values = parse_axis_values(args, hand.num_axes)
    _check_range("Force", values, 20, 100)
    ints = {axis: int(v) for axis, v in values.items()}
    _report_write(hand, f"setforce {ints}", hand.set_forces(ints))


# --------------------------------------------------------------------------
# Reading values
# --------------------------------------------------------------------------
@command("getpos", "getpos <axis|all>", "Read measured position, raw and as a percentage.")
def _getpos(hand, args):
    _require_one(args)
    for axis in parse_axes(args[0], hand.num_axes):
        raw = hand.read_feedback("position", axis)
        percent = hand.position_percent(axis)
        logger.info("Axis %d: position=%s (%.1f%%)", axis, raw, percent)


@command("getspeed", "getspeed <axis|all>", "Read measured speed alongside the commanded setpoint.")
def _getspeed(hand, args):
    _require_one(args)
    for axis in parse_axes(args[0], hand.num_axes):
        logger.info("Axis %d: speed=%s (setpoint %s)",
                    axis, hand.read_feedback("speed", axis), hand.read_setpoint("speed", axis))


@command("getforce", "getforce <axis|all>",
         "Read the commanded force setpoint (the DH5 exposes no force feedback register).")
def _getforce(hand, args):
    _require_one(args)
    for axis in parse_axes(args[0], hand.num_axes):
        logger.info("Axis %d: force setpoint=%s", axis, hand.read_setpoint("force", axis))


@command("getcurrent", "getcurrent <axis|all>", "Read current draw per axis.")
def _getcurrent(hand, args):
    _require_one(args)
    for axis in parse_axes(args[0], hand.num_axes):
        logger.info("Axis %d: current=%s", axis, hand.read_feedback("current", axis))


@command("getstatus", "getstatus <axis|all>",
         "Read operating status: moving / reached position / stalled.")
def _getstatus(hand, args):
    _require_one(args)
    for axis in parse_axes(args[0], hand.num_axes):
        logger.info("Axis %d: %s", axis, hand.axis_status_label(axis))


@command("getsensors", "getsensors <1-5|all> [raw|watch] [interval]",
         "Read fingertip sensors (1=thumb .. 5=little). 'raw' dumps registers with both byte orders; "
         "'watch' polls every [interval] seconds until Ctrl+C.")
def _getsensors(hand, args):
    if not 1 <= len(args) <= 3:
        raise UsageError("Expected 1 to 3 arguments.")

    finger_arg = args[0].lower()
    if finger_arg == "all":
        fingers = list(reg.FINGERS)
    elif finger_arg in reg.FINGER_SENSOR_BASE_REGISTER:
        fingers = [finger_arg]
    else:
        raise UsageError(f"Invalid finger {args[0]!r}. Use one of: {', '.join(reg.FINGERS)}, or 'all'.")

    mode = args[1].lower() if len(args) >= 2 else None
    if mode not in (None, "raw", "watch"):
        raise UsageError(f"Invalid mode {args[1]!r}. Use 'raw' or 'watch'.")

    if mode == "watch":
        try:
            interval = float(args[2]) if len(args) == 3 else 0.5
        except ValueError:
            raise UsageError("Interval must be a number of seconds.") from None
        hand.watch_sensors(fingers, interval=interval)
        return

    num_points = hand.sensor_points_per_finger()
    logger.info("Sensor points per finger (reg 0x0222): %s", num_points)
    for finger in fingers:
        if mode == "raw":
            hand.dump_finger_sensor_raw(finger, num_points=num_points)
        else:
            logger.info("%s", hand.describe_finger(finger, num_points))


# --------------------------------------------------------------------------
# Device management
# --------------------------------------------------------------------------
@command("calibrate", "calibrate", "Guided fingertip sensor calibration (register 0x0505).")
def _calibrate(hand, args):
    _require_none(args)
    hand.calibrate_sensors_interactive()


@command("faults", "faults", "Show the current fault status.")
def _faults(hand, args):
    _require_none(args)
    logger.info("Current fault status: %s", format_faults(hand.current_faults()))


@command("reset", "reset", "Clear current faults.")
def _reset(hand, args):
    _require_none(args)
    hand.reset_faults()


@command("history", "history", "Show the stored fault history.")
def _history(hand, args):
    _require_none(args)
    logger.info("Fault history: %s", format_faults(hand.fault_history()))


@command("checkinit", "checkinit", "Show the initialization status of all axes.")
def _checkinit(hand, args):
    _require_none(args)
    logger.info("Initialization status: %s", hand.check_initialization())


@command("initialize_axis", "initialize_axis <axis> <mode>",
         "Initialize one axis. Mode 1 = close, 2 = open, 3 = find total stroke.")
def _initialize_axis(hand, args):
    if len(args) != 2:
        raise UsageError("Expected <axis> and <mode>.")
    try:
        axis, mode = int(args[0]), int(args[1])
    except ValueError:
        raise UsageError("Axis and mode must be integers.") from None
    if mode not in reg.INIT_MODES:
        raise UsageError(f"Mode must be one of {list(reg.INIT_MODES)}.")
    if not 1 <= axis <= hand.num_axes:
        raise UsageError(f"Invalid axis {axis}. Must be 1-{hand.num_axes}.")
    _report_write(hand, f"initialize_axis(axis={axis}, mode={mode})", hand.initialize_axis(axis, mode))


# --------------------------------------------------------------------------
# Gestures, generated from the gesture table
# --------------------------------------------------------------------------
def _make_gesture_handler(gesture):
    def handler(hand, args):
        width = 0.0
        variant = None

        if args and gesture.is_scalable:
            try:
                width = float(args[0])
            except ValueError:
                raise UsageError(f"Width must be a number 0-{gesture.max_width:g}.") from None
            args = args[1:]

        if args:
            variant = args[0].lower()
            if variant not in gesture.variants:
                raise UsageError(
                    f"Unknown variant {args[0]!r}. Use one of: {', '.join(gesture.variant_names)}."
                )
            args = args[1:]

        if args:
            raise UsageError("Too many arguments.")

        try:
            result = gestures.perform(hand, gesture.name, width=width, variant=variant)
        except ValueError as exc:
            raise UsageError(str(exc)) from None

        if result is None:
            logger.info("%s() did not run - preconditions not met.", gesture.name)
        else:
            logger.info("%s() completed.", gesture.name)

    return handler


def _gesture_usage(gesture) -> str:
    parts = [gesture.name]
    if gesture.is_scalable:
        parts.append(f"[width 0-{gesture.max_width:g}]")
    if len(gesture.variants) > 1:
        parts.append("[" + "|".join(gesture.variant_names) + "]")
    return " ".join(parts)


def _register_gestures() -> None:
    for gesture in gestures.GESTURES.values():
        command(gesture.name, _gesture_usage(gesture), gestures.describe(gesture))(
            _make_gesture_handler(gesture)
        )


_register_gestures()


# --------------------------------------------------------------------------
# Help
# --------------------------------------------------------------------------
def help_text() -> str:
    """Render the help screen from the registry, so it cannot drift out of
    sync with what the prompt actually accepts."""
    rows = [(cmd.usage, cmd.summary) for cmd in _ORDER]
    rows.append(("help", "Show this help text."))
    rows.append((" | ".join(EXIT_COMMANDS), "Close the connection and exit."))

    width = max(len(usage) for usage, _ in rows)
    lines = ["", "Available commands (axis = 1-6):", ""]
    lines += [f"  {usage.ljust(width)}  {summary}" for usage, summary in rows]
    lines.append("")
    return "\n".join(lines)


# --------------------------------------------------------------------------
# The loop
# --------------------------------------------------------------------------
def run(hand: DH5Hand) -> None:
    """Read commands until the user exits.

    The hand is never driven unless a command explicitly drives it.
    """
    logger.info("Entering idle mode. Axes are NOT being driven automatically.")
    logger.info("Type 'help' for a list of commands, or 'close' to shut down.")

    while True:
        try:
            raw = input(PROMPT).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            logger.info("Input closed - shutting down.")
            return

        if not raw:
            continue

        try:
            parts = shlex.split(raw)
        except ValueError as exc:
            logger.error("Could not parse %r: %s", raw, exc)
            continue

        name = parts[0].lower()
        args = parts[1:]

        if name in EXIT_COMMANDS:
            return
        if name == "help":
            print(help_text())
            continue

        cmd = COMMANDS.get(name)
        if cmd is None:
            logger.error("Unknown command %r. Type 'help'.", name)
            continue

        try:
            cmd.handler(hand, args)
        except UsageError as exc:
            logger.error("%s", exc)
            logger.error("Usage: %s", cmd.usage)
        except KeyboardInterrupt:
            print()
            logger.warning("%s interrupted.", name)
        except Exception:
            logger.exception("Error while executing %r", raw)
