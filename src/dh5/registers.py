"""DH5 Modbus register map and per-axis limits.

Every address in this module comes from the DH5 Modbus API document
(sections 6.4.2 - 6.4.4). Nothing here talks to hardware; it is pure data,
which is what makes it safe to import from tests.

The single most important distinction in this file is SETPOINT vs FEEDBACK:

    setpoint registers  (0x0101 / 0x0107 / 0x010D)
        what you have COMMANDED. Writing here moves the hand.
    feedback registers  (0x0207 / 0x020D / 0x0213)
        what the hand is ACTUALLY doing right now.

Reading a feedback register and writing the result into a setpoint register
is almost always a bug (a stationary axis reports speed 0, so you would
command speed 0). `SETPOINT_BASE` and `FEEDBACK_BASE` keep the two apart by
name so that mistake is hard to make. See `dh5.hand.DH5Hand.read_setpoint`.
"""

from typing import Dict, Tuple

# --------------------------------------------------------------------------
# Modbus function codes
# --------------------------------------------------------------------------
READ_HOLDING_REGISTERS = 0x03
WRITE_SINGLE_REGISTER = 0x06
WRITE_MULTIPLE_REGISTERS = 0x10

NUM_AXES = 6

# --------------------------------------------------------------------------
# Command / setpoint registers (section 6.4.2)
# One register per axis, contiguous: base + (axis - 1) for axis 1..6.
# --------------------------------------------------------------------------
SETPOINT_BASE: Dict[str, int] = {
    "position": 0x0101,  # 0x0101..0x0106, raw units of 0.01 mm
    "force": 0x0107,     # 0x0107..0x010C, percent
    "speed": 0x010D,     # 0x010D..0x0112, percent
}

# --------------------------------------------------------------------------
# Feedback registers (section 6.4.3)
# --------------------------------------------------------------------------
FEEDBACK_BASE: Dict[str, int] = {
    "position": 0x0207,  # 0x0207..0x020C, raw units of 0.01 mm
    "speed": 0x020D,     # 0x020D..0x0212, percent
    "current": 0x0213,   # 0x0213..0x0218, mA
}

# Force has a setpoint but no documented per-axis feedback register.
FEEDBACK_UNAVAILABLE = ("force",)

AXIS_STATUS_BASE_REGISTER = 0x0201  # 0x0201..0x0206, operating status per axis
AXIS_STATUS_LABELS = {0: "moving", 1: "reached position", 2: "stalled"}

INITIALIZE_COMMAND_REGISTER = 0x0100  # 2 bits per axis
INITIALIZE_STATUS_REGISTER = 0x0200   # 2 bits per axis

CURRENT_FAULT_REGISTER = 0x021F
HISTORY_FAULT_REGISTER = 0x0B00
HISTORY_FAULT_COUNT = 0x3F
# Carried over from the vendor's ROS2 driver (DH5.py); not cross-checked
# against the manual.
CLEAR_HISTORY_FAULTS_REGISTER = 0x0B3F

# --------------------------------------------------------------------------
# System parameter / action registers (section 6.4.4)
#
# All of these share one protocol: write 1, then poll until the register
# resets itself to 0, which signals the action finished.
# --------------------------------------------------------------------------
SAVE_PARAMETERS_REGISTER = 0x0300
UART_CONFIG_REGISTER = 0x0302
CLEAR_FAULTS_REGISTER = 0x0501
RESTART_SYSTEM_REGISTER = 0x0503
BURN_IN_REGISTER = 0x0504
SENSOR_CALIBRATION_REGISTER = 0x0505

# --------------------------------------------------------------------------
# Fingertip sensors (section 6.4.3)
#
# Each finger owns a contiguous block of up to 16 sensor "points". Each point
# is one IEEE-754 float packed into two consecutive 16-bit registers, so a
# full finger block spans 16 * 2 = 32 registers.
# --------------------------------------------------------------------------
SENSOR_POINTS_PER_FINGER_REGISTER = 0x0222  # 2/16 = Saigan, 3 = Hualichuang

FINGER_SENSOR_BASE_REGISTER: Dict[str, int] = {
    "thumb": 0x022A,
    "index": 0x024A,
    "middle": 0x026A,
    "ring": 0x028A,
    "little": 0x02AA,
}
FINGERS = tuple(FINGER_SENSOR_BASE_REGISTER)
MAX_SENSOR_POINTS_PER_FINGER = 16

# Number of sensor points that identifies a Hualichuang 3-axis sensor, whose
# three raw values are (Mx, My, Fz) rather than independent pressure points.
HUALICHUANG_POINTS = 3

# --------------------------------------------------------------------------
# Per-axis motion limits, raw position units (0.01 mm), as (min, max).
# Axes 1 and 6 are the thumb axes and have roughly half the stroke of the
# four finger axes.
# --------------------------------------------------------------------------
AXIS_LIMITS: Dict[int, Tuple[int, int]] = {
    1: (0, 865),
    2: (0, 1683),
    3: (0, 1682),
    4: (0, 1680),
    5: (0, 1685),
    6: (0, 867),
}

# Initialization modes for register 0x0100 (2 bits per axis).
INIT_MODE_CLOSE = 0b01
INIT_MODE_OPEN = 0b10
INIT_MODE_FIND_STROKE = 0b11
INIT_MODES = (INIT_MODE_CLOSE, INIT_MODE_OPEN, INIT_MODE_FIND_STROKE)


def setpoint_register(kind: str, axis: int) -> int:
    """Address of the setpoint register for `kind` ('position', 'force',
    'speed') on `axis` (1..6)."""
    _validate_axis(axis)
    if kind not in SETPOINT_BASE:
        raise ValueError(f"No setpoint register for {kind!r}; known: {sorted(SETPOINT_BASE)}")
    return SETPOINT_BASE[kind] + (axis - 1)


def feedback_register(kind: str, axis: int) -> int:
    """Address of the feedback register for `kind` ('position', 'speed',
    'current') on `axis` (1..6)."""
    _validate_axis(axis)
    if kind in FEEDBACK_UNAVAILABLE:
        raise ValueError(f"The DH5 exposes no per-axis feedback register for {kind!r}.")
    if kind not in FEEDBACK_BASE:
        raise ValueError(f"No feedback register for {kind!r}; known: {sorted(FEEDBACK_BASE)}")
    return FEEDBACK_BASE[kind] + (axis - 1)


def status_register(axis: int) -> int:
    """Address of the operating-status register for `axis` (1..6)."""
    _validate_axis(axis)
    return AXIS_STATUS_BASE_REGISTER + (axis - 1)


def _validate_axis(axis: int, num_axes: int = NUM_AXES) -> None:
    if not isinstance(axis, int) or isinstance(axis, bool):
        raise TypeError(f"Axis must be an int, got {axis!r}")
    if not (1 <= axis <= num_axes):
        raise ValueError(f"Invalid axis {axis}; must be 1-{num_axes}.")
