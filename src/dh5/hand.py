"""`DH5Hand` - the domain layer on top of the raw Modbus transport.

Where `DH5ModbusAPI` speaks registers, `DH5Hand` speaks axes and percent.
It owns the connection, the per-axis limits, and the two things the raw
class gets wrong on its own:

1. **Setpoint vs feedback.** `move()` and `set_speeds()` write a contiguous
   register block, so they must supply values for axes the caller did not
   mention. Those filler values are read from the *setpoint* registers, not
   the feedback registers. Reading feedback there is how you end up
   commanding speed 0 on every axis that happens to be standing still.

2. **Simultaneity.** Several axes in one `0x10` frame start moving at the
   same instant; a loop of `0x06` writes staggers them by a round trip each.
"""

import logging
import time
from typing import Dict, Iterable, List, Mapping, NamedTuple, Optional, Sequence, Tuple

from . import registers as reg
from .conversions import (
    first_value,
    hualichuang_tangential_force,
    percent_to_position,
    position_to_percent,
    registers_to_float,
    registers_to_floats,
)
from .modbus import DH5ModbusAPI

logger = logging.getLogger(__name__)

# Freshly calibrated, contact-free sensors should read close to zero. This
# is a rule of thumb, not a value from the manual - raise it if your
# sensor's natural zero-point noise is larger.
SENSOR_ZERO_TOLERANCE = 5.0


class MoveResult(NamedTuple):
    """What a move returned. Unpacks as the `(result, positions, statuses)`
    triple the older function API returned."""

    result: object
    positions: Dict[int, int]
    statuses: Optional[Dict[int, int]]


def format_fault_code(code):
    """Render one fault code as hex where that makes sense, else unchanged."""
    if isinstance(code, bool):
        return code
    if isinstance(code, int):
        return f"0x{code:02X}"
    if isinstance(code, str):
        try:
            return f"0x{int(code, 16):02X}"
        except ValueError:
            return code
    return code


def format_faults(faults):
    """Apply `format_fault_code` across a dict, list or bare value."""
    if isinstance(faults, dict):
        return {axis: format_fault_code(code) for axis, code in faults.items()}
    if isinstance(faults, (list, tuple)):
        return [format_fault_code(code) for code in faults]
    return format_fault_code(faults)


class DH5Hand:
    """High-level control of a DH5 six-axis hand.

    Construct either around an existing transport::

        hand = DH5Hand(api=DH5ModbusAPI(port="COM3"))

    or let it build one::

        with DH5Hand(port="COM3") as hand:
            hand.initialize()
            hand.move({2: 100, 5: 100})
    """

    def __init__(
        self,
        api: Optional[DH5ModbusAPI] = None,
        *,
        port: str = "COM3",
        modbus_id: int = 1,
        baud_rate: int = 115200,
        stop_bits: int = 1,
        parity: str = "N",
        num_axes: int = reg.NUM_AXES,
        poll_interval: float = 0.5,
        byte_order: str = "high_low",
    ):
        self.api = api if api is not None else DH5ModbusAPI(
            port=port,
            modbus_id=modbus_id,
            baud_rate=baud_rate,
            stop_bits=stop_bits,
            parity=parity,
        )
        self.num_axes = num_axes
        self.poll_interval = poll_interval
        self.byte_order = byte_order

    # ------------------------------------------------------------------
    # Connection lifecycle
    # ------------------------------------------------------------------
    def connect(self) -> bool:
        status = self.api.open_connection()
        if status != self.api.SUCCESS:
            logger.error("Failed to open connection: %s", status)
            return False
        logger.info("Connection opened.")
        return True

    def close(self):
        return self.api.close_connection()

    def __enter__(self) -> "DH5Hand":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

    # ------------------------------------------------------------------
    # Faults
    # ------------------------------------------------------------------
    def current_faults(self):
        return self.api.get_cur_faults()

    def fault_history(self):
        return self.api.get_history_faults()

    def reset_faults(self) -> bool:
        result = self.api.reset_faults()
        if result == self.api.SUCCESS:
            logger.info("Faults reset successfully.")
            return True
        logger.warning("reset_faults() returned %s", format_faults(result))
        return False

    def clear_existing_faults(self) -> None:
        """Report any current fault and try to clear it. Best effort - a
        failure here is logged, not raised, because the caller usually wants
        to attempt initialization regardless."""
        faults = self.current_faults()
        logger.info("Current fault status: %s", format_faults(faults))
        if faults and faults != self.api.SUCCESS:
            logger.info("Faults detected - attempting reset_faults()...")
            self.reset_faults()

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------
    def initialize(
        self,
        mode: int = reg.INIT_MODE_OPEN,
        timeout: float = 20.0,
        poll_interval: Optional[float] = None,
        clear_faults: bool = True,
    ) -> bool:
        """Clear faults, send `initialize(mode)`, and poll until every axis
        reports 'initialized'. Returns True when the hand is ready.

        The connection must already be open - call `connect()` or use the
        class as a context manager first.
        """
        if not self.api.is_connected:
            logger.error("Cannot initialize: connection is not open.")
            return False

        if clear_faults:
            self.clear_existing_faults()

        logger.info("Sending initialize(mode=%s)...", bin(mode))
        status = self.api.initialize(mode)
        if status != self.api.SUCCESS:
            logger.error("Initialization command failed: %s", status)
            return False

        return self.wait_for_initialization(timeout=timeout, poll_interval=poll_interval)

    def wait_for_initialization(self, timeout: float = 20.0, poll_interval: Optional[float] = None) -> bool:
        poll_interval = self.poll_interval if poll_interval is None else poll_interval
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.api.check_initialization()
            logger.info("Initialization status: %s", status)
            if isinstance(status, dict) and all(state == "initialized" for state in status.values()):
                logger.info("All axes initialized successfully.")
                return True
            time.sleep(poll_interval)
        logger.error("Timed out after %.1fs waiting for all axes to initialize.", timeout)
        return False

    def check_initialization(self):
        return self.api.check_initialization()

    def initialize_axis(self, axis: int, mode: int):
        return self.api.initialize_axis(axis, mode)

    # ------------------------------------------------------------------
    # Reading: setpoints vs feedback, kept deliberately separate
    # ------------------------------------------------------------------
    def read_setpoint(self, kind: str, axis: int) -> int:
        """Read what was last COMMANDED for `kind` ('position', 'force',
        'speed') on `axis`. This is the value to use when filling in axes a
        batch write must leave unchanged."""
        self._validate_axis(axis)
        response = self.api.send_modbus_command(
            reg.READ_HOLDING_REGISTERS, reg.setpoint_register(kind, axis), data_length=1
        )
        return int(round(first_value(response)))

    def read_feedback(self, kind: str, axis: int) -> int:
        """Read what the axis is ACTUALLY doing for `kind` ('position',
        'speed', 'current'). Never write this back into a setpoint."""
        self._validate_axis(axis)
        response = self.api.send_modbus_command(
            reg.READ_HOLDING_REGISTERS, reg.feedback_register(kind, axis), data_length=1
        )
        return int(round(first_value(response)))

    def position_percent(self, axis: int) -> float:
        """Current measured position of `axis`, as a percentage of stroke."""
        return position_to_percent(self.read_feedback("position", axis), reg.AXIS_LIMITS[axis])

    def positions_percent(self) -> Dict[int, float]:
        return {axis: self.position_percent(axis) for axis in self.axes}

    def axis_status(self, axis: int) -> int:
        """0 = moving, 1 = reached position, 2 = stalled."""
        self._validate_axis(axis)
        response = self.api.send_modbus_command(
            reg.READ_HOLDING_REGISTERS, reg.status_register(axis), data_length=1
        )
        return first_value(response)

    def axis_status_label(self, axis: int) -> str:
        status = self.axis_status(axis)
        return reg.AXIS_STATUS_LABELS.get(status, str(status))

    @property
    def axes(self) -> List[int]:
        return list(range(1, self.num_axes + 1))

    def _validate_axis(self, axis: int) -> None:
        if axis not in reg.AXIS_LIMITS or not (1 <= axis <= self.num_axes):
            raise ValueError(f"Invalid axis {axis!r}; must be 1-{self.num_axes}.")

    # ------------------------------------------------------------------
    # Motion
    # ------------------------------------------------------------------
    def wait_for_axes(self, axes: Iterable[int], poll_interval: Optional[float] = None,
                      timeout: Optional[float] = 30.0) -> Dict[int, int]:
        """Poll `axes` until none of them still report 'moving'.

        Returns {axis: final_status}. Gives up after `timeout` seconds
        (pass None to wait forever) so a stalled axis that never leaves
        state 0 cannot hang the caller indefinitely.
        """
        poll_interval = self.poll_interval if poll_interval is None else poll_interval
        axes = list(axes)
        deadline = None if timeout is None else time.monotonic() + timeout

        while True:
            time.sleep(poll_interval)
            statuses = {axis: self.axis_status(axis) for axis in axes}
            moving = [axis for axis, status in statuses.items() if status == 0]
            if not moving:
                return statuses
            if deadline is not None and time.monotonic() > deadline:
                logger.warning("Timed out after %.1fs with axes still moving: %s", timeout, moving)
                return statuses
            logger.debug("...axes still moving: %s", moving)

    def move(
        self,
        axis_percents: Mapping[int, float],
        wait: bool = True,
        poll_interval: Optional[float] = None,
    ) -> MoveResult:
        """Move one or more axes to percent positions simultaneously.

        A single axis goes out as one `0x06` write; several axes go out as
        one `0x10` block write so they start together.
        """
        if not axis_percents:
            raise ValueError("At least one axis must be given.")
        for axis in axis_percents:
            self._validate_axis(axis)

        positions = {
            axis: percent_to_position(percent, reg.AXIS_LIMITS[axis])
            for axis, percent in axis_percents.items()
        }

        if len(positions) == 1:
            (axis, position), = positions.items()
            result = self.api.set_axis_position(axis, position)
        else:
            result = self._write_axis_block("position", positions)

        logger.info("move(%s) -> %s; positions=%s", dict(axis_percents), result, positions)

        statuses = None
        if wait and result == self.api.SUCCESS:
            statuses = self.wait_for_axes(positions, poll_interval)
            for axis in sorted(statuses):
                logger.info("Axis %d: %s", axis, reg.AXIS_STATUS_LABELS.get(statuses[axis], statuses[axis]))

        return MoveResult(result, positions, statuses)

    def move_axis(self, axis: int, percent: float, wait: bool = True,
                  poll_interval: Optional[float] = None) -> MoveResult:
        """Move a single axis. `wait` defaults to True, matching `move()` -
        with `wait=False`, two back-to-back calls overwrite each other's
        setpoint and the first target is never reached."""
        return self.move({axis: percent}, wait=wait, poll_interval=poll_interval)

    # ------------------------------------------------------------------
    # Speed and force
    # ------------------------------------------------------------------
    def set_speeds(self, axis_speeds: Mapping[int, int]):
        """Set speed on one or more axes. Axes not listed keep their
        current *setpoint* - not their measured speed, which would be 0 for
        anything standing still."""
        return self._set_axis_values("speed", axis_speeds)

    def set_speed(self, axis: int, speed: int):
        return self._set_axis_values("speed", {axis: speed})

    def set_forces(self, axis_forces: Mapping[int, int]):
        """Set force on one or more axes, leaving the rest at their current
        setpoint."""
        return self._set_axis_values("force", axis_forces)

    def set_force(self, axis: int, force: int):
        return self._set_axis_values("force", {axis: force})

    def _set_axis_values(self, kind: str, axis_values: Mapping[int, int]):
        if not axis_values:
            raise ValueError("At least one axis must be given.")
        for axis in axis_values:
            self._validate_axis(axis)

        values = {axis: int(round(value)) for axis, value in axis_values.items()}
        if len(values) == 1:
            (axis, value), = values.items()
            result = self.api.write_axis_setpoint(kind, axis, value)
        else:
            result = self._write_axis_block(kind, values)
        logger.info("set %s %s -> %s", kind, values, result)
        return result

    def _write_axis_block(self, kind: str, axis_values: Mapping[int, int]):
        """Write `axis_values` into the contiguous setpoint block for `kind`
        in one `0x10` frame, so every listed axis acts at the same instant.

        `0x10` cannot skip a register in the middle of its block, so axes
        the caller did not mention must still be given a value. That value
        is their current SETPOINT, read back from the same block - writing
        feedback here would quietly re-command every other axis.
        """
        full_block = self.api.read_block(reg.SETPOINT_BASE[kind], reg.NUM_AXES)
        if not isinstance(full_block, (list, tuple)) or len(full_block) != reg.NUM_AXES:
            # Fall back to one read per axis if the block read failed, so a
            # transient error doesn't turn into a write of garbage.
            logger.warning("Block read of %s setpoints returned %r; falling back to per-axis reads.",
                           kind, full_block)
            full_block = [self.read_setpoint(kind, axis) for axis in range(1, reg.NUM_AXES + 1)]

        values = [
            int(round(axis_values.get(axis, full_block[axis - 1])))
            for axis in range(1, reg.NUM_AXES + 1)
        ]
        return self.api.send_modbus_command(
            function_code=reg.WRITE_MULTIPLE_REGISTERS,
            register_address=reg.SETPOINT_BASE[kind],
            data=values,
            data_length=reg.NUM_AXES,
        )

    # ------------------------------------------------------------------
    # Fingertip sensors
    # ------------------------------------------------------------------
    def sensor_points_per_finger(self) -> int:
        """Read register 0x0222 - how many of each finger's 16 sensor point
        slots actually carry data. Identifies the sensor type: 2 or 16 =
        Saigan, 3 = Hualichuang."""
        response = self.api.send_modbus_command(
            reg.READ_HOLDING_REGISTERS, reg.SENSOR_POINTS_PER_FINGER_REGISTER, data_length=1
        )
        return int(first_value(response))

    def read_finger(self, finger: str, num_points: Optional[int] = None,
                    byte_order: Optional[str] = None) -> List[float]:
        """Read one finger's sensor points as floats, in a single frame."""
        base_register, num_points = self._sensor_block(finger, num_points)
        byte_order = byte_order or self.byte_order
        num_registers = num_points * 2

        response = self.api.send_modbus_command(
            reg.READ_HOLDING_REGISTERS, base_register, data_length=num_registers
        )
        if not isinstance(response, (list, tuple)) or len(response) != num_registers:
            raise ValueError(
                f"Expected {num_registers} register values for {finger!r} sensors, got: {response!r}"
            )
        return registers_to_floats(response, byte_order=byte_order)

    def read_finger_hualichuang(self, finger: str, byte_order: Optional[str] = None) -> Tuple[float, float]:
        """Tangential forces (fx, fy) for a Hualichuang-type finger. Only
        meaningful when `sensor_points_per_finger()` reports 3."""
        reading = self.read_finger_full(finger, byte_order=byte_order)
        return reading["fx"], reading["fy"]

    def read_finger_full(self, finger: str, byte_order: Optional[str] = None) -> Dict[str, float]:
        """A Hualichuang finger's three raw values plus the two derived
        ones: {'mx', 'my', 'fz', 'fx', 'fy'}. See
        `hualichuang_tangential_force` for a caveat about fy."""
        mx, my, fz = self.read_finger(finger, num_points=reg.HUALICHUANG_POINTS, byte_order=byte_order)
        fx, fy = hualichuang_tangential_force(mx, my, fz)
        return {"mx": mx, "my": my, "fz": fz, "fx": fx, "fy": fy}

    def read_all_sensors(self, num_points: Optional[int] = None,
                         byte_order: Optional[str] = None) -> Dict[str, List[float]]:
        return {
            finger: self.read_finger(finger, num_points=num_points, byte_order=byte_order)
            for finger in reg.FINGERS
        }

    def dump_finger_sensor_raw(self, finger: str, num_points: int = 3) -> None:
        """Print one finger's raw registers next to BOTH possible float
        conversions. Use this once to confirm which byte order your sensor
        uses, then leave `byte_order` alone."""
        base_register, num_points = self._sensor_block(finger, num_points)
        num_registers = num_points * 2

        response = self.api.send_modbus_command(
            reg.READ_HOLDING_REGISTERS, base_register, data_length=num_registers
        )
        if not isinstance(response, (list, tuple)) or len(response) != num_registers:
            raise ValueError(
                f"Expected {num_registers} register values for {finger!r} sensors, got: {response!r}"
            )

        logger.info("Raw sensor registers for %r (base 0x%04X):", finger, base_register)
        for i in range(0, num_registers, 2):
            reg_a, reg_b = response[i], response[i + 1]
            logger.info(
                "  point %d: reg[0x%04X]=0x%04X reg[0x%04X]=0x%04X  ->  low_high=%r  high_low=%r",
                i // 2 + 1, base_register + i, reg_a, base_register + i + 1, reg_b,
                registers_to_float(reg_a, reg_b, byte_order="low_high"),
                registers_to_float(reg_a, reg_b, byte_order="high_low"),
            )

    def _sensor_block(self, finger: str, num_points: Optional[int]) -> Tuple[int, int]:
        finger = finger.lower()
        if finger not in reg.FINGER_SENSOR_BASE_REGISTER:
            raise ValueError(f"Invalid finger {finger!r}. Must be one of {list(reg.FINGERS)}.")
        if num_points is None:
            num_points = reg.MAX_SENSOR_POINTS_PER_FINGER
        if not (1 <= num_points <= reg.MAX_SENSOR_POINTS_PER_FINGER):
            raise ValueError(f"num_points must be 1-{reg.MAX_SENSOR_POINTS_PER_FINGER}.")
        return reg.FINGER_SENSOR_BASE_REGISTER[finger], num_points

    def describe_finger(self, finger: str, num_points: int) -> str:
        """One human-readable line of sensor values for `finger`, labelled
        Mx/My/Fz/fx/fy for Hualichuang sensors and a plain list otherwise."""
        if num_points == reg.HUALICHUANG_POINTS:
            r = self.read_finger_full(finger)
            return (f"{finger}: Mx={r['mx']:.4f}  My={r['my']:.4f}  Fz={r['fz']:.4f}  "
                    f"fx={r['fx']:.4f}  fy={r['fy']:.4f}")
        values = self.read_finger(finger, num_points=num_points)
        return f"{finger}: {[round(v, 4) for v in values]}"

    def watch_sensors(self, fingers: Sequence[str], interval: float = 0.5,
                      num_points: Optional[int] = None) -> None:
        """Print sensor values for `fingers` every `interval` seconds until
        Ctrl+C. (Single-keypress detection differs between Windows and
        POSIX; Ctrl+C is the portable way to break a polling loop.)"""
        if num_points is None:
            num_points = self.sensor_points_per_finger()

        logger.info("Watching sensors every %.2fs. Press Ctrl+C to stop.", interval)
        try:
            while True:
                for finger in fingers:
                    logger.info("%s", self.describe_finger(finger, num_points))
                time.sleep(interval)
        except KeyboardInterrupt:
            print()
            logger.info("Stopped watching sensors.")

    # ------------------------------------------------------------------
    # Sensor calibration (register 0x0505)
    # ------------------------------------------------------------------
    def calibrate_sensors(self, timeout: float = 30.0, poll_interval: Optional[float] = None) -> bool:
        """Write 1 to 0x0505 and poll until it self-resets to 0, the
        manual's pattern for this whole family of action registers.

        All five fingertips must be free of contact for the entire
        duration. This does not check that - use
        `calibrate_sensors_interactive()` for a guided flow.
        """
        poll_interval = self.poll_interval if poll_interval is None else poll_interval
        logger.info("Writing 1 to 0x0505 to start sensor calibration...")
        self.api.send_modbus_command(
            reg.WRITE_SINGLE_REGISTER, reg.SENSOR_CALIBRATION_REGISTER, data=1, data_length=1
        )

        deadline = time.monotonic() + timeout
        started = time.monotonic()
        while time.monotonic() < deadline:
            time.sleep(poll_interval)
            value = first_value(self.api.send_modbus_command(
                reg.READ_HOLDING_REGISTERS, reg.SENSOR_CALIBRATION_REGISTER, data_length=1
            ))
            if value == 0:
                logger.info("Calibration complete after %.1fs.", time.monotonic() - started)
                return True
            logger.info("Still calibrating (reg 0x0505 = %s)...", value)

        logger.error("Timed out after %.1fs waiting for calibration to complete.", timeout)
        return False

    def calibrate_sensors_interactive(self, timeout: float = 10.0,
                                      poll_interval: Optional[float] = None,
                                      sanity_check: bool = True) -> bool:
        """Guided calibration: explain the contact-free requirement, wait
        for confirmation, calibrate, then check every finger really does
        read near zero."""
        print("=" * 60)
        print("DH5 FINGERTIP SENSOR CALIBRATION")
        print("=" * 60)
        print("Before continuing, make sure ALL FIVE FINGERS are completely")
        print("free of contact: nothing touching them, no objects resting")
        print("against them, no cables pulling on them, and the hand is not")
        print("mid-motion or vibrating.")
        print()
        print("This will take several seconds and must not be interrupted.")
        confirmation = input("Type 'y' when the sensors are clear, or anything else to abort: ").strip().lower()
        if confirmation != "y":
            logger.info("Calibration aborted.")
            return False

        if not self.calibrate_sensors(timeout=timeout, poll_interval=poll_interval):
            logger.error("Calibration did not confirm completion - do not trust sensor readings yet.")
            return False

        if not sanity_check:
            return True

        logger.info("Running post-calibration sanity check (reading all fingers)...")
        num_points = self.sensor_points_per_finger()
        flagged = []
        for finger in reg.FINGERS:
            if num_points == reg.HUALICHUANG_POINTS:
                reading = self.read_finger_full(finger)
                worst = max(abs(reading[k]) for k in ("mx", "my", "fz"))
                logger.info("  %s: Mx=%.4f  My=%.4f  Fz=%.4f",
                            finger, reading["mx"], reading["my"], reading["fz"])
            else:
                values = self.read_finger(finger, num_points=num_points)
                worst = max((abs(v) for v in values), default=0.0)
                logger.info("  %s: %s", finger, [round(v, 4) for v in values])
            if worst > SENSOR_ZERO_TOLERANCE:
                logger.warning("    ! %s still reads ~%.2f away from zero - check for contact.", finger, worst)
                flagged.append(finger)

        if flagged:
            logger.error("Still non-zero after calibration: %s. Clear them and re-run.", ", ".join(flagged))
            return False
        logger.info("All fingers read close to zero - calibration looks good.")
        return True

