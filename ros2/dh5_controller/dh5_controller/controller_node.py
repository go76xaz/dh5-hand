"""ROS2 node for the DH5 six-axis hand, built on the `dh5` library.

Everything hardware-related (Modbus, percent conversion, gestures) lives in
the `dh5` package. `ros2/dh5_controller/dh5` is a symlink to the real
`src/dh5`, and this package's setup.py lists it as one of its own packages,
so a plain `colcon build --symlink-install` makes `import dh5` work for this
node - no separate `pip install` of the library needed. `pyserial` is still
a real third-party dependency though: make sure `python3 -c "import serial"`
works in the environment you build/run with (apt install python3-serial, or
pip if you have it) before building.

This node only translates ROS interfaces into `DH5Hand` calls and back:

    topics    dh5/AxisInfos      axis state (raw + percent position, status, ...)
              dh5/joint_states   sensor_msgs/JointState, position in percent
              dh5/fingertips     fingertip sensors (mx, my, fz, stale flag)
    action    dh5/move_axes      non-blocking move with feedback, cancel and
                                 optional stop on fingertip contact
    services  dh5/stop, dh5/calibrate_sensors, the move/speed/force/fault
              services, dh5/raw/*, and one service per gesture

Skill services are generated from `dh5.gestures.GESTURES`, so a new entry in
that table becomes a new `dh5/<name>` service with no change here, unless
its `ros_service` flag is False (terminal/CLI-only gestures):

    * gestures without a width  -> std_srvs/Trigger
    * gestures with a width     -> dh5_interfaces/TwoFingerPinch
      (`width`, plus `axis_mode` selecting the gesture variant)

Threading: `DH5ModbusAPI` locks every Modbus exchange itself, so the
publishers can keep reading while a multi-second gesture is running. What
must not overlap is two *commands* (a second gesture would fight the first
over the setpoints), so commands - services and action goals alike - take
`_motion_lock`, and one that arrives while another is running is refused.
`dh5/stop` is the exception: it never waits for the lock, because it has to
work exactly while something else is running.
"""

import logging
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from dh5_interfaces.action import MoveAxes
from dh5_interfaces.msg import AxisInfos, Fingertip, FingertipArray
from dh5_interfaces.srv import (
    Initialize,
    MoveAxesPercent,
    SetAxesValues,
    SetAxisValue,
    SetValues,
    TwoFingerPinch,
)

from dh5 import DH5Hand, format_faults, gestures, registers
from dh5.sensors import ContactDetector, StaleTracker

JOINT_NAMES = [
    '01:Thumb_Yaw',
    '02:Index',
    '03:Middle',
    '04:Ring',
    '05:Pinky',
    '06:Thumb_Pitch',
]

# Initialization and sensor calibration log every status poll, and take
# seconds anyway - no point polling them at the fast move rate.
SLOW_POLL_INTERVAL = 0.5


class _RosLogHandler(logging.Handler):
    """Forward the `dh5.*` library log records into the node's ROS logger."""

    def __init__(self, ros_logger):
        super().__init__(logging.INFO)
        self._ros_logger = ros_logger

    def emit(self, record):
        message = record.getMessage()
        if record.levelno >= logging.ERROR:
            self._ros_logger.error(message)
        elif record.levelno >= logging.WARNING:
            self._ros_logger.warning(message)
        else:
            self._ros_logger.info(message)


class _RateProbe:
    """Logs once how long a periodic bus read takes on average - the real
    ceiling for that timer's rate, whatever rate was requested."""

    def __init__(self, logger, name, requested_hz, samples=50):
        self._logger = logger
        self._name = name
        self._requested_hz = requested_hz
        self._samples = samples
        self._durations = []

    def record(self, seconds):
        if len(self._durations) >= self._samples:
            return
        self._durations.append(seconds)
        if len(self._durations) < self._samples:
            return
        average = sum(self._durations) / len(self._durations)
        text = (f'{self._name}: one read cycle takes {average * 1000:.1f} ms on average '
                f'(~{1 / average:.0f} Hz possible on its own; {self._requested_hz:g} Hz requested).')
        if average > 1 / self._requested_hz:
            self._logger.warning(text + ' The requested rate cannot be reached.')
        else:
            self._logger.info(text)


class DH5Controller(Node):
    def __init__(self):
        super().__init__('dh5_controller_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('modbus_id', 1)
        self.declare_parameter('baud_rate', 115200)
        state_rate_hz = self._declare_number('state_rate_hz', 10.0)
        sensor_rate_hz = self._declare_number('sensor_rate_hz', 10.0)
        self.stale_timeout = self._declare_number('stale_timeout', 1.0)
        self.move_poll_interval = self._declare_number('move_poll_interval', 0.05)
        self.contact_threshold = self._declare_number('contact_threshold', 0.5)
        self.move_timeout = self._declare_number('move_timeout', 30.0)
        # Deprecated: seconds between state messages, replaced by state_rate_hz.
        publish_period = self._declare_number('publish_period', -1.0)
        if publish_period > 0:
            state_rate_hz = 1.0 / publish_period
            self.get_logger().warning(
                f'publish_period is deprecated; use state_rate_hz (now {state_rate_hz:g} Hz).')
        for name, value in (('state_rate_hz', state_rate_hz), ('sensor_rate_hz', sensor_rate_hz)):
            if value <= 0:
                raise ValueError(f'{name} must be positive, got {value}.')

        # Route the library's logging through ROS so it shows up in the
        # node's output like everything else.
        dh5_logger = logging.getLogger('dh5')
        dh5_logger.addHandler(_RosLogHandler(self.get_logger()))
        dh5_logger.setLevel(logging.INFO)

        port = self.get_parameter('port').value
        self.hand = DH5Hand(
            port=port,
            modbus_id=self.get_parameter('modbus_id').value,
            baud_rate=self.get_parameter('baud_rate').value,
            poll_interval=self.move_poll_interval,
        )
        if not self.hand.connect():
            raise RuntimeError(f'Could not open the DH5 on {port} (check the port parameter).')
        self.sensors_available = self._check_sensor_type()

        self._motion_lock = threading.Lock()
        self.hw_group = ReentrantCallbackGroup()

        self.pub_axis_infos = self.create_publisher(AxisInfos, 'dh5/AxisInfos', 10)
        self.pub_joint_states = self.create_publisher(JointState, 'dh5/joint_states', 10)

        # --- Raw / low-level services -------------------------------------
        # Namespaced under dh5/raw/ so they don't shadow the intended,
        # percent-/axis-based services below in `ros2 service list` or tab
        # completion. These write raw register units with no percent
        # conversion and require a value for every axis - prefer
        # move_axis(es)_percent / set_axis(es)_speed / set_axis_force unless
        # you specifically need to bypass that.
        for name, handler in (
            ('set_position', self.set_position_cb),
            ('set_speed', self.set_speed_cb),
            ('set_force', self.set_force_cb),
        ):
            self.create_service(SetValues, f'dh5/raw/{name}', handler, callback_group=self.hw_group)

        for name, handler in (
            ('set_axis_position', self.set_axis_position_cb),
            ('set_axis_speed', self.set_axis_speed_cb),
            ('set_axis_force', self.set_axis_force_cb),
            ('move_axis_percent', self.move_axis_percent_cb),
        ):
            self.create_service(SetAxisValue, f'dh5/{name}', handler, callback_group=self.hw_group)

        self.create_service(MoveAxesPercent, 'dh5/move_axes_percent', self.move_axes_percent_cb,
                            callback_group=self.hw_group)
        self.create_service(SetAxesValues, 'dh5/set_axes_speed', self.set_axes_speed_cb,
                            callback_group=self.hw_group)
        self.create_service(Initialize, 'dh5/initialize', self.initialize_cb,
                            callback_group=self.hw_group)

        for name, handler in (
            ('clear_cur_fault', self.clear_cur_fault_cb),
            ('clear_history_faults', self.clear_history_faults_cb),
            ('restart_system', self.restart_system_cb),
            ('get_faults', self.get_faults_cb),
            ('stop', self.stop_cb),
            ('calibrate_sensors', self.calibrate_sensors_cb),
        ):
            self.create_service(Trigger, f'dh5/{name}', handler, callback_group=self.hw_group)

        # --- Non-blocking motion ------------------------------------------
        self._move_axes_server = ActionServer(
            self,
            MoveAxes,
            'dh5/move_axes',
            execute_callback=self.execute_move_axes,
            goal_callback=self.move_axes_goal_cb,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=self.hw_group,
        )

        # --- Skills, one service per entry in the gesture table ----------
        for gesture in gestures.GESTURES.values():
            if not gesture.ros_service:
                continue
            srv_type = TwoFingerPinch if gesture.is_scalable else Trigger
            self.create_service(srv_type, f'dh5/{gesture.name}',
                                self._make_gesture_cb(gesture), callback_group=self.hw_group)
            self.get_logger().info(f'Skill service: dh5/{gesture.name}')

        # --- Periodic publishers ------------------------------------------
        # Each timer gets its own mutually exclusive group, so a slow bus
        # cycle delays that timer instead of piling up overlapping copies.
        self._state_probe = _RateProbe(self.get_logger(), 'dh5/AxisInfos', state_rate_hz)
        self.state_timer = self.create_timer(
            1.0 / state_rate_hz, self.publish_state,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self._stale = StaleTracker(self.stale_timeout)
        self._stale_fingers = set()
        if self.sensors_available:
            self.pub_fingertips = self.create_publisher(FingertipArray, 'dh5/fingertips', 10)
            self._sensor_probe = _RateProbe(self.get_logger(), 'dh5/fingertips', sensor_rate_hz)
            self.sensor_timer = self.create_timer(
                1.0 / sensor_rate_hz, self.publish_fingertips,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

        self.get_logger().info('DH5 Controller Node started.')

    # ----------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------
    def _declare_number(self, name, default):
        """Declare a numeric parameter that accepts ints and floats alike,
        so `state_rate_hz:=20` works as well as `state_rate_hz:=20.0`."""
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        return float(self.get_parameter(name).value)

    def _check_sensor_type(self):
        try:
            points = self.hand.sensor_points_per_finger()
        except Exception as exc:
            self.get_logger().warning(
                f'Could not read the fingertip sensor type ({exc}); dh5/fingertips and '
                'stop_on_contact are disabled.')
            return False
        if points != registers.HUALICHUANG_POINTS:
            self.get_logger().warning(
                f'The hand reports {points} sensor points per finger; dh5/fingertips and '
                'stop_on_contact need the 3-axis (mx, my, fz) sensors and are disabled.')
            return False
        return True

    def _run(self, response, action, ok_message='OK'):
        """Run `action()` as the only motion command in flight and fill in
        `response`.

        `action` returns a Modbus result (`SUCCESS` = ok, anything else is
        an error code/message), a bool, or a `(success, message)` tuple. It
        may raise `ValueError` for bad arguments.
        """
        if not self._motion_lock.acquire(blocking=False):
            response.success = False
            response.message = 'Busy: another command is still running.'
            return response
        try:
            # A dh5/stop aimed at an earlier command must not cancel this one.
            self.hand.clear_stop()
            outcome = action()
        except ValueError as exc:
            response.success = False
            response.message = str(exc)
        except Exception as exc:  # hardware/serial trouble must not kill the node
            self.get_logger().error(f'Command failed: {exc}')
            response.success = False
            response.message = f'{type(exc).__name__}: {exc}'
        else:
            response.success, response.message = self._interpret(outcome, ok_message)
        finally:
            self._motion_lock.release()
        return response

    def _interpret(self, outcome, ok_message):
        if isinstance(outcome, tuple):
            return outcome
        if isinstance(outcome, bool):
            return outcome, ok_message if outcome else 'Failed (see node log).'
        if outcome == self.hand.api.SUCCESS:
            return True, ok_message
        return False, f'Device/bus returned {outcome!r}'

    # ----------------------------------------------------------------
    # Raw / low-level callbacks
    # ----------------------------------------------------------------
    def set_position_cb(self, request, response):
        return self._run(response, lambda: self.hand.api.set_position(list(request.value)))

    def set_speed_cb(self, request, response):
        return self._run(response, lambda: self.hand.api.set_speed(list(request.value)))

    def set_force_cb(self, request, response):
        return self._run(response, lambda: self.hand.api.set_force(list(request.value)))

    def set_axis_position_cb(self, request, response):
        return self._run(response, lambda: self.hand.api.set_axis_position(request.axis, request.value))

    def set_axis_speed_cb(self, request, response):
        return self._run(response, lambda: self.hand.set_speed(request.axis, request.value))

    def set_axes_speed_cb(self, request, response):
        if len(request.axes) != len(request.values):
            response.success = False
            response.message = "'axes' and 'values' must be the same length."
            return response
        axis_speeds = dict(zip(request.axes, request.values))
        return self._run(response, lambda: self.hand.set_speeds(axis_speeds))

    def set_axis_force_cb(self, request, response):
        return self._run(response, lambda: self.hand.set_force(request.axis, request.value))

    def initialize_cb(self, request, response):
        def action():
            if request.mode not in registers.INIT_MODES:
                raise ValueError(f'mode must be one of {list(registers.INIT_MODES)}, got {request.mode}.')
            # Blocks until every axis reports ready, so success means the
            # hand really is initialized, not just that the command was sent.
            return self.hand.initialize(request.mode, poll_interval=SLOW_POLL_INTERVAL)
        return self._run(response, action, ok_message='Initialized.')

    def clear_cur_fault_cb(self, request, response):
        return self._run(response, self.hand.reset_faults, ok_message='Faults cleared.')

    def clear_history_faults_cb(self, request, response):
        return self._run(response, self.hand.api.reset_history_faults)

    def restart_system_cb(self, request, response):
        return self._run(response, self.hand.api.restart_system)

    def get_faults_cb(self, request, response):
        current = self.hand.current_faults()
        history = self.hand.fault_history()
        response.success = isinstance(current, list)
        response.message = f'current={format_faults(current)} history={format_faults(history)}'
        return response

    # ----------------------------------------------------------------
    # Stop and sensor calibration
    # ----------------------------------------------------------------
    def stop_cb(self, request, response):
        # Deliberately not through _run: stop must work while another
        # command holds the motion lock. That command sees the stop flag
        # and finishes with a "stopped" failure.
        try:
            result = self.hand.stop()
        except Exception as exc:
            self.get_logger().error(f'Stop failed: {exc}')
            response.success = False
            response.message = f'{type(exc).__name__}: {exc}'
            return response
        response.success = result == self.hand.api.SUCCESS
        response.message = ('All axes stopped.' if response.success
                            else f'Stop flagged, but holding the axes failed: {result!r}')
        return response

    def calibrate_sensors_cb(self, request, response):
        def action():
            state = self.hand.read_state()
            moving = [axis for axis, status in enumerate(state.status, start=1) if status == 0]
            if moving:
                return False, (f'Axes {moving} are moving; calibrate with the hand at rest and '
                               'nothing touching the fingertips.')
            if not self.hand.calibrate_sensors(poll_interval=SLOW_POLL_INTERVAL):
                return False, 'Calibration did not confirm completion (register 0x0505 never reset).'
            flagged = self.hand.check_sensors_zeroed()
            if flagged:
                return False, (f'Calibrated, but fingers {flagged} still read away from zero - '
                               'check for contact and re-run.')
            return True, 'Sensors calibrated; all fingers read near zero.'
        return self._run(response, action)

    # ----------------------------------------------------------------
    # Percent-based moves
    # ----------------------------------------------------------------
    def _move(self, axis_percents):
        """Move and turn the result into (success, message)."""
        move = self.hand.move(axis_percents, wait=True)
        if move.result != self.hand.api.SUCCESS:
            return False, f'Move rejected: device/bus returned {move.result!r}'
        if self.hand.stop_requested:
            return False, 'Stopped by dh5/stop.'
        return True, self._describe_statuses(f'Moved {dict(axis_percents)}', move.statuses)

    @staticmethod
    def _describe_statuses(prefix, statuses):
        stalled = sorted(axis for axis, status in (statuses or {}).items() if status == 2)
        moving = sorted(axis for axis, status in (statuses or {}).items() if status == 0)
        notes = []
        if stalled:
            notes.append(f'stalled axes: {stalled}')
        if moving:
            notes.append(f'still moving after timeout: {moving}')
        return prefix + (f' ({"; ".join(notes)})' if notes else '')

    def move_axis_percent_cb(self, request, response):
        return self._run(response, lambda: self._move({request.axis: request.value}))

    def move_axes_percent_cb(self, request, response):
        if len(request.axes) != len(request.percents):
            response.success = False
            response.message = "'axes' and 'percents' must be the same length."
            return response
        axis_percents = dict(zip(request.axes, request.percents))
        return self._run(response, lambda: self._move(axis_percents))

    # ----------------------------------------------------------------
    # dh5/move_axes action
    # ----------------------------------------------------------------
    def move_axes_goal_cb(self, goal_request):
        if self._motion_lock.locked():
            self.get_logger().warning('dh5/move_axes goal rejected: another command is still running.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_move_axes(self, goal_handle):
        result = MoveAxes.Result()
        if not self._motion_lock.acquire(blocking=False):
            # Another command slipped in between accepting and executing.
            result.success = False
            result.message = 'Busy: another command is still running.'
            goal_handle.abort()
            return result
        try:
            self.hand.clear_stop()
            return self._execute_move(goal_handle, result)
        except ValueError as exc:
            result.success = False
            result.message = str(exc)
        except Exception as exc:  # hardware/serial trouble must not kill the node
            self.get_logger().error(f'dh5/move_axes failed: {exc}')
            result.success = False
            result.message = f'{type(exc).__name__}: {exc}'
        finally:
            self._motion_lock.release()
        goal_handle.abort()
        return result

    def _validate_move_goal(self, goal):
        axes = list(goal.axes)
        if not axes:
            raise ValueError('No axes given.')
        if len(set(axes)) != len(axes):
            raise ValueError(f'Each axis may appear only once, got {axes}.')
        for axis in axes:
            if axis not in registers.AXIS_LIMITS:
                raise ValueError(f'Invalid axis {axis}; must be 1-{registers.NUM_AXES}.')
        if len(goal.percents) != len(axes):
            raise ValueError("'axes' and 'percents' must be the same length.")
        for name in ('speeds', 'forces'):
            if len(getattr(goal, name)) not in (0, len(axes)):
                raise ValueError(f"'{name}' must be empty or the same length as 'axes'.")
        return axes

    def _execute_move(self, goal_handle, result):
        goal = goal_handle.request
        axes = self._validate_move_goal(goal)
        timeout = goal.timeout if goal.timeout > 0 else self.move_timeout

        detector = None
        stale = None
        if goal.stop_on_contact:
            if not self.sensors_available:
                raise ValueError('stop_on_contact needs the 3-axis fingertip sensors, '
                                 'which are not available on this hand.')
            threshold = goal.contact_threshold if goal.contact_threshold > 0 else self.contact_threshold
            fingers = sorted({registers.AXIS_FINGER[axis] for axis in axes})
            baseline = self.hand.read_fingertips()
            missing = [finger for finger in fingers if baseline.get(finger) is None]
            if missing:
                raise ValueError(f'Could not read fingertips {missing} for the contact baseline.')
            detector = ContactDetector({finger: baseline[finger] for finger in fingers}, threshold)
            stale = StaleTracker(self.stale_timeout)

        for kind, values, setter in (('speed', goal.speeds, self.hand.set_speeds),
                                     ('force', goal.forces, self.hand.set_forces)):
            if values:
                outcome = setter(dict(zip(axes, values)))
                if outcome != self.hand.api.SUCCESS:
                    raise RuntimeError(f'Setting {kind} failed: device/bus returned {outcome!r}')

        move = self.hand.move(dict(zip(axes, goal.percents)), wait=False)
        if move.result != self.hand.api.SUCCESS:
            raise RuntimeError(f'Move rejected: device/bus returned {move.result!r}')

        targets = dict(move.positions)
        contact = set()
        stale_fingers = set()
        state = None
        started = time.monotonic()

        while True:
            time.sleep(self.move_poll_interval)
            elapsed = time.monotonic() - started

            if goal_handle.is_cancel_requested:
                self.hand.hold_axes(axes)
                outcome = 'canceled'
                break
            if self.hand.stop_requested:
                outcome = 'stopped'
                break

            try:
                state = self.hand.read_state()
            except ValueError as exc:
                self.get_logger().warning(f'State read failed during dh5/move_axes: {exc}',
                                          throttle_duration_sec=1.0)
            else:
                if detector is not None:
                    newly = self._detect_contact(axes, state, targets, elapsed, detector, stale,
                                                 contact, stale_fingers)
                    if newly:
                        self.hand.hold_axes(newly)
                        contact.update(newly)
                        for axis in newly:
                            # Held short of the target on purpose; from now
                            # on only its status says when it is done.
                            targets.pop(axis, None)

                settled = [
                    self.hand.is_settled(axis, state.status[axis - 1], state.position[axis - 1],
                                         targets.get(axis), elapsed)
                    for axis in axes
                ]
                feedback = MoveAxes.Feedback()
                feedback.percents = [state.position_percent[axis - 1] for axis in axes]
                feedback.status = [state.status[axis - 1] & 0xFF for axis in axes]
                feedback.contact = [axis in contact for axis in axes]
                goal_handle.publish_feedback(feedback)

                if all(settled):
                    outcome = 'done'
                    break

            if elapsed > timeout:
                outcome = 'timeout'
                break

        if state is None:
            state = self.hand.read_state()
        still_moving = [axis for axis in axes if state.status[axis - 1] == 0]
        if outcome == 'timeout' and still_moving:
            self.hand.hold_axes(still_moving)

        result.final_percents = [state.position_percent[axis - 1] for axis in axes]
        result.final_status = [state.status[axis - 1] & 0xFF for axis in axes]
        result.contact_axes = sorted(contact)
        result.stalled_axes = [axis for axis in axes if state.status[axis - 1] == 2]

        notes = []
        if result.contact_axes:
            notes.append(f'contact on axes {result.contact_axes}')
        if result.stalled_axes:
            notes.append(f'stalled axes {result.stalled_axes}')
        if stale_fingers:
            notes.append(f'fingertips {sorted(stale_fingers)} went stale, contact could not be '
                         'detected on them')
        suffix = f' ({"; ".join(notes)})' if notes else ''

        if outcome == 'done':
            result.success = True
            result.message = f'Moved {dict(zip(axes, goal.percents))}{suffix}'
            goal_handle.succeed()
        elif outcome == 'canceled':
            result.success = False
            result.message = f'Canceled; axes held where they were{suffix}'
            goal_handle.canceled()
        elif outcome == 'stopped':
            result.success = False
            result.message = f'Stopped by dh5/stop{suffix}'
            goal_handle.abort()
        else:
            result.success = False
            result.message = (f'Timed out after {timeout:g}s; axes {still_moving} were still moving '
                              f'and have been held{suffix}')
            goal_handle.abort()
        return result

    def _detect_contact(self, axes, state, targets, elapsed, detector, stale, contact, stale_fingers):
        """Axes that just touched something: still travelling, not already
        stopped by contact, and their fingertip moved past the threshold."""
        readings = self.hand.read_fingertips()
        finger_stale = {finger: stale.update(finger, readings.get(finger)) for finger in detector.baseline}
        stale_fingers.update(finger for finger, is_stale in finger_stale.items() if is_stale)

        newly = []
        for axis in axes:
            if axis in contact:
                continue
            if self.hand.is_settled(axis, state.status[axis - 1], state.position[axis - 1],
                                    targets.get(axis), elapsed):
                continue
            finger = registers.AXIS_FINGER[axis]
            if finger_stale[finger]:
                continue
            if detector.in_contact(finger, readings[finger]):
                newly.append(axis)
        return newly

    # ----------------------------------------------------------------
    # Skills, generated from the gesture table
    # ----------------------------------------------------------------
    def _make_gesture_cb(self, gesture):
        def callback(request, response):
            width = float(request.width) if gesture.is_scalable else 0.0
            variant = (request.axis_mode or None) if gesture.is_scalable else None

            def action():
                results = gestures.perform(self.hand, gesture.name, width=width, variant=variant)
                if results is None:
                    return False, 'Pre-flight check failed (see node log for details).'
                for move in results:
                    if move.result != self.hand.api.SUCCESS:
                        return False, f'Move rejected: device/bus returned {move.result!r}'
                if self.hand.stop_requested:
                    return False, f'{gesture.name}() stopped by dh5/stop.'
                statuses = results[-1].statuses if results else None
                return True, self._describe_statuses(f'{gesture.name}() completed', statuses)

            return self._run(response, action)

        return callback

    # ----------------------------------------------------------------
    # Publishing
    # ----------------------------------------------------------------
    def publish_state(self):
        started = time.monotonic()
        try:
            state = self.hand.read_state()  # one Modbus frame when the hand allows it
        except Exception as exc:
            self.get_logger().warning(f'Axis state read failed: {exc}', throttle_duration_sec=5.0)
            return
        self._state_probe.record(time.monotonic() - started)
        stamp = self.get_clock().now().to_msg()

        msg = AxisInfos()
        msg.name = JOINT_NAMES
        msg.header.stamp = stamp
        msg.position = state.position
        msg.velocity = state.speed
        msg.currents = state.current
        msg.cur_faults = [state.fault]
        msg.position_percent = state.position_percent
        msg.status = [status & 0xFF for status in state.status]
        self.pub_axis_infos.publish(msg)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = JOINT_NAMES
        joints.position = state.position_percent  # percent of stroke, not radians
        self.pub_joint_states.publish(joints)

    def publish_fingertips(self):
        started = time.monotonic()
        readings = self.hand.read_fingertips()
        self._sensor_probe.record(time.monotonic() - started)

        msg = FingertipArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        fingers = []
        for finger in registers.FINGERS:
            reading = readings.get(finger)
            is_stale = self._stale.update(finger, reading)
            tip = Fingertip()
            tip.id = finger
            tip.mx, tip.my, tip.fz = reading if reading is not None else (math.nan,) * 3
            tip.stale = is_stale
            fingers.append(tip)

            if is_stale and finger not in self._stale_fingers:
                self._stale_fingers.add(finger)
                self.get_logger().warning(
                    f'Fingertip {finger} is stale: no new data for {self.stale_timeout:g}s or the read failed.')
            elif not is_stale and finger in self._stale_fingers:
                self._stale_fingers.discard(finger)
                self.get_logger().info(f'Fingertip {finger} is updating again.')
        msg.fingers = fingers
        self.pub_fingertips.publish(msg)

    def destroy_node(self):
        self.hand.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DH5Controller()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
