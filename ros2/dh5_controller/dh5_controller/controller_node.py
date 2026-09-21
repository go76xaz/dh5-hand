"""ROS2 node for the DH5 six-axis hand, built on the `dh5` library.

Everything hardware-related (Modbus, percent conversion, gestures) lives in
the `dh5` package - install it into the ROS Python environment once:

    pip install -e "<path>/DH5 SDK/DH5_venv"

This node only translates ROS services into `DH5Hand` calls and back.

Skill services are generated from `dh5.gestures.GESTURES`, so a new entry in
that table becomes a new `dh5/<name>` service with no change here:

    * gestures without a width  -> std_srvs/Trigger
    * gestures with a width     -> dh5_interfaces/TwoFingerPinch
      (`width`, plus `axis_mode` selecting the gesture variant)

Threading: `DH5ModbusAPI` locks every Modbus exchange itself, so the state
publisher can keep reading while a multi-second gesture is running. What
must not overlap is two *commands* (a second gesture would fight the first
over the setpoints), so commands take `_motion_lock` and a command that
arrives while another is running is refused with a "busy" response.
"""

import logging
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_srvs.srv import Trigger
from dh5_interfaces.msg import AxisInfos
from dh5_interfaces.srv import (
    Initialize,
    MoveAxesPercent,
    SetAxisValue,
    SetValues,
    TwoFingerPinch,
)

from dh5 import DH5Hand, format_faults, gestures, registers

JOINT_NAMES = [
    '01:Thumb_Yaw',
    '02:Index',
    '03:Middle',
    '04:Ring',
    '05:Pinky',
    '06:Thumb_Pitch',
]


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


def _signed16(value: int) -> int:
    """Modbus registers arrive unsigned; velocity and current are signed."""
    value &= 0xFFFF
    return value - 0x10000 if value & 0x8000 else value


class DH5Controller(Node):
    def __init__(self):
        super().__init__('dh5_controller_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('modbus_id', 1)
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('publish_period', 0.5)  # seconds

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
        )
        if not self.hand.connect():
            raise RuntimeError(f'Could not open the DH5 on {port} (check the port parameter).')

        self._motion_lock = threading.Lock()
        self.hw_group = ReentrantCallbackGroup()

        self.pub_joint_state = self.create_publisher(AxisInfos, 'dh5/AxisInfos', 10)

        # --- Raw / low-level services -----------------------------------
        for name, handler in (
            ('set_position', self.set_position_cb),
            ('set_speed', self.set_speed_cb),
            ('set_force', self.set_force_cb),
        ):
            self.create_service(SetValues, f'dh5/{name}', handler, callback_group=self.hw_group)

        for name, handler in (
            ('set_axis_position', self.set_axis_position_cb),
            ('set_axis_speed', self.set_axis_speed_cb),
            ('set_axis_force', self.set_axis_force_cb),
            ('move_axis_percent', self.move_axis_percent_cb),
        ):
            self.create_service(SetAxisValue, f'dh5/{name}', handler, callback_group=self.hw_group)

        self.create_service(MoveAxesPercent, 'dh5/move_axes_percent', self.move_axes_percent_cb,
                            callback_group=self.hw_group)
        self.create_service(Initialize, 'dh5/initialize', self.initialize_cb,
                            callback_group=self.hw_group)

        for name, handler in (
            ('clear_cur_fault', self.clear_cur_fault_cb),
            ('clear_history_faults', self.clear_history_faults_cb),
            ('restart_system', self.restart_system_cb),
            ('get_faults', self.get_faults_cb),
        ):
            self.create_service(Trigger, f'dh5/{name}', handler, callback_group=self.hw_group)

        # --- Skills, one service per entry in the gesture table ----------
        for gesture in gestures.GESTURES.values():
            srv_type = TwoFingerPinch if gesture.is_scalable else Trigger
            self.create_service(srv_type, f'dh5/{gesture.name}',
                                self._make_gesture_cb(gesture), callback_group=self.hw_group)
            self.get_logger().info(f'Skill service: dh5/{gesture.name}')

        self.timer = self.create_timer(
            self.get_parameter('publish_period').value,
            self.publish_joint_states,
            callback_group=self.hw_group,
        )
        self.get_logger().info('DH5 Controller Node started.')

    # ----------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------
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

    def set_axis_force_cb(self, request, response):
        return self._run(response, lambda: self.hand.set_force(request.axis, request.value))

    def initialize_cb(self, request, response):
        def action():
            if request.mode not in registers.INIT_MODES:
                raise ValueError(f'mode must be one of {list(registers.INIT_MODES)}, got {request.mode}.')
            # Blocks until every axis reports ready, so success means the
            # hand really is initialized, not just that the command was sent.
            return self.hand.initialize(request.mode)
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
    # Percent-based moves
    # ----------------------------------------------------------------
    def _move(self, axis_percents):
        """Move and turn the result into (success, message)."""
        move = self.hand.move(axis_percents, wait=True)
        if move.result != self.hand.api.SUCCESS:
            return False, f'Move rejected: device/bus returned {move.result!r}'
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
                statuses = results[-1].statuses if results else None
                return True, self._describe_statuses(f'{gesture.name}() completed', statuses)

            return self._run(response, action)

        return callback

    # ----------------------------------------------------------------
    # State publishing
    # ----------------------------------------------------------------
    def publish_joint_states(self):
        api = self.hand.api  # every read is one locked Modbus exchange
        positions = api.get_position_fd()
        velocities = api.get_speed_fd()
        currents = api.get_current_fd()
        faults = api.get_cur_faults()

        msg = AxisInfos()
        msg.name = JOINT_NAMES
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = [v & 0xFFFF for v in positions] if isinstance(positions, list) else []
        msg.velocity = [_signed16(v) for v in velocities] if isinstance(velocities, list) else []
        msg.currents = [_signed16(v) for v in currents] if isinstance(currents, list) else []
        msg.cur_faults = [faults[0]] if isinstance(faults, list) and faults else []

        if not (msg.position and msg.velocity and msg.currents):
            self.get_logger().warning(
                'Incomplete feedback read (position=%r velocity=%r current=%r)'
                % (positions, velocities, currents),
                throttle_duration_sec=5.0,
            )
        self.pub_joint_state.publish(msg)

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
