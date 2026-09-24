"""The ROS2 node's logic, run against the fake DH5 with stub `rclpy`.

ROS is not installed on the development machine, so the ROS modules are
replaced with minimal stand-ins. What is under test is the node's own logic:
that services and the move action turn library results into honest
success/message responses, that the skill services follow the gesture
table, and what the publishers put into their messages.
"""

import importlib.util
import math
import sys
import threading
import types
from pathlib import Path
from types import SimpleNamespace

import pytest

from dh5 import gestures, registers as reg
from dh5.sensors import StaleTracker

NODE_FILE = (
    Path(__file__).resolve().parents[1]
    / "ros2" / "dh5_controller" / "dh5_controller" / "controller_node.py"
)

pytestmark = pytest.mark.skipif(not NODE_FILE.exists(), reason="ROS2 package not in this checkout")


class _Msg:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)


class _StubNode:
    """Just enough of rclpy.node.Node for DH5Controller to construct."""

    def __init__(self, name):
        self.params, self.services, self.published, self.actions = {}, {}, {}, {}

    def declare_parameter(self, name, default, descriptor=None):
        self.params[name] = SimpleNamespace(value=default)

    def get_parameter(self, name):
        return self.params[name]

    def get_logger(self):
        log = SimpleNamespace()
        for level in ("info", "warning", "error"):
            setattr(log, level, lambda *a, **k: None)
        return log

    def create_publisher(self, msg_type, topic, qos):
        messages = self.published.setdefault(topic, [])
        return SimpleNamespace(publish=messages.append)

    def create_service(self, srv_type, name, callback, callback_group=None):
        self.services[name] = (srv_type, callback)

    def create_timer(self, *a, **k):
        return None

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))

    def destroy_node(self):
        pass


class _StubActionServer:
    def __init__(self, node, action_type, name, **callbacks):
        node.actions[name] = callbacks


class FakeGoalHandle:
    """Stands in for rclpy's ServerGoalHandle. `on_feedback(n)` runs after
    the n-th feedback message, which lets a test change the fake hand while
    the action is in the middle of its loop."""

    def __init__(self, on_feedback=None, **goal):
        fields = dict(speeds=[], forces=[], stop_on_contact=False, contact_threshold=0.0, timeout=0.0)
        fields.update(goal)
        self.request = SimpleNamespace(**fields)
        self.feedback = []
        self.state = None
        self.cancel = False
        self._on_feedback = on_feedback

    @property
    def is_cancel_requested(self):
        return self.cancel

    def publish_feedback(self, feedback):
        self.feedback.append(feedback)
        if self._on_feedback:
            self._on_feedback(len(self.feedback))

    def succeed(self):
        self.state = "succeeded"

    def abort(self):
        self.state = "aborted"

    def canceled(self):
        self.state = "canceled"


@pytest.fixture
def node_module(monkeypatch):
    def module(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, mod)
        return mod

    srv = lambda name: type(name, (), {})  # noqa: E731
    msg = lambda name: type(name, (_Msg,), {})  # noqa: E731
    module("rclpy")
    module("rclpy.node", Node=_StubNode)
    module("rclpy.action", ActionServer=_StubActionServer,
           CancelResponse=SimpleNamespace(ACCEPT="accept", REJECT="reject"),
           GoalResponse=SimpleNamespace(ACCEPT="accept", REJECT="reject"))
    module("rclpy.callback_groups", ReentrantCallbackGroup=object, MutuallyExclusiveCallbackGroup=object)
    module("rclpy.executors", MultiThreadedExecutor=object)
    module("rcl_interfaces")
    module("rcl_interfaces.msg", ParameterDescriptor=lambda **kwargs: None)
    module("sensor_msgs")
    module("sensor_msgs.msg", JointState=msg("JointState"))
    module("std_srvs")
    module("std_srvs.srv", Trigger=srv("Trigger"))
    module("dh5_interfaces")
    module("dh5_interfaces.msg", AxisInfos=msg("AxisInfos"), Fingertip=srv("Fingertip"),
           FingertipArray=msg("FingertipArray"))
    module("dh5_interfaces.srv", **{n: srv(n) for n in
           ("Initialize", "MoveAxesPercent", "SetAxesValues", "SetAxisValue", "SetValues",
            "TwoFingerPinch")})
    module("dh5_interfaces.action",
           MoveAxes=type("MoveAxes", (), {"Result": srv("Result"), "Feedback": srv("Feedback")}))

    spec = importlib.util.spec_from_file_location("controller_node", NODE_FILE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture
def node(node_module, hand, monkeypatch):
    monkeypatch.setattr(node_module, "DH5Hand", lambda **kwargs: hand)
    hand.connect = lambda: True
    node = node_module.DH5Controller()
    node.move_poll_interval = 0  # keep the action loop from sleeping
    return node


def call(node, service, **fields):
    request = SimpleNamespace(**fields)
    response = SimpleNamespace(success=None, message=None)
    _, callback = node.services[f"dh5/{service}"]
    return callback(request, response)


def run_goal(node, goal_handle):
    return node.execute_move_axes(goal_handle)


# --------------------------------------------------------------------------
# Services
# --------------------------------------------------------------------------
def test_one_service_per_gesture_with_the_right_type(node):
    for gesture in gestures.GESTURES.values():
        if not gesture.ros_service:
            assert f"dh5/{gesture.name}" not in node.services
            continue
        srv_type, _ = node.services[f"dh5/{gesture.name}"]
        assert srv_type.__name__ == ("TwoFingerPinch" if gesture.is_scalable else "Trigger")


def test_skill_service_succeeds_and_moves_the_hand(node, device):
    response = call(node, "open_hand")
    assert response.success is True
    assert device.setpoints("position") == [round(reg.AXIS_LIMITS[a][1] * 0.99) for a in range(1, 7)]


def test_skill_precondition_failure_is_reported(node, device):
    # round_grip refuses to run from a closed hand.
    response = call(node, "round_grip")
    assert response.success is False
    assert "Pre-flight" in response.message


def test_scalable_skill_takes_width_and_variant(node):
    for axis in (2, 6):
        node.hand.api.serial_connection.device.set_position_percent(axis, 100)
    response = call(node, "two_finger_pinch", width=10, axis_mode="axis2")
    assert response.success is True


def test_bad_width_is_a_failure_not_a_crash(node):
    response = call(node, "two_finger_pinch", width=99, axis_mode="axis2")
    assert response.success is False
    assert "width" in response.message


def test_bus_error_becomes_failure(node, device):
    device.exception_code = 2
    response = call(node, "move_axis_percent", axis=1, value=50)
    assert response.success is False
    assert "Modbus exception" in response.message


def test_mismatched_axes_and_percents_rejected(node):
    response = call(node, "move_axes_percent", axes=[1, 2], percents=[10.0])
    assert response.success is False


def test_command_while_busy_is_refused(node):
    node._motion_lock.acquire()
    try:
        response = call(node, "open_hand")
    finally:
        node._motion_lock.release()
    assert response.success is False
    assert "Busy" in response.message


def test_initialize_validates_mode(node):
    assert call(node, "initialize", mode=9).success is False
    assert call(node, "initialize", mode=reg.INIT_MODE_OPEN).success is True


# --------------------------------------------------------------------------
# Stop
# --------------------------------------------------------------------------
def test_stop_works_while_another_command_holds_the_lock(node, device):
    device.position_feedback[3] = 700
    node._motion_lock.acquire()
    try:
        response = call(node, "stop")
    finally:
        node._motion_lock.release()
    assert response.success is True
    assert device.setpoints("position")[2] == 700
    assert node.hand.stop_requested


def test_stop_interrupts_a_blocking_move(node, device):
    device.set_statuses(0)  # the move would otherwise run into its 30 s timeout
    stopper = threading.Timer(0.1, node.hand.stop)
    stopper.start()
    response = call(node, "move_axis_percent", axis=2, value=80)
    stopper.join()
    assert response.success is False
    assert "Stopped" in response.message


def test_a_new_command_is_not_affected_by_an_earlier_stop(node):
    node.hand.stop()
    assert call(node, "move_axis_percent", axis=2, value=80).success is True


# --------------------------------------------------------------------------
# Sensor calibration
# --------------------------------------------------------------------------
def test_calibration_succeeds_on_a_resting_hand(node, device):
    response = call(node, "calibrate_sensors")
    assert response.success is True
    assert (reg.SENSOR_CALIBRATION_REGISTER, [1]) in device.writes


def test_calibration_is_refused_while_an_axis_moves(node, device):
    device.set_statuses(0, axes=[4])
    response = call(node, "calibrate_sensors")
    assert response.success is False
    assert "[4]" in response.message
    assert not any(address == reg.SENSOR_CALIBRATION_REGISTER for address, _ in device.writes)


def test_calibration_reports_fingers_still_under_load(node, device):
    device.set_fingertip("5", 0.0, 9.0, 0.0)
    response = call(node, "calibrate_sensors")
    assert response.success is False
    assert "'5'" in response.message


# --------------------------------------------------------------------------
# Publishers
# --------------------------------------------------------------------------
def test_state_is_published_as_axis_infos_and_joint_states(node, device):
    device.set_position_percent(2, 25)
    device.registers[reg.AXIS_STATUS_BASE_REGISTER + 1] = 2  # axis 2 stalled
    node.publish_state()
    info, = node.published["dh5/AxisInfos"]
    joints, = node.published["dh5/joint_states"]
    assert info.position_percent[1] == pytest.approx(25, abs=0.1)
    assert info.status == [1, 2, 1, 1, 1, 1]
    assert joints.position == info.position_percent
    assert joints.name == info.name


def test_publish_survives_a_failed_read(node, device):
    device.exception_code = 2  # the combined read fails, the fallback reads do not
    node.publish_state()
    assert len(node.published["dh5/AxisInfos"]) == 1


def test_fingertips_are_published_for_all_five_fingers(node, device):
    device.set_fingertip("2", -3.0, 0.5, 1.25)
    node.publish_fingertips()
    msg, = node.published["dh5/fingertips"]
    assert [tip.id for tip in msg.fingers] == list(reg.FINGERS)
    tip = msg.fingers[1]
    assert (tip.mx, tip.my, tip.fz) == pytest.approx((-3.0, 0.5, 1.25))
    assert not any(tip.stale for tip in msg.fingers)


def test_a_frozen_fingertip_is_flagged_stale(node, device):
    clock = SimpleNamespace(now=0.0)
    node._stale = StaleTracker(1.0, clock=lambda: clock.now)
    node.publish_fingertips()
    clock.now = 2.0
    device.set_fingertip("1", 0.1, 0.0, 0.0)  # finger 1 still updates, the others froze
    node.publish_fingertips()
    fingers = node.published["dh5/fingertips"][-1].fingers
    assert [tip.stale for tip in fingers] == [False, True, True, True, True]


def test_a_failed_fingertip_read_is_nan_and_stale(node, device):
    device.refused_reads.add((reg.FINGER_SENSOR_BASE_REGISTER["3"], 6))
    node.publish_fingertips()
    tip = node.published["dh5/fingertips"][0].fingers[2]
    assert tip.stale and math.isnan(tip.mx)


# --------------------------------------------------------------------------
# dh5/move_axes action
# --------------------------------------------------------------------------
def test_goals_are_rejected_while_busy(node):
    goal_cb = node.actions["dh5/move_axes"]["goal_callback"]
    assert goal_cb(None) == "accept"
    node._motion_lock.acquire()
    try:
        assert goal_cb(None) == "reject"
    finally:
        node._motion_lock.release()


def test_goal_moves_with_its_own_speeds(node, device):
    goal = FakeGoalHandle(axes=[2, 3], percents=[40.0, 60.0], speeds=[20, 30])
    result = run_goal(node, goal)
    assert goal.state == "succeeded" and result.success is True
    assert device.setpoints("speed")[1:3] == [20, 30]
    assert result.final_percents == pytest.approx([40, 60], abs=0.1)
    assert goal.feedback  # at least one progress message


def test_an_invalid_goal_is_aborted_with_a_reason(node):
    goal = FakeGoalHandle(axes=[2, 3], percents=[40.0], speeds=[])
    result = run_goal(node, goal)
    assert goal.state == "aborted" and result.success is False
    assert "same length" in result.message


def test_contact_stops_only_the_touching_finger(node, device):
    device.set_statuses(0, axes=[2, 3])
    device.position_feedback[2] = 300  # where axis 2 is when it touches

    def on_feedback(count):
        if count == 1:
            device.set_fingertip("2", 2.0, 0.0, 0.0)
        if count == 2:
            device.position_feedback.pop(2)  # the hold has reached the axis
            device.set_statuses(1)

    goal = FakeGoalHandle(on_feedback, axes=[2, 3], percents=[100.0, 100.0], stop_on_contact=True)
    result = run_goal(node, goal)
    assert result.success is True
    assert result.contact_axes == [2]
    assert device.setpoints("position")[1] == 300  # held where it touched
    assert device.setpoints("position")[2] == reg.AXIS_LIMITS[3][1]  # kept going


def test_a_canceled_goal_holds_its_axes(node, device):
    device.set_statuses(0)
    goal = FakeGoalHandle(axes=[4], percents=[100.0])
    goal.cancel = True
    result = run_goal(node, goal)
    assert goal.state == "canceled" and result.success is False
    assert any(address == reg.SETPOINT_BASE["position"] + 3 for address, _ in device.writes[1:])


def test_a_goal_that_never_arrives_times_out_and_is_held(node, device):
    device.set_statuses(0)
    goal = FakeGoalHandle(axes=[5], percents=[100.0], timeout=0.05)
    result = run_goal(node, goal)
    assert goal.state == "aborted"
    assert "Timed out" in result.message


def test_dh5_stop_ends_a_running_goal(node, device):
    device.set_statuses(0)
    goal = FakeGoalHandle(lambda count: node.hand.stop(), axes=[2], percents=[100.0])
    result = run_goal(node, goal)
    assert goal.state == "aborted"
    assert "Stopped" in result.message


def test_contact_stop_needs_the_sensors(node):
    node.sensors_available = False
    goal = FakeGoalHandle(axes=[2], percents=[100.0], stop_on_contact=True)
    result = run_goal(node, goal)
    assert goal.state == "aborted"
    assert "stop_on_contact" in result.message
