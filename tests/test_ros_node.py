"""The ROS2 node's service logic, run against the fake DH5 with stub `rclpy`.

ROS is not installed on the development machine, so the ROS modules are
replaced with minimal stand-ins. What is under test is the node's own logic:
that services turn library results into honest success/message responses,
and that the skill services follow the gesture table.
"""

import importlib.util
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import pytest

from dh5 import gestures, registers as reg

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
        self.params, self.services, self.published = {}, {}, []

    def declare_parameter(self, name, default):
        self.params[name] = SimpleNamespace(value=default)

    def get_parameter(self, name):
        return self.params[name]

    def get_logger(self):
        log = SimpleNamespace()
        for level in ("info", "warning", "error"):
            setattr(log, level, lambda *a, **k: None)
        return log

    def create_publisher(self, *a, **k):
        return SimpleNamespace(publish=self.published.append)

    def create_service(self, srv_type, name, callback, callback_group=None):
        self.services[name] = (srv_type, callback)

    def create_timer(self, *a, **k):
        return None

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))

    def destroy_node(self):
        pass


@pytest.fixture
def node_module(monkeypatch):
    def module(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, mod)
        return mod

    srv = lambda name: type(name, (), {})  # noqa: E731
    module("rclpy")
    module("rclpy.node", Node=_StubNode)
    module("rclpy.callback_groups", ReentrantCallbackGroup=object)
    module("rclpy.executors", MultiThreadedExecutor=object)
    module("std_srvs")
    module("std_srvs.srv", Trigger=srv("Trigger"))
    module("dh5_interfaces")
    module("dh5_interfaces.msg", AxisInfos=_Msg)
    module("dh5_interfaces.srv", **{n: srv(n) for n in
           ("Initialize", "MoveAxesPercent", "SetAxisValue", "SetValues", "TwoFingerPinch")})

    spec = importlib.util.spec_from_file_location("controller_node", NODE_FILE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture
def node(node_module, hand, monkeypatch):
    monkeypatch.setattr(node_module, "DH5Hand", lambda **kwargs: hand)
    hand.connect = lambda: True
    return node_module.DH5Controller()


def call(node, service, **fields):
    request = SimpleNamespace(**fields)
    response = SimpleNamespace(success=None, message=None)
    _, callback = node.services[f"dh5/{service}"]
    return callback(request, response)


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


def test_publish_survives_a_failed_read(node, device):
    device.exception_code = 2  # first feedback read fails
    node.publish_joint_states()
    assert len(node.published) == 1
