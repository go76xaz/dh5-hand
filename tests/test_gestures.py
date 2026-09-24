"""Gesture table: interpolation, preconditions, and the runner."""

import pytest

from dh5 import gestures
from dh5 import registers as reg
from dh5.gestures import GESTURES, Pose


def open_everything(device):
    """Put every axis fully open, so preconditions are satisfied."""
    for axis in range(1, 7):
        device.set_position_percent(axis, 100)


class TestPoseResolution:
    def test_fixed_targets_pass_straight_through(self):
        assert Pose(fixed={1: 40, 2: 100}).targets() == {1: 40, 2: 100}

    def test_ramp_at_zero_width_uses_the_low_endpoint(self):
        pose = Pose(ramp={2: (58, 83)})
        assert pose.targets(width=0, max_width=25) == {2: 58}

    def test_ramp_at_full_width_uses_the_high_endpoint(self):
        pose = Pose(ramp={2: (58, 83)})
        assert pose.targets(width=25, max_width=25) == {2: 83}

    def test_ramp_interpolates_linearly(self):
        pose = Pose(ramp={2: (58, 83)})
        assert pose.targets(width=12.5, max_width=25)[2] == pytest.approx(70.5)

    def test_ramp_is_clamped_to_its_span(self):
        pose = Pose(ramp={2: (58, 83)})
        assert pose.targets(width=100, max_width=25) == {2: 83}

    def test_fixed_and_ramped_axes_combine(self):
        pose = Pose(fixed={1: 50}, ramp={6: (25, 50)})
        assert pose.targets(width=25, max_width=25) == {1: 50, 6: 50}


class TestGestureTable:
    @pytest.mark.parametrize("name", sorted(GESTURES))
    def test_every_gesture_is_self_consistent(self, name):
        gesture = GESTURES[name]
        assert gesture.name == name, "table key and gesture name must agree"
        assert gesture.summary
        assert gesture.default_variant in gesture.variants
        assert gesture.poses()

    @pytest.mark.parametrize("name", sorted(GESTURES))
    def test_every_target_axis_is_real_and_in_range(self, name):
        gesture = GESTURES[name]
        for variant in gesture.variant_names:
            for pose in gesture.poses(variant):
                widths = [0, gesture.max_width] if gesture.is_scalable else [0]
                for width in widths:
                    for axis, percent in pose.targets(width, gesture.max_width).items():
                        assert axis in reg.AXIS_LIMITS
                        assert 0 <= percent <= 100

    @pytest.mark.parametrize("name", sorted(GESTURES))
    def test_guarded_axes_actually_have_a_target(self, name):
        """`require_above` compares against a target, so every guarded axis
        must appear in the resolved pose."""
        gesture = GESTURES[name]
        for variant in gesture.variant_names:
            for pose in gesture.poses(variant):
                targets = pose.targets(0, gesture.max_width)
                for axis in pose.require_above:
                    assert axis in targets

    def test_unknown_gesture_names_are_rejected(self):
        with pytest.raises(ValueError):
            gestures.get("jazz_hands")

    def test_unknown_variant_is_rejected(self):
        with pytest.raises(ValueError):
            GESTURES["two_finger_pinch"].poses("axis9")


class TestPerform:
    def test_open_hand_drives_every_axis_to_99_percent(self, hand, device):
        gestures.perform(hand, "open_hand", wait=False)
        assert device.setpoints("position") == [round(reg.AXIS_LIMITS[a][1] * 0.99) for a in range(1, 7)]

    def test_wink_runs_all_three_poses(self, hand, device):
        open_everything(device)
        results = gestures.perform(hand, "wink", wait=False)
        assert len(results) == 3

    def test_round_grip_refuses_from_a_closed_hand(self, hand, device):
        for axis in range(1, 7):
            device.set_position_percent(axis, 0)
        assert gestures.perform(hand, "round_grip", wait=False) is None

    def test_round_grip_runs_from_an_open_hand(self, hand, device):
        open_everything(device)
        assert gestures.perform(hand, "round_grip", wait=False) is not None

    def test_a_stopped_gesture_skips_its_remaining_poses(self, hand, device):
        open_everything(device)
        hand.stop()
        writes_before = len(device.writes)
        assert gestures.perform(hand, "wink", wait=False) == []
        assert len(device.writes) == writes_before

    def test_a_blocked_gesture_writes_nothing(self, hand, device):
        for axis in range(1, 7):
            device.set_position_percent(axis, 0)
        device.writes.clear()
        gestures.perform(hand, "round_grip", wait=False)
        assert device.writes == []

    @pytest.mark.parametrize("variant", ["axis2", "axis3"])
    @pytest.mark.parametrize("width", [0, 10, 25])
    def test_pinch_variants_and_widths_reach_the_hardware(self, hand, device, variant, width):
        open_everything(device)
        assert gestures.perform(hand, "two_finger_pinch", width=width,
                                variant=variant, wait=False) is not None

    def test_pinch_width_changes_the_opening(self, hand, device):
        open_everything(device)
        gestures.perform(hand, "two_finger_pinch", width=0, wait=False)
        narrow = device.setpoints("position")[1]

        open_everything(device)
        gestures.perform(hand, "two_finger_pinch", width=25, wait=False)
        wide = device.setpoints("position")[1]

        assert wide > narrow

    def test_pinch_rejects_an_out_of_range_width(self, hand, device):
        open_everything(device)
        with pytest.raises(ValueError):
            gestures.perform(hand, "two_finger_pinch", width=40)

    def test_width_on_a_fixed_gesture_is_an_error(self, hand):
        with pytest.raises(ValueError):
            gestures.perform(hand, "open_hand", width=5)

    def test_wink2_moves_to_99_then_staggers_axes_2_to_5(self, hand, device, monkeypatch):
        monkeypatch.setattr(gestures.time, "sleep", lambda seconds: None)
        for axis in range(1, 7):
            device.set_position_percent(axis, 50)
        device.writes.clear()

        results = gestures.perform(hand, "wink2", wait=False)

        assert len(results) == 2
        assert device.setpoints("position")[0] == round(reg.AXIS_LIMITS[1][1] * 0.99)  # axis 1
        assert device.setpoints("position")[5] == round(reg.AXIS_LIMITS[6][1] * 0.99)  # axis 6

        stagger_writes = [w for w in device.writes if len(w[1]) == 1]
        assert len(stagger_writes) == 8
        moved_axes = [(w[0] - reg.SETPOINT_BASE["position"]) + 1 for w in stagger_writes]
        assert moved_axes == [2, 3, 4, 5, 2, 3, 4, 5]

    def test_point_prepares_axes_before_the_combined_move(self, hand, device):
        for axis in range(1, 7):
            device.set_position_percent(axis, 0)
        device.writes.clear()

        gestures.perform(hand, "point", wait=False)

        single_writes = [w for w in device.writes if len(w[1]) == 1]
        block_writes = [w for w in device.writes if len(w[1]) == reg.NUM_AXES]
        assert single_writes, "axes below their target should be moved individually first"
        assert block_writes, "the final pose should be one simultaneous move"
        assert device.writes.index(block_writes[-1]) > device.writes.index(single_writes[0])


class TestDescribe:
    def test_scalable_gestures_advertise_their_width(self):
        assert "width" in gestures.describe(GESTURES["two_finger_pinch"])

    def test_multi_variant_gestures_advertise_their_variants(self):
        text = gestures.describe(GESTURES["two_finger_pinch"])
        assert "axis2" in text and "axis3" in text

    def test_simple_gestures_stay_plain(self):
        assert gestures.describe(GESTURES["open_hand"]) == GESTURES["open_hand"].summary
