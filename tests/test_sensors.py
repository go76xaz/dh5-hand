"""Fingertip bookkeeping: frozen-data and contact detection."""

import pytest

from dh5.sensors import ContactDetector, StaleTracker


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class TestStaleTracker:
    def test_changing_readings_are_never_stale(self):
        clock = Clock()
        tracker = StaleTracker(1.0, clock=clock)
        for value in range(5):
            clock.now += 2
            assert tracker.update("1", (value, 0, 0)) is False

    def test_a_repeat_within_the_timeout_is_normal(self):
        clock = Clock()
        tracker = StaleTracker(1.0, clock=clock)
        tracker.update("1", (1, 2, 3))
        clock.now = 0.9
        assert tracker.update("1", (1, 2, 3)) is False

    def test_an_unchanged_reading_past_the_timeout_is_stale(self):
        clock = Clock()
        tracker = StaleTracker(1.0, clock=clock)
        tracker.update("1", (1, 2, 3))
        clock.now = 1.5
        assert tracker.update("1", (1, 2, 3)) is True
        clock.now = 1.6
        assert tracker.update("1", (1, 2, 4)) is False  # updating again

    def test_fingers_are_tracked_separately(self):
        clock = Clock()
        tracker = StaleTracker(1.0, clock=clock)
        tracker.update("1", (0, 0, 0))
        clock.now = 5
        assert tracker.update("2", (0, 0, 0)) is False

    def test_a_failed_read_is_stale(self):
        assert StaleTracker(1.0).update("1", None) is True


class TestContactDetector:
    def test_contact_is_measured_from_the_baseline_not_from_zero(self):
        detector = ContactDetector({"2": (3.0, 0.0, 3.0)}, threshold=0.5)
        assert not detector.in_contact("2", (3.2, 0.1, 3.1))  # offset, no contact
        assert detector.in_contact("2", (1.0, 0.0, 3.0))

    def test_every_channel_counts(self):
        detector = ContactDetector({"1": (0, 0, 0)}, threshold=0.5)
        assert detector.magnitude("1", (0.3, 0.4, 0)) == pytest.approx(0.5)
        assert detector.in_contact("1", (0, 0, -0.6))

    def test_threshold_must_be_positive(self):
        with pytest.raises(ValueError):
            ContactDetector({"1": (0, 0, 0)}, threshold=0)
