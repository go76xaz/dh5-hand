"""Fingertip sensor bookkeeping that needs no hardware: spotting frozen
data and spotting contact.

Both work on one finger's raw `(mx, my, fz)` triple, keyed by the finger
ids in `dh5.registers.FINGERS`.
"""

import math
import time
from typing import Callable, Dict, Mapping, Optional, Sequence, Tuple

Reading = Tuple[float, float, float]


class StaleTracker:
    """Flags a finger whose reading has not changed at all for `timeout`
    seconds.

    Live sensor data always wobbles in its last digits, so a reading that
    stays bit-for-bit identical means the hand stopped updating its sensor
    registers - observed on the real DH5 as 3-3.6 s freezes. The check is
    time-based rather than a count of repeats, because the sensor may
    update slower than it is polled, which makes single repeats normal.
    """

    def __init__(self, timeout: float, clock: Callable[[], float] = time.monotonic):
        self.timeout = timeout
        self._clock = clock
        self._last: Dict[str, Tuple[Reading, float]] = {}

    def update(self, finger: str, reading: Optional[Sequence[float]]) -> bool:
        """Record `reading` and return True if `finger` is stale. A failed
        read (`None`) counts as stale."""
        if reading is None:
            return True
        reading = tuple(reading)
        now = self._clock()
        last = self._last.get(finger)
        if last is None or last[0] != reading:
            self._last[finger] = (reading, now)
            return False
        return now - last[1] > self.timeout


class ContactDetector:
    """Decides contact per finger from how far its reading has moved away
    from a baseline taken just before a motion starts.

    Comparing against a fresh baseline, not against zero, makes the check
    immune to calibration drift and to offsets left over from earlier
    contact.
    """

    def __init__(self, baseline: Mapping[str, Sequence[float]], threshold: float):
        if threshold <= 0:
            raise ValueError(f"Contact threshold must be positive, got {threshold}.")
        self.baseline = {finger: tuple(reading) for finger, reading in baseline.items()}
        self.threshold = threshold

    def magnitude(self, finger: str, reading: Sequence[float]) -> float:
        """Euclidean distance of `reading` from the finger's baseline."""
        return math.dist(self.baseline[finger], reading)

    def in_contact(self, finger: str, reading: Sequence[float]) -> bool:
        return self.magnitude(finger, reading) > self.threshold
