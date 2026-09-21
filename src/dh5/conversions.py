"""Pure conversions between raw Modbus values and physical units.

Nothing in here touches hardware or imports `serial`, which is what makes
every one of these functions directly testable.
"""

import struct
from typing import Sequence, Tuple


def first_value(response):
    """Normalise a single-register read.

    `send_modbus_command` returns a one-element list for a read of one
    register, but callers (and fakes) sometimes hand back a bare value.
    Accept either.
    """
    if isinstance(response, (list, tuple)):
        if not response:
            raise ValueError("Empty register response")
        return response[0]
    return response


def percent_to_position(percent: float, limits: Tuple[int, int]) -> int:
    """Map 0-100 % onto an axis's raw stroke, clamped to both ends."""
    percent = min(100.0, max(0.0, float(percent)))
    low, high = limits
    position = int(round(low + (high - low) * (percent / 100.0)))
    return min(high, max(low, position))


def position_to_percent(position: float, limits: Tuple[int, int]) -> float:
    """Map a raw position back onto 0-100 %, clamped to both ends."""
    low, high = limits
    if high == low:
        return 0.0
    percent = ((position - low) / (high - low)) * 100.0
    return min(100.0, max(0.0, percent))


def registers_to_float(low: int, high: int, byte_order: str = "high_low") -> float:
    """Combine two consecutive 16-bit registers into one IEEE-754 float.

    The manual's union-based snippet (section 6.4.3, `shortsToFloat(sLow,
    sHigh)`) shows how the two words combine but not which physical
    register is the low word. Confirmed empirically on real hardware: the
    register at the LOWER address holds the HIGH word, which is the
    opposite of what the snippet's parameter order suggests. Hence the
    "high_low" default; "low_high" is kept for debugging other sensor
    variants. See `DH5Hand.dump_finger_sensor_raw`.
    """
    if byte_order == "low_high":
        raw = struct.pack("<HH", low & 0xFFFF, high & 0xFFFF)
    elif byte_order == "high_low":
        raw = struct.pack("<HH", high & 0xFFFF, low & 0xFFFF)
    else:
        raise ValueError("byte_order must be 'low_high' or 'high_low'")
    return struct.unpack("<f", raw)[0]


def registers_to_floats(values: Sequence[int], byte_order: str = "high_low"):
    """Convert a flat list of register words into floats, two words each."""
    if len(values) % 2:
        raise ValueError(f"Need an even number of registers, got {len(values)}")
    return [
        registers_to_float(values[i], values[i + 1], byte_order=byte_order)
        for i in range(0, len(values), 2)
    ]


def hualichuang_tangential_force(mx: float, my: float, fz: float) -> Tuple[float, float]:
    """Derive tangential forces (fx, fy) from one Hualichuang 3-axis
    reading, using the formula printed in section 6.4.3:

        fx = (My + 7 * Fz) / 7
        fy = (0 * Fz - Mx) / 7

    CAVEAT: the `0 * Fz` term looks like a coefficient lost when the PDF
    was generated, not an intentional zero — it makes fy independent of Fz,
    which is unusual for this kind of sensor. It is transcribed literally.
    Treat fy with suspicion and prefer the raw (mx, my, fz) triple until
    the real coefficient is confirmed with DH-Robotics, or empirically:
    apply a known pure-normal load and see whether fy really pins to 0
    whenever Mx is 0, as this formula forces it to.
    """
    fx = (my + 7 * fz) / 7
    fy = (0 * fz - mx) / 7
    return fx, fy
