"""Percent mapping, register-pair floats, and the Hualichuang formula."""

import struct

import pytest

from dh5 import registers as reg
from dh5.conversions import (
    first_value,
    hualichuang_tangential_force,
    percent_to_position,
    position_to_percent,
    registers_to_float,
    registers_to_floats,
)


class TestPercentMapping:
    @pytest.mark.parametrize("axis", sorted(reg.AXIS_LIMITS))
    def test_endpoints_hit_the_limits_exactly(self, axis):
        low, high = reg.AXIS_LIMITS[axis]
        assert percent_to_position(0, (low, high)) == low
        assert percent_to_position(100, (low, high)) == high

    def test_midpoint(self):
        assert percent_to_position(50, (0, 1000)) == 500

    @pytest.mark.parametrize("percent,expected", [(-10, 0), (0, 0), (150, 1000), (100, 1000)])
    def test_out_of_range_is_clamped_not_extrapolated(self, percent, expected):
        assert percent_to_position(percent, (0, 1000)) == expected

    @pytest.mark.parametrize("axis", sorted(reg.AXIS_LIMITS))
    @pytest.mark.parametrize("percent", [0, 12.5, 37, 58, 83, 100])
    def test_round_trip_is_stable_to_a_tenth_of_a_percent(self, axis, percent):
        limits = reg.AXIS_LIMITS[axis]
        assert position_to_percent(percent_to_position(percent, limits), limits) == pytest.approx(
            percent, abs=0.1
        )

    def test_degenerate_axis_does_not_divide_by_zero(self):
        assert position_to_percent(5, (10, 10)) == 0.0

    def test_position_below_range_clamps(self):
        assert position_to_percent(-50, (0, 1000)) == 0.0
        assert position_to_percent(5000, (0, 1000)) == 100.0


class TestFirstValue:
    def test_unwraps_a_single_element_list(self):
        assert first_value([42]) == 42

    def test_passes_a_bare_value_through(self):
        assert first_value(42) == 42

    def test_empty_response_is_an_error_not_an_index_crash(self):
        with pytest.raises(ValueError):
            first_value([])


class TestRegisterFloats:
    def make_words(self, value: float):
        """Split a float into (low_address_word, high_address_word) the way
        the DH5 lays it out: the lower address carries the high word."""
        raw = struct.pack("<f", value)
        low_word, high_word = struct.unpack("<HH", raw)
        return high_word, low_word  # (at base+0, at base+1)

    @pytest.mark.parametrize("value", [0.0, 1.0, -1.0, 3.14159, 1234.5, -0.001])
    def test_high_low_is_the_documented_hardware_order(self, value):
        first, second = self.make_words(value)
        assert registers_to_float(first, second) == pytest.approx(value, rel=1e-6)

    def test_low_high_is_available_for_other_sensor_variants(self):
        # The two orders must disagree, or the byte_order switch would be
        # pointless and dump_finger_sensor_raw could not tell them apart.
        assert registers_to_float(0x1234, 0x5678, "high_low") != registers_to_float(
            0x1234, 0x5678, "low_high"
        )

    def test_unknown_byte_order_is_rejected(self):
        with pytest.raises(ValueError):
            registers_to_float(0, 0, byte_order="sideways")

    def test_odd_register_count_is_rejected(self):
        with pytest.raises(ValueError):
            registers_to_floats([1, 2, 3])

    def test_converts_a_whole_block(self):
        words = []
        for value in (1.0, 2.0, 3.0):
            words.extend(self.make_words(value))
        assert registers_to_floats(words) == pytest.approx([1.0, 2.0, 3.0])


class TestHualichuang:
    def test_matches_the_formula_printed_in_the_manual(self):
        fx, fy = hualichuang_tangential_force(mx=7.0, my=14.0, fz=1.0)
        assert fx == pytest.approx((14.0 + 7 * 1.0) / 7)
        assert fy == pytest.approx(-1.0)

    def test_fy_ignores_fz_which_is_the_documented_caveat(self):
        # If this ever starts failing, the transcription caveat in
        # `hualichuang_tangential_force` has been resolved and its docstring
        # needs updating too.
        _, fy_low = hualichuang_tangential_force(1.0, 0.0, 0.0)
        _, fy_high = hualichuang_tangential_force(1.0, 0.0, 1000.0)
        assert fy_low == fy_high
