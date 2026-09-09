#!/usr/bin/env python3
"""LaneQualityEstimator의 주행 게이트 기본 동작 테스트."""

import math
import os
import sys
import unittest
from types import SimpleNamespace


LANE_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), "lane")
sys.path.insert(0, LANE_DIR)

from lane_quality import LaneQualityEstimator  # noqa: E402


def make_line(side, points=220, x_range=(5.0, 15.0), width=3.3):
    y = width / 2.0 if side == "left" else -width / 2.0
    return SimpleNamespace(
        n_points=points,
        x_range=x_range,
        y_at=lambda _x: y,
    )


def make_result(left=None, right=None, lateral=0.0, heading=0.0):
    return SimpleNamespace(
        ego_left=left,
        ego_right=right,
        lateral_error=lambda: lateral,
        heading_error=lambda: heading,
    )


class LaneQualityTest(unittest.TestCase):
    def test_two_stable_boundaries_are_primary_quality(self):
        estimator = LaneQualityEstimator()
        result = make_result(make_line("left"), make_line("right"))

        first = estimator.update(result)
        second = estimator.update(result)

        self.assertTrue(first["valid"])
        self.assertGreaterEqual(first["confidence"], 0.8)
        self.assertEqual(second["good_streak"], 2)
        self.assertAlmostEqual(first["lane_width_m"], 3.3, places=3)

    def test_one_boundary_cannot_reach_primary_threshold(self):
        estimator = LaneQualityEstimator()
        result = make_result(make_line("left"), None)

        quality = estimator.update(result)

        self.assertTrue(quality["valid"])
        self.assertLess(quality["confidence"], 0.8)
        self.assertFalse(quality["both_visible"])

    def test_invalid_result_has_zero_confidence(self):
        estimator = LaneQualityEstimator()
        result = make_result(None, None, lateral=math.nan, heading=math.nan)

        quality = estimator.update(result)

        self.assertFalse(quality["valid"])
        self.assertEqual(quality["confidence"], 0.0)
        self.assertEqual(quality["good_streak"], 0)

    def test_impossible_width_or_reversed_boundaries_cannot_be_primary(self):
        for width in (.5, 8., -3.3):
            with self.subTest(width=width):
                quality = LaneQualityEstimator().update(make_result(
                    make_line("left", width=width), make_line("right", width=width)))
                self.assertFalse(quality["valid"])
                self.assertLess(quality["confidence"], .8)

    def test_temporal_geometry_jump_cannot_be_primary(self):
        estimator = LaneQualityEstimator()
        estimator.update(make_result(make_line("left"), make_line("right")))
        quality = estimator.update(make_result(make_line("left"), make_line("right"), lateral=1., heading=.4))
        self.assertLess(quality["confidence"], .8)

    def test_extrapolated_or_nonfinite_boundary_is_not_primary(self):
        for line in (make_line("left", points=math.nan), make_line("left", x_range=(20., 30.))):
            quality = LaneQualityEstimator().update(make_result(line, make_line("right")))
            self.assertFalse(quality["valid"])

    def test_far_heading_requires_observed_support_on_each_boundary(self):
        for x_range in ((5., 12.), (8., 20.)):
            with self.subTest(x_range=x_range):
                quality = LaneQualityEstimator().update(make_result(
                    make_line("left", x_range=x_range), make_line("right")))
                self.assertFalse(quality["valid"])
                self.assertEqual(quality["confidence"], 0.)

    def test_crossing_diverging_or_nonfinite_far_boundary_is_rejected(self):
        for far_width in (-1., 6., math.nan):
            with self.subTest(far_width=far_width):
                left = make_line("left")
                left.y_at = lambda x: 1.65 if x == 7. else -1.65 + far_width
                quality = LaneQualityEstimator().update(make_result(left, make_line("right")))
                self.assertFalse(quality["valid"])

    def test_bad_width_between_valid_endpoints_is_rejected(self):
        left = make_line("left")
        left.y_at = lambda x: 1.65 + .1 * (x - 7.) * (14. - x)
        quality = LaneQualityEstimator().update(make_result(left, make_line("right")))
        self.assertFalse(quality["valid"])

    def test_curved_parallel_boundaries_remain_primary(self):
        left, right = make_line("left"), make_line("right")
        left.y_at = lambda x: .008 * x * x + 1.65
        right.y_at = lambda x: .008 * x * x - 1.65
        quality = LaneQualityEstimator().update(make_result(left, right, lateral=-.392, heading=.166))
        self.assertTrue(quality["valid"])
        self.assertGreaterEqual(quality["confidence"], .8)
        self.assertAlmostEqual(quality["lane_width_min_m"], 3.3)
        self.assertAlmostEqual(quality["lane_width_max_m"], 3.3)


if __name__ == "__main__":
    unittest.main()
