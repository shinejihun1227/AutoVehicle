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


if __name__ == "__main__":
    unittest.main()
