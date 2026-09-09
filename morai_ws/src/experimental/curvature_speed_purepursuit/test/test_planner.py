#!/usr/bin/env python3
import os
import sys
import unittest
import math


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from curvature_speed_purepursuit.planner import (  # noqa: E402
    PathPoint,
    build_speed_profile,
    clean_consecutive_duplicates,
    cumulative_arc_lengths,
    curvature_profile,
    three_point_curvature,
    curvature_speed_mps,
)


class PlannerTest(unittest.TestCase):
    def test_curvature_speed_depends_on_radius_not_turn_direction(self):
        for radius in (5., 15., 40.):
            for direction in (-1., 1.):
                self.assertAlmostEqual(curvature_speed_mps(direction / radius, 1.), math.sqrt(radius))
        self.assertIsNone(curvature_speed_mps(0., 1.))
        self.assertAlmostEqual(curvature_speed_mps(.1, .5), math.sqrt(5.))

    def test_invalid_curvature_cannot_create_a_permissive_speed(self):
        for curvature, lateral in ((float("nan"), 1.), (float("inf"), 1.), (.1, 0.), (.1, -1.), (.1, float("nan"))):
            with self.assertRaises(ValueError):
                curvature_speed_mps(curvature, lateral)

    def test_nominal_speed_profile_uses_same_budget_on_straights_and_mirrored_bends(self):
        s = [i * 10. for i in range(11)]
        for curvature in (0., 1. / 15., -1. / 15.):
            profile = build_speed_profile(s, [curvature] * 11, 10., 1., 1., 1., final_speed_mps=10.)
            expected = 10. if curvature == 0. else math.sqrt(15.)
            self.assertTrue(all(abs(speed - expected) < 1e-9 for speed in profile))

    def test_clean_keeps_one_lap_endpoint(self):
        points = [
            PathPoint(0.0, 0.0, 0.0),
            PathPoint(1.0, 0.0, 0.0),
            PathPoint(1.0, 0.0, 0.0),
            PathPoint(2.0, 0.0, 0.0),
            PathPoint(0.0, 0.0, 0.0),
        ]
        cleaned = clean_consecutive_duplicates(points)
        self.assertEqual(len(cleaned), 4)
        self.assertEqual(cleaned[0], cleaned[-1])

    def test_curvature_sign(self):
        left = [PathPoint(0.0, 0.0), PathPoint(1.0, 0.0), PathPoint(1.0, 1.0)]
        right = [PathPoint(0.0, 0.0), PathPoint(1.0, 0.0), PathPoint(1.0, -1.0)]
        self.assertGreater(three_point_curvature(left, 1), 0.0)
        self.assertLess(three_point_curvature(right, 1), 0.0)

    def test_speed_profile_slows_for_curvature_and_stops_at_goal(self):
        points = [PathPoint(float(index), 0.0) for index in range(11)]
        s_values = cumulative_arc_lengths(points)
        curvatures = [0.0] * len(points)
        curvatures[5] = 0.5
        profile = build_speed_profile(
            s_values,
            curvatures,
            max_speed_mps=2.0,
            lateral_accel_limit_mps2=1.0,
            max_accel_mps2=1.0,
            max_decel_mps2=1.0,
            initial_speed_mps=0.0,
            final_speed_mps=0.0,
        )
        self.assertEqual(profile[-1], 0.0)
        self.assertLess(profile[5], 2.0)
        self.assertLessEqual(profile[0], profile[1])

    def test_curvature_profile_has_same_length(self):
        points = [
            PathPoint(0.0, 0.0),
            PathPoint(1.0, 0.0),
            PathPoint(2.0, 0.2),
            PathPoint(3.0, 0.7),
            PathPoint(4.0, 1.5),
        ]
        profile = curvature_profile(points, half_window_points=1, smoothing_window=3)
        self.assertEqual(len(profile), len(points))


if __name__ == "__main__":
    unittest.main()
