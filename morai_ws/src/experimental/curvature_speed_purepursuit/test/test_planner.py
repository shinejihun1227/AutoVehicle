#!/usr/bin/env python3
import os
import sys
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from curvature_speed_purepursuit.planner import (  # noqa: E402
    PathPoint,
    build_speed_profile,
    clean_consecutive_duplicates,
    cumulative_arc_lengths,
    curvature_profile,
    three_point_curvature,
)


class PlannerTest(unittest.TestCase):
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
