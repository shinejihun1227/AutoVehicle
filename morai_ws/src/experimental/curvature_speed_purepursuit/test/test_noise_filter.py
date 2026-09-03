#!/usr/bin/env python3
import math
import os
import sys
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from curvature_speed_purepursuit.noise_filter import (  # noqa: E402
    MotionState,
    OdometryNoiseModel,
    RobustOdometryFilter,
    median_angle,
)


class NoiseFilterTest(unittest.TestCase):
    def test_noise_model_is_repeatable_with_same_seed(self):
        source = MotionState(10.0, -2.0, 3.1, 2.0, 0.1)
        first = OdometryNoiseModel(0.25, 0.03, 0.05, 0.02, 0.002, 0.01, 7)
        second = OdometryNoiseModel(0.25, 0.03, 0.05, 0.02, 0.002, 0.01, 7)
        self.assertEqual(first.apply(source, 0.05), second.apply(source, 0.05))

    def test_angle_median_handles_pi_wrap(self):
        result = median_angle([3.13, -3.13, 3.12], reference=3.13)
        self.assertGreater(abs(result), 3.0)

    def test_filter_rejects_large_jump(self):
        filter_ = RobustOdometryFilter(
            median_window_size=3,
            ema_alpha=1.0,
            max_position_jump_m=2.0,
            max_measurement_speed_mps=10.0,
            max_yaw_jump_rad=math.radians(45.0),
        )
        self.assertIsNotNone(filter_.update(MotionState(0.0, 0.0, 0.0, 0.0, 0.0), 0.1))
        self.assertIsNotNone(filter_.update(MotionState(0.1, 0.0, 0.01, 0.0, 0.0), 0.1))
        self.assertIsNone(filter_.update(MotionState(3.0, 0.0, 0.01, 0.0, 0.0), 0.1))

    def test_filter_smooths_with_ema(self):
        filter_ = RobustOdometryFilter(
            median_window_size=1,
            ema_alpha=0.5,
            max_position_jump_m=10.0,
            max_measurement_speed_mps=100.0,
            max_yaw_jump_rad=math.pi,
        )
        first = filter_.update(MotionState(0.0, 0.0, 0.0, 0.0, 0.0), 0.1)
        second = filter_.update(MotionState(2.0, 0.0, 0.2, 2.0, 0.0), 0.1)
        self.assertEqual(first.x, 0.0)
        self.assertAlmostEqual(second.x, 1.0)
        self.assertAlmostEqual(second.yaw, 0.1)

    def test_default_noise_scale_keeps_nominal_motion(self):
        noise = OdometryNoiseModel(0.25, 0.03, 0.05, 0.02, 0.002, 0.01, 20260901)
        filter_ = RobustOdometryFilter(
            median_window_size=3,
            ema_alpha=0.35,
            max_position_jump_m=5.0,
            max_measurement_speed_mps=50.0,
            max_yaw_jump_rad=math.radians(45.0),
        )
        accepted = 0
        for index in range(200):
            true_state = MotionState(index * 0.04, 0.0, 0.0, 2.0, 0.0)
            if filter_.update(noise.apply(true_state, 0.02), 0.02) is not None:
                accepted += 1
        self.assertGreaterEqual(accepted, 180)


if __name__ == "__main__":
    unittest.main()
