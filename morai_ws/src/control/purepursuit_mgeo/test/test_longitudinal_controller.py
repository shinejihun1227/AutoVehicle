#!/usr/bin/env python3
import os
import sys
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from purepursuit_mgeo.longitudinal_controller import SpeedPIController  # noqa: E402


class LongitudinalControllerTest(unittest.TestCase):
    def setUp(self):
        self.controller = SpeedPIController(
            kp=0.8,
            ki=0.05,
            max_accel_mps2=1.0,
            max_decel_mps2=1.5,
            integral_limit_kph_s=10.8,
            speed_error_deadband_kph=0.1,
        )

    def test_speed_error_accelerates(self):
        output = self.controller.update(7.2, 0.0, 0.05)
        self.assertGreater(output.accel, 0.0)
        self.assertEqual(output.brake, 0.0)
        self.assertLessEqual(output.accel, 1.0)

    def test_speed_error_brakes(self):
        output = self.controller.update(3.6, 7.2, 0.05)
        self.assertEqual(output.accel, 0.0)
        self.assertGreater(output.brake, 0.0)
        self.assertLessEqual(output.brake, 1.0)

    def test_stop_resets_integral_and_applies_full_brake(self):
        self.controller.update(7.2, 0.0, 0.05)
        output = self.controller.update(0.0, 3.6, 0.05, stop=True)
        self.assertEqual(output.accel, 0.0)
        self.assertEqual(output.brake, 1.0)
        self.assertEqual(self.controller.integral_error_mps_s, 0.0)

    def test_deadband_is_neutral(self):
        output = self.controller.update(3.6, 3.63, 0.05)
        self.assertEqual(output.accel, 0.0)
        self.assertEqual(output.brake, 0.0)


if __name__ == "__main__":
    unittest.main()
