"""Real BEV error/quality -> fallback -> kinematic bicycle, without inference/ROS.

Synthetic observed boundaries describe a straight road or concentric circular
lane edges. This checks direction and tracking geometry, not camera accuracy,
slip, suspension, actuator delay, or MORAI's pedal dynamics.
"""
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np
import test_camera_fallback as fixtures


LANE = Path(__file__).resolve().parents[3] / "detection" / "camera_perception" / "lane"


def load_geometry():
    spec = importlib.util.spec_from_file_location("_closed_loop_lane_detection", LANE / "lane_detection.py")
    module = importlib.util.module_from_spec(spec)
    mocks = {"cv2": Mock(), "torch": NS(no_grad=lambda: lambda function: function),
             "GenerateLabels": NS(load_camera=Mock()), "seg_model": NS(LaneSegNet=Mock()),
             "seg_dataset": NS(IMAGENET_MEAN=[0., 0., 0.], IMAGENET_STD=[1., 1., 1.],
                               INPUT_H=256, INPUT_W=640, NUM_CLASSES=5), spec.name: module}
    with patch.dict(sys.modules, mocks), patch.object(sys, "path", [str(LANE)] + sys.path):
        spec.loader.exec_module(module)
    return module


class LaneClosedLoopTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry = load_geometry()
        spec = importlib.util.spec_from_file_location("_closed_loop_lane_quality", LANE / "lane_quality.py")
        quality = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(quality)
        cls.quality_type = quality.LaneQualityEstimator

    def setUp(self):
        self.fixture = fixtures.FallbackTest()
        self.fixture.setUp()
        self.quality = self.quality_type()

    def observe(self, pose, radius=None):
        px, py, yaw = pose
        x = np.linspace(5., 17., 240)
        lanes = []
        for lane_id, offset in ((-1, 1.65), (1, -1.65)):
            if radius is None:
                y = (offset - py) / math.cos(yaw) - x * math.tan(yaw)
            else:
                sign = math.copysign(1., radius)
                cx = -px * math.cos(yaw) + (radius - py) * math.sin(yaw)
                cy = px * math.sin(yaw) + (radius - py) * math.cos(yaw)
                boundary_radius = abs(radius) - sign * offset
                y = cy - sign * np.sqrt(boundary_radius**2 - (x - cx)**2)
            lanes.append(self.geometry.Lane(cls=1, coef=np.polyfit(x, y, 2),
                                           x_range=(float(x[0]), float(x[-1])),
                                           points=np.column_stack((x, y)), lane_id=lane_id))
        return self.geometry.DetectionResult(lanes=lanes)

    def frame(self, pose, state, radius=None):
        result = self.observe(pose, radius)
        quality = self.quality.update(result)
        self.assertTrue(quality["valid"], quality)
        self.assertGreaterEqual(quality["confidence"], .8, quality)
        self.fixture.lane(lateral=quality["lateral_error"], heading=quality["heading_error"],
                          confidence=quality["confidence"], valid=quality["valid"])
        return self.fixture.frame(state, lane=False, speed=2., steering=0.)

    def track(self, initial_offset=0., initial_yaw=0., radius=None):
        # Each subcase starts a new vehicle/sensor epoch.
        self.setUp()
        pose = [0., initial_offset, initial_yaw]
        for _ in range(25):
            self.frame(pose, "NORMAL", radius)
        errors, active_count = [], 0
        first_steering = None
        previous_steering = 0.
        for _ in range(260):  # 13 s, at most 26 m: inside the default blackout budget.
            output, status = self.frame(pose, "GPS_BLACKOUT", radius)
            self.assertLessEqual(abs(output.steering - previous_steering), .8 * .05 + 1e-9)
            previous_steering = output.steering
            if status["mode"] == "gps_blackout_camera_fallback":
                self.assertEqual(output.brake, 0., status)
                if first_steering is None:
                    first_steering = output.steering
                active_count += 1
                px, py, yaw = pose
                pose = [px + 2. * math.cos(yaw) * .05, py + 2. * math.sin(yaw) * .05,
                        yaw + 2. / 3.0 * math.tan(output.steering) * .05]
            error = (pose[1] if radius is None else
                     abs(radius) - math.hypot(pose[0], pose[1] - radius))
            errors.append(abs(error))
        self.assertGreater(active_count, 240)
        return {"final_error_m": errors[-1], "max_error_m": max(errors),
                "first_steering": first_steering, "final_yaw": pose[2]}

    def test_vehicle_left_of_straight_lane_steers_right_and_converges(self):
        metrics = self.track(initial_offset=.7)
        self.assertLess(metrics["first_steering"], 0.)
        self.assertLess(metrics["final_error_m"], .15, metrics)
        self.assertLessEqual(metrics["max_error_m"], .72, metrics)

    def test_vehicle_right_of_straight_lane_steers_left_and_converges(self):
        metrics = self.track(initial_offset=-.7)
        self.assertGreater(metrics["first_steering"], 0.)
        self.assertLess(metrics["final_error_m"], .15, metrics)
        self.assertLessEqual(metrics["max_error_m"], .72, metrics)

    def test_heading_error_without_initial_offset_converges(self):
        for yaw in (-.07, .07):
            with self.subTest(yaw=yaw):
                metrics = self.track(initial_yaw=yaw)
                self.assertLess(metrics["first_steering"] * yaw, 0., metrics)
                self.assertLess(metrics["final_error_m"], .15, metrics)
                self.assertLess(abs(metrics["final_yaw"]), .03, metrics)

    def test_gentle_left_and_right_curves_do_not_cut_inside_lane(self):
        for radius in (-60., 60.):
            with self.subTest(radius=radius):
                metrics = self.track(radius=radius)
                self.assertGreater(metrics["first_steering"] * radius, 0., metrics)
                self.assertLess(metrics["max_error_m"], .35, metrics)


if __name__ == "__main__":
    unittest.main()
