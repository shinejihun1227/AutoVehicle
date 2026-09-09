"""Synthetic segmentation masks exercise geometry without ROS/model loading."""

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "lane"))
sys.path.insert(0, str(PACKAGE / "src"))
sys.path.insert(0, str(PACKAGE.parents[1] / "control" / "stopline_control" / "src"))

from stopline_geometry import select_stopline
from stopline_control.core import StopLineControllerCore
from camera_perception.traffic_signal import straight_observation


def paint(mask, distance, width=3.5, thickness=.25, lateral=0., slope=0.):
    x = 40. - (np.arange(mask.shape[0])[:, None] + .5) * .05
    y = 10. - (np.arange(mask.shape[1])[None, :] + .5) * .05
    mask[(np.abs(x - (distance + slope * y)) <= thickness / 2.)
         & (np.abs(y - lateral) <= width / 2.)] = 4


class StopLineGeometryTest(unittest.TestCase):
    def test_nearest_line_is_selected_instead_of_midpoint(self):
        mask = np.zeros((800, 400), dtype=np.uint8)
        paint(mask, 8.)
        paint(mask, 24., thickness=.5)
        candidate = select_stopline(mask)
        self.assertAlmostEqual(candidate.distance_m, 8., delta=.05)
        self.assertGreater(candidate.confidence, .5)
        self.assertLess(candidate.confidence, 1.)

    def test_disconnected_side_markings_do_not_form_a_stopline(self):
        mask = np.zeros((800, 400), dtype=np.uint8)
        paint(mask, 8., width=1.5, lateral=1.25)
        paint(mask, 16., width=1.5, lateral=-1.25)
        self.assertIsNone(select_stopline(mask))

    def test_large_centre_hole_is_rejected(self):
        mask = np.zeros((800, 400), dtype=np.uint8)
        paint(mask, 8.)
        mask[:, 188:212] = 0
        self.assertIsNone(select_stopline(mask))

    def test_small_segmentation_hole_is_tolerated(self):
        mask = np.zeros((800, 400), dtype=np.uint8)
        paint(mask, 8.)
        mask[:, 199:201] = 0
        self.assertAlmostEqual(select_stopline(mask).distance_m, 8., delta=.05)

    def test_narrow_longitudinal_and_thick_false_markings_are_rejected(self):
        for width, thickness, slope in ((.4, 8., 0.), (3., 2., 0.), (3., .2, 1.2)):
            mask = np.zeros((800, 400), dtype=np.uint8)
            paint(mask, 10., width=width, thickness=thickness, slope=slope)
            self.assertIsNone(select_stopline(mask))

    def test_oblique_line_uses_ego_centre_intersection(self):
        mask = np.zeros((800, 400), dtype=np.uint8)
        paint(mask, 12., width=3.4, lateral=.3, slope=.4)
        candidate = select_stopline(mask)
        self.assertAlmostEqual(candidate.distance_m, 12., delta=.05)
        self.assertEqual(candidate.lateral_m, 0.)

    def test_side_road_and_small_near_noise_do_not_hide_valid_line(self):
        mask = np.zeros((800, 400), dtype=np.uint8)
        paint(mask, 4., width=2., lateral=4.)
        paint(mask, 6., width=.3, thickness=.1)
        paint(mask, 18.)
        self.assertAlmostEqual(select_stopline(mask).distance_m, 18., delta=.05)

    def test_empty_invalid_and_non_stop_classes(self):
        for mask in ([], [1, 2], np.zeros((800, 400)), np.full((800, 400), 1)):
            self.assertIsNone(select_stopline(mask))
        with self.assertRaises(ValueError):
            select_stopline(np.zeros((4, 4)), resolution_m=0.)

    def test_detection_to_red_stop_dropout_hold_and_green_release(self):
        mask = np.zeros((800, 400), dtype=np.uint8)
        paint(mask, 8.)
        paint(mask, 20.)
        candidate = select_stopline(mask)
        core = StopLineControllerCore(front_reference_offset_m=3.845)

        def signal(names, t):
            state, confidence = straight_observation([
                SimpleNamespace(class_name=name, conf=.9) for name in names])
            core.observe_signal(state, confidence, state != "UNKNOWN", t, t, t)

        signal(["Red"], 10.)
        core.observe_line(candidate.distance_m, candidate.confidence, True, 10., 10., 10.)
        speed, travel, step = 2., 0., .05
        decision = core.update(10., 10., speed)
        for i in range(1, 161):
            # Ideal calibrated plant, not a MORAI dynamics/stop-distance claim.
            next_speed = max(0., speed + (decision.accel_limit - 1.5 * decision.brake) * step)
            travel += .5 * (speed + next_speed) * step
            speed = next_speed
            t = 10. + i * step
            signal(["Red", "Green"] if i % 5 == 0 else [], t)
            decision = core.update(t, t, speed)
            if decision.mode == "HOLD" and speed == 0:
                break
        self.assertEqual(decision.mode, "HOLD")
        self.assertEqual(decision.brake, 1.)
        self.assertLess(travel + 3.845, 8.)
        signal(["Green"], t + .1)
        self.assertEqual(core.update(t + .1, t + .1, 0.).brake, 1.)
        signal(["Green"], t + .45)
        self.assertEqual(core.update(t + .45, t + .45, 0.).mode, "NOMINAL")


if __name__ == "__main__":
    unittest.main()
