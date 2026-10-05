import math
import unittest

from purepursuit_mgeo.landmark_capture import project_stopline, quaternion_yaw
from purepursuit_mgeo.route_geometry import RoutePolyline


class LandmarkCaptureTests(unittest.TestCase):
    def test_projects_forward_camera_range_onto_straight_route(self):
        route = RoutePolyline([(0.0, 0.0), (100.0, 0.0)])
        result = project_stopline(route, 10.0, 0.0, 0.0, 6.0, 10.0)
        self.assertAlmostEqual(result["map_xy"][0], 16.0)
        self.assertAlmostEqual(result["map_xy"][1], 0.0)
        self.assertAlmostEqual(result["route_s_m"], 16.0)
        self.assertAlmostEqual(result["route_offset_m"], 0.0)

    def test_projects_with_vehicle_yaw_and_normalizes_quaternion(self):
        route = RoutePolyline([(0.0, 0.0), (0.0, 100.0)])
        yaw = quaternion_yaw(0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
        result = project_stopline(route, 0.0, 10.0, yaw, 5.0, 10.0)
        self.assertAlmostEqual(result["map_xy"][0], 0.0, places=6)
        self.assertAlmostEqual(result["map_xy"][1], 15.0, places=6)
        self.assertAlmostEqual(result["route_s_m"], 15.0)

    def test_rejects_invalid_pose_and_zero_quaternion(self):
        route = RoutePolyline([(0.0, 0.0), (100.0, 0.0)])
        with self.assertRaises(ValueError):
            project_stopline(route, 0.0, 0.0, float("nan"), 2.0, 0.0)
        with self.assertRaises(ValueError):
            quaternion_yaw(0.0, 0.0, 0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
