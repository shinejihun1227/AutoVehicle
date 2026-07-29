import math
import unittest

try:
    import numpy as np
except ImportError:  # pragma: no cover - depends on target Ubuntu image
    np = None

from roi_camera_integration.path_overlay import (
    CameraIntrinsics,
    CameraMount,
    points_to_xyz,
    project_path_to_image,
    project_vehicle_points,
    world_to_vehicle,
)


@unittest.skipIf(np is None, "numpy is required for path overlay tests")
class PathOverlayTests(unittest.TestCase):
    def test_vehicle_transform_keeps_forward_point_in_front(self):
        points = np.asarray([[10.0, 0.0, 0.0], [10.0, 2.0, 0.0]])
        vehicle = world_to_vehicle(points, 0.0, 0.0, 0.0, 0.0)
        np.testing.assert_allclose(vehicle[:, :2], points[:, :2])

    def test_vehicle_yaw_rotates_global_path_into_forward_axis(self):
        points = np.asarray([[0.0, 10.0, 0.0]])
        vehicle = world_to_vehicle(points, 0.0, 0.0, 0.0, math.pi / 2.0)
        np.testing.assert_allclose(vehicle[0], [10.0, 0.0, 0.0], atol=1.0e-9)

    def test_front_point_projects_to_image_center(self):
        camera = CameraMount(x_m=1.9, y_m=0.0, z_m=1.2, pitch_down_deg=0.0)
        intrinsics = CameraIntrinsics(640, 360, horizontal_fov_deg=90.0)
        pixels, depth = project_vehicle_points(
            np.asarray([[11.9, 0.0, 1.2]]), camera, intrinsics
        )
        np.testing.assert_allclose(pixels[0], [320.0, 180.0], atol=1.0e-9)
        self.assertAlmostEqual(depth[0], 10.0)

    def test_path_projection_filters_behind_and_out_of_view_points(self):
        camera = CameraMount(x_m=1.9, y_m=0.0, z_m=1.2, pitch_down_deg=0.0)
        intrinsics = CameraIntrinsics(640, 360, horizontal_fov_deg=90.0)
        path = np.asarray(
            [
                [0.0, 0.0, 0.0],
                [11.9, 0.0, 0.0],
                [21.9, 0.0, 0.0],
                [21.9, 100.0, 0.0],
            ]
        )
        _pixels, valid, _vehicle = project_path_to_image(
            path,
            0.0,
            0.0,
            0.0,
            0.0,
            camera,
            intrinsics,
            min_forward_m=0.5,
            max_forward_m=30.0,
            max_lateral_m=35.0,
        )
        self.assertEqual(valid.tolist(), [False, True, True, False])

    def test_waypoint_adapter_accepts_x_y_objects_without_z(self):
        class Waypoint:
            x = 1.0
            y = 2.0

        points = points_to_xyz([Waypoint()], default_z=7.5)
        np.testing.assert_allclose(points, [[1.0, 2.0, 7.5]])


if __name__ == "__main__":
    unittest.main()
