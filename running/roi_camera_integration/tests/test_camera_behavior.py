import unittest

from roi_camera_integration.camera_behavior import (
    CameraControlPolicy,
    FrontCameraBehavior,
)
from roi_camera_integration.front_camera_perception import FrontCameraObservation


def _observation(
    stamp=10.0,
    traffic_state="unknown",
    stop_line_detected=False,
    lane_offset_px=None,
):
    return FrontCameraObservation(
        monotonic_time=stamp,
        width=640,
        height=360,
        traffic_state=traffic_state,
        traffic_score=0,
        stop_line_detected=stop_line_detected,
        lane_offset_px=lane_offset_px,
    )


class FrontCameraBehaviorTests(unittest.TestCase):
    def test_lane_correction_is_disabled_by_default(self):
        behavior = FrontCameraBehavior()
        behavior.update(_observation(lane_offset_px=200.0))

        output = behavior.apply(0.2, 0.0, 0.15, now_monotonic=10.1)

        self.assertEqual(output, (0.2, 0.0, 0.15))

    def test_lane_correction_has_expected_sign_and_limit(self):
        policy = CameraControlPolicy(
            lane_steer_gain=0.5,
            max_lane_steer_correction=0.1,
        )
        behavior = FrontCameraBehavior(policy)
        behavior.update(_observation(lane_offset_px=320.0))

        output = behavior.apply(0.2, 0.0, 0.2, now_monotonic=10.1)

        # Positive pixel offset means the detected lane center is to the
        # vehicle's right; the correction therefore turns left (negative here
        # because MORAI's normalized steering sign is configurable).
        self.assertEqual(output[:2], (0.2, 0.0))
        self.assertAlmostEqual(output[2], 0.1)

    def test_red_light_requires_stop_line_by_default(self):
        behavior = FrontCameraBehavior()
        behavior.update(_observation(traffic_state="red", stop_line_detected=False))

        self.assertEqual(
            behavior.apply(0.4, 0.0, 0.0, now_monotonic=10.1),
            (0.4, 0.0, 0.0),
        )

    def test_red_light_with_stop_line_requests_full_brake(self):
        behavior = FrontCameraBehavior()
        behavior.update(_observation(traffic_state="red", stop_line_detected=True))

        self.assertEqual(
            behavior.apply(0.4, 0.0, 0.2, now_monotonic=10.1),
            (0.0, 1.0, 0.2),
        )

    def test_stale_camera_does_not_change_controller_output(self):
        behavior = FrontCameraBehavior(
            CameraControlPolicy(camera_stale_timeout_sec=0.2, lane_steer_gain=0.5)
        )
        behavior.update(_observation(lane_offset_px=200.0))

        self.assertEqual(
            behavior.apply(0.3, 0.1, -0.2, now_monotonic=10.3),
            (0.3, 0.1, -0.2),
        )

    def test_green_light_releases_previous_stop_request(self):
        behavior = FrontCameraBehavior()
        behavior.update(_observation(traffic_state="red", stop_line_detected=True))
        behavior.update(_observation(stamp=10.1, traffic_state="green"))

        self.assertEqual(
            behavior.apply(0.4, 0.0, 0.0, now_monotonic=10.2),
            (0.4, 0.0, 0.0),
        )


if __name__ == "__main__":
    unittest.main()
