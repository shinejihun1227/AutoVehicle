import math
import unittest

from roi_camera_integration.lane_fusion import (
    FusedLaneEstimate,
    LaneObservation,
    MultiCameraLaneFusion,
    calculate_lane_steering,
)


class MultiCameraLaneFusionTests(unittest.TestCase):
    def test_fuses_fresh_confident_cameras(self):
        fusion = MultiCameraLaneFusion(
            source_weights={"front": 1.0, "left": 1.2, "right": 1.2}
        )
        estimate = fusion.fuse(
            [
                LaneObservation("front", 10.0, 0.10, 0.04, 0.9),
                LaneObservation("left", 10.05, 0.16, 0.02, 0.8),
                LaneObservation("right", 10.05, 0.12, 0.03, 0.8),
            ],
            now=10.10,
        )

        self.assertTrue(estimate.valid)
        self.assertEqual(estimate.source_count, 3)
        self.assertEqual(estimate.sources, ("front", "left", "right"))
        self.assertAlmostEqual(estimate.lateral_error_m, 0.127, delta=0.02)
        self.assertGreater(estimate.confidence, 0.7)

    def test_stale_and_low_confidence_observations_are_ignored(self):
        fusion = MultiCameraLaneFusion(max_age_sec=0.2, min_confidence=0.5)
        estimate = fusion.fuse(
            [
                LaneObservation("front", 9.0, 0.2, 0.0, 0.95),
                LaneObservation("left", 10.0, 0.4, 0.0, 0.2),
            ],
            now=10.0,
        )

        self.assertFalse(estimate.valid)
        self.assertEqual(estimate.source_count, 0)
        self.assertEqual(estimate.confidence, 0.0)

    def test_lateral_outlier_is_rejected(self):
        fusion = MultiCameraLaneFusion(outlier_threshold_m=0.5)
        estimate = fusion.fuse(
            [
                LaneObservation("front", 10.0, 0.10, 0.0, 0.9),
                LaneObservation("left", 10.0, 0.15, 0.0, 0.9),
                LaneObservation("right", 10.0, 4.0, 0.0, 0.9),
            ],
            now=10.0,
        )

        self.assertTrue(estimate.valid)
        self.assertEqual(estimate.source_count, 2)
        self.assertNotIn("right", estimate.sources)
        self.assertLess(estimate.lateral_error_m, 0.2)

    def test_heading_fusion_wraps_at_pi(self):
        fusion = MultiCameraLaneFusion()
        estimate = fusion.fuse(
            [
                LaneObservation("front", 10.0, 0.0, math.pi - 0.02, 0.9),
                LaneObservation("left", 10.0, 0.0, -math.pi + 0.02, 0.9),
            ],
            now=10.0,
        )

        self.assertTrue(estimate.valid)
        self.assertGreater(abs(estimate.heading_error_rad), 3.0)

    def test_lane_steering_is_bounded_and_zero_for_invalid_estimate(self):
        valid = FusedLaneEstimate(
            True, 10.0, 2.0, math.radians(20.0), 0.9, 1, ("front",)
        )
        correction = calculate_lane_steering(
            valid,
            speed_mps=5.0,
            max_correction_rad=math.radians(3.0),
        )
        self.assertAlmostEqual(correction, math.radians(3.0))

        invalid = FusedLaneEstimate(False, None, 2.0, 0.5, 0.0, 0, ())
        self.assertEqual(calculate_lane_steering(invalid, speed_mps=5.0), 0.0)


if __name__ == "__main__":
    unittest.main()
