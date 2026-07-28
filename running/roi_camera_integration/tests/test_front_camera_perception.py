import unittest

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - depends on the target Ubuntu image
    cv2 = None
    np = None

from roi_camera_integration.front_camera_perception import FrontCameraPerception


@unittest.skipIf(cv2 is None or np is None, "OpenCV/numpy are required for image tests")
class FrontCameraPerceptionTests(unittest.TestCase):
    @staticmethod
    def _image(shift_px=0):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.line(image, (160 + shift_px, 479), (280 + shift_px, 280), (255, 255, 255), 8)
        cv2.line(image, (480 + shift_px, 479), (360 + shift_px, 280), (255, 255, 255), 8)
        return image

    def test_centered_lane_has_near_zero_pixel_offset(self):
        perception = FrontCameraPerception(min_lane_pixels=100)
        offset = perception._lane_offset(self._image())

        self.assertIsNotNone(offset)
        self.assertAlmostEqual(offset, 0.0, delta=15.0)

    def test_shifted_lane_preserves_offset_sign(self):
        perception = FrontCameraPerception(min_lane_pixels=100)
        offset = perception._lane_offset(self._image(shift_px=40))

        self.assertIsNotNone(offset)
        self.assertGreater(offset, 20.0)

    def test_empty_road_returns_no_lane(self):
        perception = FrontCameraPerception(min_lane_pixels=100)
        offset = perception._lane_offset(np.zeros((480, 640, 3), dtype=np.uint8))

        self.assertIsNone(offset)


if __name__ == "__main__":
    unittest.main()
