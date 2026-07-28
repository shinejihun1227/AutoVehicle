#!/usr/bin/env python3
"""Small Front-camera perception prototype for ROI integration.

This is intentionally conservative: traffic-light + stop-line output can
request a brake override.  Lane steering correction is exposed but disabled
by default until it is calibrated against the competition route.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - exercised on the Ubuntu target
    cv2 = None
    np = None


@dataclass(frozen=True)
class FrontCameraObservation:
    monotonic_time: float
    width: int
    height: int
    traffic_state: str
    traffic_score: int
    stop_line_detected: bool
    lane_offset_px: Optional[float]


class FrontCameraPerception:
    """RGB-only prototype detector; no GT/BBox data is used."""

    def __init__(
        self,
        resize_width: int = 640,
        process_rate_hz: float = 15.0,
        min_traffic_pixels: int = 60,
        min_stop_line_pixels: int = 1500,
        min_lane_pixels: int = 400,
    ) -> None:
        self.resize_width = int(resize_width)
        self.process_period = 0.0 if process_rate_hz <= 0.0 else 1.0 / process_rate_hz
        self.min_traffic_pixels = int(min_traffic_pixels)
        self.min_stop_line_pixels = int(min_stop_line_pixels)
        self.min_lane_pixels = int(min_lane_pixels)
        self._last_process_time = 0.0

    def process_jpeg(
        self, jpeg: bytes, monotonic_time: Optional[float] = None
    ) -> Optional[FrontCameraObservation]:
        if cv2 is None or np is None:
            raise RuntimeError("FrontCameraPerception requires python3-opencv and numpy")

        now = time.monotonic() if monotonic_time is None else float(monotonic_time)
        if self.process_period > 0.0 and now - self._last_process_time < self.process_period:
            return None
        self._last_process_time = now

        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return None

        image = self._resize(image)
        height, width = image.shape[:2]
        traffic_state, traffic_score = self._traffic_light(image)
        stop_line = self._stop_line(image)
        lane_offset = self._lane_offset(image)
        return FrontCameraObservation(
            monotonic_time=now,
            width=width,
            height=height,
            traffic_state=traffic_state,
            traffic_score=traffic_score,
            stop_line_detected=stop_line,
            lane_offset_px=lane_offset,
        )

    def _resize(self, image):
        if self.resize_width <= 0 or image.shape[1] == self.resize_width:
            return image
        scale = float(self.resize_width) / float(image.shape[1])
        height = max(1, int(image.shape[0] * scale))
        return cv2.resize(image, (self.resize_width, height), interpolation=cv2.INTER_AREA)

    def _traffic_light(self, image):
        height = image.shape[0]
        roi = image[: int(height * 0.45), :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        red_a = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        red_b = cv2.inRange(hsv, np.array([170, 100, 100]), np.array([180, 255, 255]))
        counts = {
            "red": int(cv2.countNonZero(cv2.bitwise_or(red_a, red_b))),
            "yellow": int(cv2.countNonZero(cv2.inRange(
                hsv, np.array([18, 90, 120]), np.array([38, 255, 255])
            ))),
            "green": int(cv2.countNonZero(cv2.inRange(
                hsv, np.array([45, 70, 80]), np.array([90, 255, 255])
            ))),
        }
        state, score = max(counts.items(), key=lambda item: item[1])
        if score < self.min_traffic_pixels:
            return "unknown", score
        return state, score

    def _stop_line(self, image) -> bool:
        height = image.shape[0]
        roi = image[int(height * 0.65):, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, np.array([0, 0, 190]), np.array([180, 60, 255]))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 3))
        horizontal = cv2.morphologyEx(white, cv2.MORPH_OPEN, kernel)
        return int(cv2.countNonZero(horizontal)) >= self.min_stop_line_pixels

    def _lane_offset(self, image) -> Optional[float]:
        height, width = image.shape[:2]
        roi = image[int(height * 0.55):, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, np.array([0, 0, 180]), np.array([180, 70, 255]))
        yellow = cv2.inRange(hsv, np.array([15, 60, 80]), np.array([40, 255, 255]))
        mask = cv2.bitwise_or(white, yellow)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        _ys, xs = np.where(mask > 0)
        if len(xs) < self.min_lane_pixels:
            return None

        left_x = xs[xs < width * 0.5]
        right_x = xs[xs >= width * 0.5]
        if len(left_x) >= self.min_lane_pixels * 0.25 and len(right_x) >= self.min_lane_pixels * 0.25:
            lane_center = (float(np.median(left_x)) + float(np.median(right_x))) * 0.5
        else:
            lane_center = float(np.median(xs))
        return lane_center - width * 0.5

