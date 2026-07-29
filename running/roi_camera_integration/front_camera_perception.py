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
    source_width: Optional[int] = None
    source_height: Optional[int] = None
    lane_confidence: float = 0.0
    left_lane_x: Optional[float] = None
    right_lane_x: Optional[float] = None


class FrontCameraPerception:
    """RGB-only prototype detector; no GT/BBox data is used."""

    def __init__(
        self,
        resize_width: int = 640,
        process_rate_hz: float = 15.0,
        min_traffic_pixels: int = 60,
        min_stop_line_pixels: int = 1500,
        min_lane_pixels: int = 400,
        min_lane_side_pixels: int = 80,
        lane_smoothing_alpha: float = 0.25,
    ) -> None:
        self.resize_width = int(resize_width)
        self.process_period = 0.0 if process_rate_hz <= 0.0 else 1.0 / process_rate_hz
        self.min_traffic_pixels = int(min_traffic_pixels)
        self.min_stop_line_pixels = int(min_stop_line_pixels)
        self.min_lane_pixels = int(min_lane_pixels)
        self.min_lane_side_pixels = int(min_lane_side_pixels)
        self.lane_smoothing_alpha = max(0.0, min(1.0, float(lane_smoothing_alpha)))
        self._last_process_time = 0.0
        self._last_lane_mask = None
        self._last_lane_center_x = None
        self._last_left_fit = None
        self._last_right_fit = None
        self._last_left_lane_x = None
        self._last_right_lane_x = None
        self._last_lane_confidence = 0.0
        self._smoothed_lane_offset = None
        self.last_debug_overlay = None

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

        source_height, source_width = image.shape[:2]
        image = self._resize(image)
        height, width = image.shape[:2]
        traffic_state, traffic_score = self._traffic_light(image)
        stop_line = self._stop_line(image)
        lane_offset = self._lane_offset(image)
        self.last_debug_overlay = self._build_debug_overlay(
            image,
            traffic_state,
            traffic_score,
            stop_line,
            lane_offset,
        )
        return FrontCameraObservation(
            monotonic_time=now,
            width=width,
            height=height,
            traffic_state=traffic_state,
            traffic_score=traffic_score,
            stop_line_detected=stop_line,
            lane_offset_px=lane_offset,
            source_width=source_width,
            source_height=source_height,
            lane_confidence=self._last_lane_confidence,
            left_lane_x=self._last_left_lane_x,
            right_lane_x=self._last_right_lane_x,
        )

    def _build_debug_overlay(
        self,
        image,
        traffic_state: str,
        traffic_score: int,
        stop_line: bool,
        lane_offset: Optional[float],
    ):
        """Render the exact lane mask and centers used by the detector."""

        overlay = image.copy()
        if self._last_lane_mask is not None:
            highlighted = np.zeros_like(image)
            # BGR yellow highlights every pixel accepted by the lane mask.
            mask_height = self._last_lane_mask.shape[0]
            mask_y = image.shape[0] - mask_height
            highlighted[mask_y:, :, 1] = self._last_lane_mask
            highlighted[mask_y:, :, 2] = self._last_lane_mask
            overlay = cv2.addWeighted(overlay, 0.78, highlighted, 0.62, 0.0)

        height, width = overlay.shape[:2]
        lane_roi_y = int(height * 0.55)
        cv2.line(overlay, (0, lane_roi_y), (width - 1, lane_roi_y), (255, 0, 255), 1)
        center_x = width // 2
        cv2.line(overlay, (center_x, 0), (center_x, height - 1), (0, 0, 255), 2)
        self._draw_lane_fit(overlay, self._last_left_fit, (255, 0, 0))
        self._draw_lane_fit(overlay, self._last_right_fit, (0, 165, 255))

        if self._last_lane_center_x is None:
            lane_text = "lane: NOT DETECTED"
            lane_color = (0, 0, 255)
        else:
            lane_x = int(round(self._last_lane_center_x))
            cv2.line(overlay, (lane_x, lane_roi_y), (lane_x, height - 1), (0, 255, 0), 3)
            lane_text = "lane_x={} offset_px={:+.1f}".format(
                lane_x,
                0.0 if lane_offset is None else lane_offset,
            )
            lane_color = (0, 255, 0)
        lane_text += " conf={:.2f}".format(self._last_lane_confidence)

        cv2.putText(
            overlay,
            lane_text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            lane_color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            "traffic={} score={} stop_line={}".format(
                traffic_state, traffic_score, stop_line
            ),
            (12, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            "yellow=pixels blue=left orange=right red=center green=lane",
            (12, height - 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return overlay

    def _draw_lane_fit(self, image, coefficients, color) -> None:
        if coefficients is None:
            return
        height, _width = image.shape[:2]
        roi_y = int(height * 0.55)
        points = []
        for normalized_y in np.linspace(0.0, 1.0, 24):
            y = int(round(roi_y + normalized_y * (height - 1 - roi_y)))
            x = int(round(np.polyval(coefficients, normalized_y)))
            if 0 <= x < image.shape[1]:
                points.append((x, y))
        if len(points) >= 2:
            cv2.polylines(image, [np.asarray(points, dtype=np.int32)], False, color, 3)

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
        self._last_lane_mask = mask
        self._last_lane_center_x = None
        self._last_left_fit = None
        self._last_right_fit = None
        self._last_left_lane_x = None
        self._last_right_lane_x = None
        self._last_lane_confidence = 0.0

        _ys, xs = np.where(mask > 0)
        if len(xs) < self.min_lane_pixels:
            return None

        full_ys = _ys + int(height * 0.55)
        left_points = xs < width * 0.5
        right_points = ~left_points
        left_fit = self._fit_lane_curve(
            full_ys[left_points], xs[left_points], height, width
        )
        right_fit = self._fit_lane_curve(
            full_ys[right_points], xs[right_points], height, width
        )

        if left_fit is not None:
            self._last_left_fit, self._last_left_lane_x, left_quality = left_fit
        else:
            left_quality = 0.0
        if right_fit is not None:
            self._last_right_fit, self._last_right_lane_x, right_quality = right_fit
        else:
            right_quality = 0.0

        lane_center = None
        confidence = 0.0
        if self._last_left_lane_x is not None and self._last_right_lane_x is not None:
            lane_width = self._last_right_lane_x - self._last_left_lane_x
            min_width = width * 0.20
            max_width = width * 1.25
            if min_width <= lane_width <= max_width:
                lane_center = (self._last_left_lane_x + self._last_right_lane_x) * 0.5
                confidence = min(left_quality, right_quality)
            else:
                # Keep the fitted lines visible, but do not report a strong
                # lane estimate when their geometry is implausible.
                confidence = 0.15
        elif self._last_left_lane_x is not None or self._last_right_lane_x is not None:
            # One boundary is still useful for diagnostics, but this fallback
            # has low confidence because a lane width cannot be verified.
            lane_center = float(np.median(xs))
            confidence = 0.25 * max(left_quality, right_quality)
        else:
            lane_center = float(np.median(xs))
            confidence = 0.10

        raw_offset = lane_center - width * 0.5
        if self._smoothed_lane_offset is None:
            self._smoothed_lane_offset = raw_offset
        else:
            alpha = self.lane_smoothing_alpha
            self._smoothed_lane_offset = (
                alpha * raw_offset + (1.0 - alpha) * self._smoothed_lane_offset
            )
        self._last_lane_confidence = max(0.0, min(1.0, confidence))
        self._last_lane_center_x = width * 0.5 + self._smoothed_lane_offset
        return self._smoothed_lane_offset

    def _fit_lane_curve(self, ys, xs, height: int, width: int):
        if len(xs) < self.min_lane_side_pixels:
            return None
        if np.ptp(ys) < height * 0.15:
            return None

        roi_y = height * 0.55
        normalized_y = (ys.astype(np.float32) - roi_y) / max(1.0, height - roi_y)
        coefficients = np.polyfit(normalized_y, xs.astype(np.float32), 2)

        # Reject isolated bright objects and horizontal stop-line pixels with
        # one robust residual pass before accepting the curve.
        for _ in range(2):
            predicted = np.polyval(coefficients, normalized_y)
            residual = np.abs(xs - predicted)
            threshold = max(6.0, min(24.0, float(np.percentile(residual, 75)) * 1.5))
            inliers = residual <= threshold
            if int(np.count_nonzero(inliers)) < self.min_lane_side_pixels:
                return None
            coefficients = np.polyfit(
                normalized_y[inliers], xs[inliers].astype(np.float32), 2
            )
            normalized_y = normalized_y[inliers]
            xs = xs[inliers]

        residual = np.abs(xs - np.polyval(coefficients, normalized_y))
        fit_quality = max(0.0, 1.0 - min(30.0, float(np.median(residual))) / 30.0)
        count_quality = min(1.0, len(xs) / float(self.min_lane_side_pixels * 4))
        x_at_eval = float(np.polyval(coefficients, 0.90))
        if not 0.0 <= x_at_eval <= float(width - 1):
            return None
        return coefficients, x_at_eval, fit_quality * count_quality
