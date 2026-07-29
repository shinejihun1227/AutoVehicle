#!/usr/bin/env python3
"""Small Front-camera perception prototype for ROI integration.

This is intentionally conservative: traffic-light + stop-line output can
request a brake override.  Lane steering correction is exposed but disabled
by default until it is calibrated against the competition route.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

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


# Normalized points for the initial Front-camera BEV. These are deliberately
# kept configurable because the exact road/camera calibration must be checked
# against the actual MORAI route.
DEFAULT_BEV_SOURCE_POINTS = (
    (0.30, 0.55),
    (0.70, 0.55),
    (0.96, 1.00),
    (0.04, 1.00),
)
DEFAULT_BEV_DESTINATION_POINTS = (
    (0.25, 0.00),
    (0.75, 0.00),
    (0.75, 1.00),
    (0.25, 1.00),
)


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
        bev_source_points: Optional[Sequence[Tuple[float, float]]] = None,
        bev_destination_points: Optional[Sequence[Tuple[float, float]]] = None,
        sobel_x_threshold: int = 45,
        sobel_y_threshold: int = 45,
        ransac_iterations: int = 80,
    ) -> None:
        self.resize_width = int(resize_width)
        self.process_period = 0.0 if process_rate_hz <= 0.0 else 1.0 / process_rate_hz
        self.min_traffic_pixels = int(min_traffic_pixels)
        self.min_stop_line_pixels = int(min_stop_line_pixels)
        self.min_lane_pixels = int(min_lane_pixels)
        self.min_lane_side_pixels = int(min_lane_side_pixels)
        self.lane_smoothing_alpha = max(0.0, min(1.0, float(lane_smoothing_alpha)))
        self.bev_source_points = tuple(
            bev_source_points or DEFAULT_BEV_SOURCE_POINTS
        )
        self.bev_destination_points = tuple(
            bev_destination_points or DEFAULT_BEV_DESTINATION_POINTS
        )
        self.sobel_x_threshold = int(sobel_x_threshold)
        self.sobel_y_threshold = int(sobel_y_threshold)
        self.ransac_iterations = max(10, int(ransac_iterations))
        self._last_process_time = 0.0
        self._last_lane_mask = None
        self._last_lane_center_x = None
        self._last_left_fit = None
        self._last_right_fit = None
        self._last_left_lane_x = None
        self._last_right_lane_x = None
        self._last_lane_confidence = 0.0
        self._smoothed_lane_offset = None
        self._last_bev_image = None
        self._last_color_mask = None
        self._last_edge_mask = None
        self._last_stop_line_segments = []
        self._last_stop_line_detected = False
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
        lane_offset = self._lane_offset(image)
        stop_line = self._last_stop_line_detected
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

        debug_image = self._last_bev_image if self._last_bev_image is not None else image
        overlay = debug_image.copy()
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
        for x1, y1, x2, y2 in self._last_stop_line_segments:
            cv2.line(overlay, (x1, y1), (x2, y2), (255, 255, 0), 3)

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
            "BEV traffic={} score={} stop_line={}".format(
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
            "yellow=pixels blue=left orange=right red=center green=lane cyan=stop",
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
        points = []
        for normalized_y in np.linspace(0.0, 1.0, 24):
            y = int(round(normalized_y * (height - 1)))
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

    def _preprocess_for_lane(self, image):
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        channels = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        channels = (clahe.apply(channels[0]), channels[1], channels[2])
        enhanced = cv2.cvtColor(cv2.merge(channels), cv2.COLOR_LAB2BGR)
        return cv2.GaussianBlur(enhanced, (5, 5), 0)

    def _warp_to_bev(self, image):
        height, width = image.shape[:2]

        def to_pixels(points):
            return np.asarray(
                [(float(x) * width, float(y) * height) for x, y in points],
                dtype=np.float32,
            )

        source = to_pixels(self.bev_source_points)
        destination = to_pixels(self.bev_destination_points)
        matrix = cv2.getPerspectiveTransform(source, destination)
        return cv2.warpPerspective(image, matrix, (width, height))

    def _feature_masks(self, bev):
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(
            hsv, np.array([0, 0, 170]), np.array([180, 90, 255])
        )
        yellow = cv2.inRange(
            hsv, np.array([12, 55, 70]), np.array([45, 255, 255])
        )
        color_mask = cv2.bitwise_or(white, yellow)

        gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        sobel_x = cv2.convertScaleAbs(cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3))
        sobel_y = cv2.convertScaleAbs(cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3))
        edge_x = cv2.inRange(sobel_x, self.sobel_x_threshold, 255)
        edge_y = cv2.inRange(sobel_y, self.sobel_y_threshold, 255)
        edge_mask = cv2.bitwise_or(edge_x, edge_y)
        edge_support = cv2.dilate(edge_mask, np.ones((3, 3), np.uint8))

        # Require color and an X/Y edge nearby. The OR inside edge_support
        # keeps slightly blurred lane paint from disappearing completely.
        lane_mask = cv2.bitwise_and(color_mask, edge_support)
        lane_mask = cv2.morphologyEx(
            lane_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)
        )
        lane_mask = cv2.morphologyEx(
            lane_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
        )
        return color_mask, edge_mask, lane_mask

    def _stop_line(self, bev):
        height, width = bev.shape[:2]
        gray = cv2.cvtColor(bev, cv2.COLOR_BGR2GRAY)
        sobel_y = cv2.convertScaleAbs(cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3))
        edge = cv2.inRange(sobel_y, self.sobel_y_threshold, 255)
        roi = np.zeros_like(edge)
        roi[int(height * 0.50):, :] = edge[int(height * 0.50):, :]
        lines = cv2.HoughLinesP(
            roi,
            1.0,
            np.pi / 180.0,
            threshold=max(20, int(width * 0.08)),
            minLineLength=max(30, int(width * 0.25)),
            maxLineGap=max(10, int(width * 0.05)),
        )
        segments = []
        if lines is not None:
            # OpenCV versions return either (N, 1, 4) or (N, 4).
            for line in np.asarray(lines).reshape(-1, 4):
                x1, y1, x2, y2 = [int(value) for value in line]
                angle = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1)))
                if angle <= 10.0 and (y1 + y2) * 0.5 >= height * 0.50:
                    segments.append((x1, y1, x2, y2))
        self._last_stop_line_segments = segments
        self._last_stop_line_detected = len(segments) > 0
        return self._last_stop_line_detected

    def _lane_offset(self, image) -> Optional[float]:
        preprocessed = self._preprocess_for_lane(image)
        bev = self._warp_to_bev(preprocessed)
        color_mask, edge_mask, lane_mask = self._feature_masks(bev)
        self._last_bev_image = bev
        self._last_color_mask = color_mask
        self._last_edge_mask = edge_mask
        self._last_lane_mask = lane_mask
        self._stop_line(bev)

        height, width = lane_mask.shape[:2]
        _ys, xs = np.where(lane_mask > 0)
        self._last_lane_center_x = None
        self._last_left_fit = None
        self._last_right_fit = None
        self._last_left_lane_x = None
        self._last_right_lane_x = None
        self._last_lane_confidence = 0.0
        if len(xs) < self.min_lane_pixels:
            return None

        left_ys, left_xs, right_ys, right_xs = self._sliding_window_points(lane_mask)
        left_fit = self._ransac_lane_curve(left_ys, left_xs, height, width)
        right_fit = self._ransac_lane_curve(right_ys, right_xs, height, width)

        left_quality = right_quality = 0.0
        if left_fit is not None:
            self._last_left_fit, self._last_left_lane_x, left_quality = left_fit
        if right_fit is not None:
            self._last_right_fit, self._last_right_lane_x, right_quality = right_fit

        lane_center = None
        confidence = 0.0
        if self._last_left_lane_x is not None and self._last_right_lane_x is not None:
            lane_width = self._last_right_lane_x - self._last_left_lane_x
            if width * 0.20 <= lane_width <= width * 1.25:
                lane_center = (self._last_left_lane_x + self._last_right_lane_x) * 0.5
                confidence = min(left_quality, right_quality)
        elif self._last_left_lane_x is not None or self._last_right_lane_x is not None:
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

    def _sliding_window_points(self, mask):
        height, width = mask.shape[:2]
        nonzero_y, nonzero_x = np.nonzero(mask > 0)
        histogram = np.sum(mask[int(height * 0.55):, :], axis=0)
        midpoint = width // 2
        left_base = int(np.argmax(histogram[:midpoint])) if midpoint else 0
        right_half = histogram[midpoint:]
        right_base = midpoint + int(np.argmax(right_half)) if len(right_half) else midpoint
        margin = max(25, int(width * 0.10))
        minpix = max(20, self.min_lane_side_pixels // 2)
        left_indices = []
        right_indices = []
        left_current = left_base
        right_current = right_base
        window_height = max(1, height // 9)

        for window in range(9):
            y_low = height - (window + 1) * window_height
            y_high = height - window * window_height
            left_match = (
                (nonzero_y >= y_low)
                & (nonzero_y < y_high)
                & (nonzero_x >= left_current - margin)
                & (nonzero_x < left_current + margin)
            )
            right_match = (
                (nonzero_y >= y_low)
                & (nonzero_y < y_high)
                & (nonzero_x >= right_current - margin)
                & (nonzero_x < right_current + margin)
            )
            left_indices.append(np.where(left_match)[0])
            right_indices.append(np.where(right_match)[0])
            if int(np.count_nonzero(left_match)) > minpix:
                left_current = int(np.mean(nonzero_x[left_match]))
            if int(np.count_nonzero(right_match)) > minpix:
                right_current = int(np.mean(nonzero_x[right_match]))

        left_indices = np.concatenate(left_indices) if left_indices else np.array([], dtype=int)
        right_indices = np.concatenate(right_indices) if right_indices else np.array([], dtype=int)
        return (
            nonzero_y[left_indices],
            nonzero_x[left_indices],
            nonzero_y[right_indices],
            nonzero_x[right_indices],
        )

    def _ransac_lane_curve(self, ys, xs, height: int, width: int):
        if len(xs) < self.min_lane_side_pixels:
            return None
        if np.ptp(ys) < height * 0.15:
            return None

        normalized_y = ys.astype(np.float32) / max(1.0, height - 1)
        rng = np.random.RandomState(7)
        best_inliers = None
        best_score = -1.0
        for _ in range(self.ransac_iterations):
            sample = rng.choice(len(xs), size=3, replace=False)
            try:
                coefficients = np.polyfit(normalized_y[sample], xs[sample], 2)
            except (np.linalg.LinAlgError, ValueError):
                continue
            residual = np.abs(xs - np.polyval(coefficients, normalized_y))
            inliers = residual <= 8.0
            count = int(np.count_nonzero(inliers))
            if count < self.min_lane_side_pixels:
                continue
            score = float(count) - float(np.median(residual[inliers])) * 0.25
            if score > best_score:
                best_score = score
                best_inliers = inliers

        if best_inliers is None:
            return None
        coefficients = np.polyfit(
            normalized_y[best_inliers], xs[best_inliers].astype(np.float32), 2
        )
        residual = np.abs(xs[best_inliers] - np.polyval(coefficients, normalized_y[best_inliers]))
        fit_quality = max(0.0, 1.0 - min(20.0, float(np.median(residual))) / 20.0)
        count_quality = min(1.0, int(np.count_nonzero(best_inliers)) / float(self.min_lane_side_pixels * 4))
        x_at_eval = float(np.polyval(coefficients, 0.85))
        if not 0.0 <= x_at_eval <= float(width - 1):
            return None
        return coefficients, x_at_eval, fit_quality * count_quality
