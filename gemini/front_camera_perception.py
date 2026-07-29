#!/usr/bin/env python3
"""Improved Front-camera perception module with BEV, Sobel, Hough, and RANSAC."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    import cv2
    import numpy as np
    from sklearn.linear_model import RANSACRegressor
    from sklearn.preprocessing import PolynomialFeatures
except ImportError:
    cv2 = None
    np = None
    RANSACRegressor = None
    PolynomialFeatures = None


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


class FrontCameraPerception:
    """Enhanced Perception using CLAHE, BEV, Sobel, Hough, and RANSAC."""

    def __init__(
        self,
        resize_width: int = 640,
        process_rate_hz: float = 15.0,
        min_traffic_pixels: int = 60,
    ) -> None:
        self.resize_width = int(resize_width)
        self.process_period = 0.0 if process_rate_hz <= 0.0 else 1.0 / process_rate_hz
        self.min_traffic_pixels = int(min_traffic_pixels)
        self._last_process_time = 0.0
        self._last_lane_mask = None
        self._last_lane_center_x = None
        self.last_debug_overlay = None

    def process_jpeg(
        self, jpeg: bytes, monotonic_time: Optional[float] = None
    ) -> Optional[FrontCameraObservation]:
        if cv2 is None or np is None or RANSACRegressor is None:
            raise RuntimeError("FrontCameraPerception requires opencv-python, numpy, and scikit-learn")

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

        # 1. CLAHE 전처리
        enhanced_img = self._apply_clahe(image)

        # 2. 신호등, 정지선, 차선 오프셋 인지 연산 실행
        traffic_state, traffic_score = self._traffic_light(enhanced_img)
        stop_line = self._stop_line_hough(enhanced_img)
        lane_offset = self._lane_offset_ransac(enhanced_img)

        # 3. 디버그 오버레이 생성
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
        )

    def _resize(self, image):
        if self.resize_width <= 0 or image.shape[1] == self.resize_width:
            return image
        scale = float(self.resize_width) / float(image.shape[1])
        height = max(1, int(image.shape[0] * scale))
        return cv2.resize(image, (self.resize_width, height), interpolation=cv2.INTER_AREA)

    def _apply_clahe(self, image):
        """CLAHE 명도 보정 + 가우시안 블러 전처리"""
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        enhanced = cv2.merge((l, a, b))
        enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        return cv2.GaussianBlur(enhanced, (3, 3), 0)

    def _apply_bev(self, image):
        """BEV (Bird's-Eye View) 조감도 시점 변환"""
        h, w = image.shape[:2]
        # 도로 원근 사다리꼴 (카메라 각도에 따라 조정 필요)
        src_pts = np.float32([
            [w * 0.25, h * 0.55],
            [w * 0.75, h * 0.55],
            [w * 0.95, h * 0.95],
            [w * 0.05, h * 0.95]
        ])
        dst_pts = np.float32([
            [w * 0.2, 0],
            [w * 0.8, 0],
            [w * 0.8, h],
            [w * 0.2, h]
        ])
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        return cv2.warpPerspective(image, M, (w, h))

    def _traffic_light(self, image):
        """신호등 검출 (상단 ROI 색상 필터링)"""
        height = image.shape[0]
        roi = image[: int(height * 0.45), :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        red_a = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        red_b = cv2.inRange(hsv, np.array([170, 100, 100]), np.array([180, 255, 255]))
        counts = {
            "red": int(cv2.countNonZero(cv2.bitwise_or(red_a, red_b))),
            "yellow": int(cv2.countNonZero(cv2.inRange(hsv, np.array([18, 90, 120]), np.array([38, 255, 255])))),
            "green": int(cv2.countNonZero(cv2.inRange(hsv, np.array([45, 70, 80]), np.array([90, 255, 255])))),
        }
        state, score = max(counts.items(), key=lambda item: item[1])
        if score < self.min_traffic_pixels:
            return "unknown", score
        return state, score

    def _stop_line_hough(self, image) -> bool:
        """HoughLinesP + Sobel Y를 활용한 수평 정지선 검출"""
        height, width = image.shape[:2]
        roi = image[int(height * 0.65):, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Sobel Y 필터로 가로 테두리 강조
        sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        sobel_y = cv2.convertScaleAbs(sobel_y)
        _, thresh = cv2.threshold(sobel_y, 60, 255, cv2.THRESH_BINARY)

        # 확률적 허프 변환으로 수평 직선 찾기
        lines = cv2.HoughLinesP(thresh, 1, np.pi / 180, threshold=40, minLineLength=60, maxLineGap=15)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # 기울기 각도 계산 (수평 -10도 ~ +10도 이내면 정지선 판정)
                angle = np.abs(np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi)
                if angle < 10.0 or angle > 170.0:
                    return True
        return False

    def _lane_offset_ransac(self, image) -> Optional[float]:
        """BEV + Sobel X + RANSAC 피팅 기반 차선 오프셋 계산"""
        bev_img = self._apply_bev(image)
        height, width = bev_img.shape[:2]
        gray = cv2.cvtColor(bev_img, cv2.COLOR_BGR2GRAY)

        # 1. Sobel X 필터로 세로 방향 차선 테두리 추출
        sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        sobel_x = cv2.convertScaleAbs(sobel_x)
        _, edge_mask = cv2.threshold(sobel_x, 50, 255, cv2.THRESH_BINARY)

        # 2. 색상 마스크 (흰색/노란색) 결합
        hsv = cv2.cvtColor(bev_img, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(hsv, np.array([0, 0, 180]), np.array([180, 70, 255]))
        yellow_mask = cv2.inRange(hsv, np.array([15, 60, 80]), np.array([40, 255, 255]))
        color_mask = cv2.bitwise_or(white_mask, yellow_mask)

        # Edge와 색상 마스크의 Bitwise AND
        combined_mask = cv2.bitwise_and(edge_mask, color_mask)
        self._last_lane_mask = combined_mask

        # 픽셀 좌표 가져오기
        ys, xs = np.where(combined_mask > 0)
        if len(xs) < 150:
            self._last_lane_center_x = None
            return None

        # 3. RANSAC 기반 피팅 (x = my + b)
        poly = PolynomialFeatures(degree=1)  # 1차 직선 피팅 (직선도로/안정성 극대화)
        ys_poly = poly.fit_transform(ys.reshape(-1, 1))

        try:
            ransac = RANSACRegressor(residual_threshold=10.0, max_trials=100)
            ransac.fit(ys_poly, xs)

            # 차량 바로 앞(화면 맨 밑 y=height)에서의 차선 중심 x 좌표 예측
            bottom_y = np.array([[height]])
            bottom_y_poly = poly.transform(bottom_y)
            lane_center_x = float(ransac.predict(bottom_y_poly)[0])

            self._last_lane_center_x = lane_center_x
            return lane_center_x - (width * 0.5)
        except Exception:
            self._last_lane_center_x = None
            return None

    def _build_debug_overlay(self, image, traffic_state, traffic_score, stop_line, lane_offset):
        overlay = image.copy()
        height, width = overlay.shape[:2]

        # 차선 마스크 노란색 표시
        if self._last_lane_mask is not None:
            mask_resized = cv2.resize(self._last_lane_mask, (width, height))
            highlighted = np.zeros_like(image)
            highlighted[:, :, 1] = mask_resized
            highlighted[:, :, 2] = mask_resized
            overlay = cv2.addWeighted(overlay, 0.8, highlighted, 0.5, 0.0)

        # 이미지 중심선 (빨간색)
        center_x = width // 2
        cv2.line(overlay, (center_x, 0), (center_x, height - 1), (0, 0, 255), 2)

        # 추정 차선 중심선 (초록색)
        if self._last_lane_center_x is not None:
            lane_x = int(round(self._last_lane_center_x))
            cv2.line(overlay, (lane_x, int(height * 0.5)), (lane_x, height - 1), (0, 255, 0), 3)
            lane_text = f"lane_x={lane_x} offset_px={lane_offset:+.1f}"
            lane_color = (0, 255, 0)
        else:
            lane_text = "lane: NOT DETECTED"
            lane_color = (0, 0, 255)

        cv2.putText(overlay, lane_text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, lane_color, 2)
        cv2.putText(
            overlay,
            f"traffic={traffic_state} score={traffic_score} stop_line={stop_line}",
            (12, 56),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )
        return overlay
