#!/usr/bin/env python3
"""Camera-to-control behavior gate for the ROI raw-UDP controller."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    # Package import when used as roi_camera_integration.
    from .front_camera_perception import FrontCameraObservation
except ImportError:
    # Top-level import when the files are run directly from run+camera/.
    from front_camera_perception import FrontCameraObservation


@dataclass(frozen=True)
class CameraControlPolicy:
    """Conservative defaults for a first on-road integration test."""

    camera_stale_timeout_sec: float = 0.5
    stop_hold_sec: float = 1.0
    red_requires_stop_line: bool = True
    lane_steer_gain: float = 0.0
    max_lane_steer_correction: float = 0.10


class FrontCameraBehavior:
    """Convert Front-camera observations into a safe control override.

    Lane steering is disabled by default.  The first integration only allows
    a red-light/stop-line observation to request braking.
    """

    def __init__(self, policy: Optional[CameraControlPolicy] = None) -> None:
        self.policy = policy or CameraControlPolicy()
        self._stop_until = 0.0
        self.last_observation: Optional[FrontCameraObservation] = None

    @property
    def stop_active(self) -> bool:
        return time.monotonic() < self._stop_until

    def update(self, observation: FrontCameraObservation) -> None:
        self.last_observation = observation
        now = observation.monotonic_time
        has_stop_evidence = observation.stop_line_detected or not self.policy.red_requires_stop_line
        if observation.traffic_state == "red" and has_stop_evidence:
            self._stop_until = max(self._stop_until, now + self.policy.stop_hold_sec)
        elif observation.traffic_state == "green":
            self._stop_until = 0.0

    def apply(
        self,
        accel: float,
        brake: float,
        steering_normalized: float,
        now_monotonic: Optional[float] = None,
    ) -> Tuple[float, float, float]:
        now = time.monotonic() if now_monotonic is None else float(now_monotonic)
        observation = self.last_observation

        if self._stop_until > now:
            return 0.0, 1.0, steering_normalized

        if observation is None or now - observation.monotonic_time > self.policy.camera_stale_timeout_sec:
            return accel, brake, steering_normalized

        corrected_steering = steering_normalized
        if self.policy.lane_steer_gain != 0.0 and observation.lane_offset_px is not None:
            normalized_offset = observation.lane_offset_px / max(1.0, observation.width * 0.5)
            correction = -self.policy.lane_steer_gain * normalized_offset
            correction = max(
                -self.policy.max_lane_steer_correction,
                min(self.policy.max_lane_steer_correction, correction),
            )
            corrected_steering = max(-1.0, min(1.0, steering_normalized + correction))

        return accel, brake, corrected_steering
