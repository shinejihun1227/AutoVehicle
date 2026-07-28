#!/usr/bin/env python3
"""Dependency-free multi-camera lane estimate fusion primitives.

The image-specific work belongs in a camera perception process.  This module
only consumes estimates that have already been transformed into the vehicle
frame, so it can be tested without MORAI, ROS, OpenCV, or a GPU.

Vehicle-frame convention:
    +x points forward and +y points to the vehicle's left.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Mapping, Optional, Tuple


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, float(value)))


def _wrap_angle(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class LaneObservation:
    """One camera's lane estimate in the common vehicle coordinate frame."""

    source: str
    stamp: float
    lateral_error_m: float
    heading_error_rad: float
    confidence: float
    curvature_1pm: Optional[float] = None


@dataclass(frozen=True)
class FusedLaneEstimate:
    """Fused lane estimate consumed by the steering-assist layer."""

    valid: bool
    stamp: Optional[float]
    lateral_error_m: float
    heading_error_rad: float
    confidence: float
    source_count: int
    sources: Tuple[str, ...]
    curvature_1pm: Optional[float] = None


class MultiCameraLaneFusion:
    """Fuse fresh, confidence-weighted camera estimates.

    Outlier rejection is deliberately simple and deterministic for the first
    integration: estimates farther than ``outlier_threshold_m`` from the
    lateral median are ignored.  If every estimate is rejected, the closest
    estimate to the median is retained so a single valid camera can still be
    used.
    """

    def __init__(
        self,
        max_age_sec: float = 0.20,
        min_confidence: float = 0.20,
        outlier_threshold_m: float = 1.50,
        source_weights: Optional[Mapping[str, float]] = None,
    ) -> None:
        if max_age_sec <= 0.0:
            raise ValueError("max_age_sec must be positive")
        if outlier_threshold_m <= 0.0:
            raise ValueError("outlier_threshold_m must be positive")
        self.max_age_sec = float(max_age_sec)
        self.min_confidence = _clamp(min_confidence, 0.0, 1.0)
        self.outlier_threshold_m = float(outlier_threshold_m)
        self.source_weights = dict(source_weights or {})

    def fuse(
        self,
        observations: Iterable[LaneObservation],
        now: float,
    ) -> FusedLaneEstimate:
        """Return a safe invalid result when no usable estimate is available."""

        candidates = [
            observation
            for observation in observations
            if self._is_usable(observation, now)
        ]
        if not candidates:
            return FusedLaneEstimate(
                valid=False,
                stamp=None,
                lateral_error_m=0.0,
                heading_error_rad=0.0,
                confidence=0.0,
                source_count=0,
                sources=(),
            )

        median_lateral = self._median(
            [observation.lateral_error_m for observation in candidates]
        )
        inliers = [
            observation
            for observation in candidates
            if abs(observation.lateral_error_m - median_lateral)
            <= self.outlier_threshold_m
        ]
        if not inliers:
            inliers = [
                min(
                    candidates,
                    key=lambda observation: abs(
                        observation.lateral_error_m - median_lateral
                    ),
                )
            ]

        weights = [self._weight(observation) for observation in inliers]
        total_weight = sum(weights)
        lateral_error = sum(
            weight * observation.lateral_error_m
            for weight, observation in zip(weights, inliers)
        ) / total_weight

        heading_sin = sum(
            weight * math.sin(observation.heading_error_rad)
            for weight, observation in zip(weights, inliers)
        )
        heading_cos = sum(
            weight * math.cos(observation.heading_error_rad)
            for weight, observation in zip(weights, inliers)
        )
        heading_error = math.atan2(heading_sin, heading_cos)

        source_weight_total = sum(
            self.source_weights.get(observation.source, 1.0)
            for observation in inliers
        )
        mean_confidence = sum(
            self.source_weights.get(observation.source, 1.0)
            * _clamp(observation.confidence, 0.0, 1.0)
            for observation in inliers
        ) / source_weight_total
        spread = max(
            abs(observation.lateral_error_m - lateral_error)
            for observation in inliers
        )
        agreement = _clamp(
            1.0 - spread / self.outlier_threshold_m,
            0.0,
            1.0,
        )

        curvature_values = [
            (weight, observation.curvature_1pm)
            for weight, observation in zip(weights, inliers)
            if observation.curvature_1pm is not None
            and math.isfinite(observation.curvature_1pm)
        ]
        curvature = None
        if curvature_values:
            curvature_weight = sum(weight for weight, _ in curvature_values)
            curvature = sum(
                weight * float(value) for weight, value in curvature_values
            ) / curvature_weight

        return FusedLaneEstimate(
            valid=True,
            stamp=max(observation.stamp for observation in inliers),
            lateral_error_m=lateral_error,
            heading_error_rad=heading_error,
            confidence=mean_confidence * agreement,
            source_count=len(inliers),
            sources=tuple(sorted(observation.source for observation in inliers)),
            curvature_1pm=curvature,
        )

    def _is_usable(self, observation: LaneObservation, now: float) -> bool:
        values = (
            observation.stamp,
            observation.lateral_error_m,
            observation.heading_error_rad,
            observation.confidence,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return False
        if now - observation.stamp > self.max_age_sec:
            return False
        if observation.confidence < self.min_confidence:
            return False
        return True

    def _weight(self, observation: LaneObservation) -> float:
        source_weight = max(
            0.0, float(self.source_weights.get(observation.source, 1.0))
        )
        return max(1e-9, source_weight * _clamp(observation.confidence, 0.0, 1.0))

    @staticmethod
    def _median(values: list[float]) -> float:
        ordered = sorted(values)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return 0.5 * (ordered[middle - 1] + ordered[middle])


def calculate_lane_steering(
    estimate: FusedLaneEstimate,
    speed_mps: float,
    lateral_gain: float = 1.0,
    heading_gain: float = 0.8,
    softening_speed_mps: float = 2.0,
    max_correction_rad: float = math.radians(3.0),
) -> float:
    """Return a bounded steering residual; invalid estimates return zero."""

    if not estimate.valid or estimate.confidence <= 0.0:
        return 0.0
    if speed_mps < 0.0 or softening_speed_mps <= 0.0:
        raise ValueError("speed_mps must be non-negative and softening positive")

    lateral_term = math.atan2(
        float(lateral_gain) * estimate.lateral_error_m,
        max(0.0, float(speed_mps)) + float(softening_speed_mps),
    )
    correction = lateral_term + float(heading_gain) * estimate.heading_error_rad
    return _clamp(correction, -abs(max_correction_rad), abs(max_correction_rad))
