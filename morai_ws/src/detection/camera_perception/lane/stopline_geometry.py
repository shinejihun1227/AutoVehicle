"""Select the nearest transverse stop-line band in a metric BEV class mask.

Only NumPy is needed; tests do not load the segmentation model or ROS. Quality
is geometric support, not a calibrated probability that the marking is real.
"""

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class StopLineCandidate:
    distance_m: float
    lateral_m: float
    confidence: float
    width_m: float
    thickness_m: float


def select_stopline(bev, x_max_m=40.0, y_max_m=10.0, resolution_m=0.05,
                    class_id=4, half_width_m=2.0, min_pixels=30,
                    min_width_m=1.5, side_support_m=0.4,
                    max_longitudinal_gap_m=0.15, max_lateral_gap_m=0.3,
                    max_thickness_m=0.7, max_angle_deg=35.0):
    """Fit x = slope*y + distance separately for each longitudinal band.

    A candidate must cross the ego centre with substantial support on both
    sides, without a large hole, excessive thickness or a longitudinal slope.
    Per-column medians prevent a thick/noisy patch from dominating the fit.
    Distance is the centreline intersection, not a median of unrelated lines.
    No temporal smoothing is applied here: current source time remains valid,
    and the controller owns motion compensation and detection-loss handling.
    """
    numeric = (x_max_m, y_max_m, resolution_m, half_width_m, min_pixels,
               min_width_m, side_support_m, max_longitudinal_gap_m,
               max_lateral_gap_m, max_thickness_m, max_angle_deg)
    if (not all(math.isfinite(v) and v > 0 for v in numeric)
            or not side_support_m < half_width_m or max_angle_deg >= 90):
        raise ValueError("Stop-line geometry limits must be finite and positive")
    mask = np.asarray(bev)
    if mask.ndim != 2 or not mask.size:
        return None
    columns = y_max_m - (np.arange(mask.shape[1]) + 0.5) * resolution_m
    corridor = np.flatnonzero(np.abs(columns) <= half_width_m)
    if not corridor.size:
        return None
    rr, local_cc = np.nonzero(mask[:, corridor] == class_id)
    cc = corridor[local_cc]
    if rr.size < min_pixels:
        return None
    # np.nonzero is row ordered. Blank distance bands separate intersections;
    # unrelated side-road pixels outside the corridor cannot connect them.
    breaks = np.flatnonzero(np.diff(rr) * resolution_m >
                            max_longitudinal_gap_m + resolution_m + 1e-9) + 1
    candidates = []
    for indices in np.split(np.arange(rr.size), breaks):
        if indices.size < min_pixels:
            continue
        x = x_max_m - (rr[indices] + 0.5) * resolution_m
        y = columns[cc[indices]]
        unique_columns = np.unique(cc[indices])
        ys = columns[unique_columns]
        width = float(np.ptp(ys) + resolution_m)
        if (ys.size < 2 or width < min_width_m
                or y.min() > -side_support_m or y.max() < side_support_m
                or np.max(np.diff(unique_columns) - 1) * resolution_m > max_lateral_gap_m + 1e-9):
            continue
        medians = np.array([np.median(x[cc[indices] == col]) for col in unique_columns])
        slope, distance = np.polyfit(ys, medians, 1)
        residual = x - (slope * y + distance)
        thickness = float(np.percentile(residual, 95) - np.percentile(residual, 5)
                          + resolution_m)
        if (distance <= 0 or abs(slope) > math.tan(math.radians(max_angle_deg))
                or thickness > max_thickness_m or width < 3.0 * thickness):
            continue
        coverage = min(1.0, ys.size * resolution_m / width)
        shape = max(0.0, 1.0 - thickness / max_thickness_m)
        support = min(1.0, width / (2.0 * half_width_m))
        confidence = min(0.99, 0.5 + 0.2 * support + 0.15 * coverage + 0.15 * shape)
        candidates.append(StopLineCandidate(float(distance), 0.0, confidence,
                                             width, thickness))
    return min(candidates, key=lambda item: item.distance_m) if candidates else None
