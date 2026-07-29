#!/usr/bin/env python3
"""Project a global MORAI route onto the front-camera image.

Coordinate convention used by the UDP controllers:

* vehicle +x: forward
* vehicle +y: left
* vehicle +z: up

The route and the ego pose must be expressed in the same map/local frame.
This module is visualization-only; it never changes a steering command.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple

try:
    import numpy as np
except ImportError:  # pragma: no cover - exercised on the Ubuntu target
    np = None

try:
    import cv2
except ImportError:  # pragma: no cover - only drawing needs OpenCV
    cv2 = None


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics derived from the MORAI horizontal camera FOV."""

    width: int
    height: int
    horizontal_fov_deg: float = 90.0
    vertical_fov_deg: Optional[float] = None
    cx: Optional[float] = None
    cy: Optional[float] = None

    def __post_init__(self) -> None:
        if self.width < 2 or self.height < 2:
            raise ValueError("camera image must be at least 2x2")
        if not 1.0 < self.horizontal_fov_deg < 179.0:
            raise ValueError("horizontal FOV must be between 1 and 179 degrees")
        if self.vertical_fov_deg is not None and not 1.0 < self.vertical_fov_deg < 179.0:
            raise ValueError("vertical FOV must be between 1 and 179 degrees")

    @property
    def fx(self) -> float:
        return 0.5 * float(self.width) / math.tan(
            math.radians(self.horizontal_fov_deg) * 0.5
        )

    @property
    def fy(self) -> float:
        if self.vertical_fov_deg is None:
            # The MORAI front camera is configured with square pixels. Deriving
            # fy from fx keeps the supplied 90-degree horizontal FOV consistent
            # after resizing 1280x720 to 640x360.
            return self.fx
        return 0.5 * float(self.height) / math.tan(
            math.radians(self.vertical_fov_deg) * 0.5
        )

    @property
    def principal_point(self) -> Tuple[float, float]:
        return (
            0.5 * float(self.width) if self.cx is None else float(self.cx),
            0.5 * float(self.height) if self.cy is None else float(self.cy),
        )


@dataclass(frozen=True)
class CameraMount:
    """Front camera pose relative to the vehicle reference point.

    ``pitch_down_deg`` is positive when the optical axis points down. MORAI's
    Cam 1 setting is represented by x=1.9, z=1.2, yaw=0, pitch-down=2 deg.
    """

    x_m: float = 1.9
    y_m: float = 0.0
    z_m: float = 1.2
    yaw_deg: float = 0.0
    pitch_down_deg: float = 2.0

    def basis_vehicle(self):
        yaw = math.radians(self.yaw_deg)
        pitch = math.radians(self.pitch_down_deg)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        cos_pitch = math.cos(pitch)
        sin_pitch = math.sin(pitch)

        # Camera coordinates: x=right, y=down, z=forward.
        forward = np.array(
            [cos_pitch * cos_yaw, cos_pitch * sin_yaw, -sin_pitch],
            dtype=np.float64,
        )
        right = np.array([sin_yaw, -cos_yaw, 0.0], dtype=np.float64)
        down = np.cross(forward, right)
        return right, down, forward


@dataclass(frozen=True)
class PathOverlayStats:
    points_in_front: int
    points_projected: int
    segments_drawn: int


def load_xyz_path(path: str):
    """Load whitespace/comma separated x y [z] path rows as an Nx3 array."""

    if np is None:
        raise RuntimeError("path overlay requires numpy")
    rows = []
    with open(path, encoding="utf-8") as source:
        for line_number, line in enumerate(source, start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            columns = stripped.replace(",", " ").split()
            if len(columns) < 2:
                continue
            try:
                x = float(columns[0])
                y = float(columns[1])
                z = float(columns[2]) if len(columns) >= 3 else 0.0
            except ValueError:
                # Permit a CSV header or other non-numeric metadata line.
                if line_number == 1:
                    continue
                raise ValueError("invalid path row {}: {}".format(line_number, stripped))
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                rows.append((x, y, z))
    if len(rows) < 2:
        raise ValueError("path must contain at least two numeric points")
    return np.asarray(rows, dtype=np.float64)


def points_to_xyz(points: Iterable, default_z: float = 0.0):
    """Convert controller waypoint objects/tuples to an Nx3 numpy array.

    The ROI package has used both ``x/y/z`` and ``x_m/y_m/z_m`` waypoint
    attributes over time. This adapter lets the overlay reuse whichever point
    representation the active Pure Pursuit implementation returns.
    """

    if np is None:
        raise RuntimeError("path overlay requires numpy")
    rows = []
    for point in points:
        if isinstance(point, dict):
            x = point.get("x_m", point.get("x"))
            y = point.get("y_m", point.get("y"))
            z = point.get("z_m", point.get("z", default_z))
        elif hasattr(point, "x_m") or hasattr(point, "y_m"):
            x = getattr(point, "x_m", None)
            y = getattr(point, "y_m", None)
            z = getattr(point, "z_m", default_z)
        elif hasattr(point, "x") or hasattr(point, "y"):
            x = getattr(point, "x", None)
            y = getattr(point, "y", None)
            z = getattr(point, "z", default_z)
        else:
            values = list(point)
            if len(values) < 2:
                continue
            x, y = values[:2]
            z = values[2] if len(values) >= 3 else default_z
        if x is None or y is None:
            continue
        rows.append((float(x), float(y), float(z)))
    if not rows:
        return np.empty((0, 3), dtype=np.float64)
    return np.asarray(rows, dtype=np.float64)


def world_to_vehicle(
    points_xyz,
    pose_x_m: float,
    pose_y_m: float,
    pose_z_m: float,
    pose_yaw_rad: float,
):
    """Transform map-frame points into the vehicle (+x forward, +y left) frame."""

    points = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    dx = points[:, 0] - float(pose_x_m)
    dy = points[:, 1] - float(pose_y_m)
    cos_yaw = math.cos(float(pose_yaw_rad))
    sin_yaw = math.sin(float(pose_yaw_rad))
    return np.column_stack(
        (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
            points[:, 2] - float(pose_z_m),
        )
    )


def project_vehicle_points(points_xyz, camera: CameraMount, intrinsics: CameraIntrinsics):
    """Project vehicle-frame points to pixels.

    Returns ``(pixels, depth)``. Invalid/behind-camera pixels are NaN; callers
    can use the positive finite depth to decide which line segments to draw.
    """

    if np is None:
        raise RuntimeError("path overlay requires numpy")
    points = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    relative = points - np.array([camera.x_m, camera.y_m, camera.z_m])
    right, down, forward = camera.basis_vehicle()
    camera_x = relative @ right
    camera_y = relative @ down
    depth = relative @ forward
    cx, cy = intrinsics.principal_point
    pixels = np.full((len(points), 2), np.nan, dtype=np.float64)
    valid = depth > 1.0e-6
    pixels[valid, 0] = intrinsics.fx * camera_x[valid] / depth[valid] + cx
    pixels[valid, 1] = intrinsics.fy * camera_y[valid] / depth[valid] + cy
    return pixels, depth


def project_path_to_image(
    path_points_xyz,
    pose_x_m: float,
    pose_y_m: float,
    pose_z_m: float,
    pose_yaw_rad: float,
    camera: CameraMount,
    intrinsics: CameraIntrinsics,
    min_forward_m: float = 0.5,
    max_forward_m: float = 45.0,
    max_lateral_m: float = 35.0,
):
    """Return projected pixels, validity flags, and vehicle-frame path points."""

    vehicle_points = world_to_vehicle(
        path_points_xyz, pose_x_m, pose_y_m, pose_z_m, pose_yaw_rad
    )
    pixels, depth = project_vehicle_points(vehicle_points, camera, intrinsics)
    in_front = (
        (vehicle_points[:, 0] >= float(min_forward_m))
        & (vehicle_points[:, 0] <= float(max_forward_m))
        & (np.abs(vehicle_points[:, 1]) <= float(max_lateral_m))
    )
    image_valid = (
        (pixels[:, 0] >= 0.0)
        & (pixels[:, 0] < float(intrinsics.width))
        & (pixels[:, 1] >= 0.0)
        & (pixels[:, 1] < float(intrinsics.height))
    )
    valid = in_front & (depth > 1.0e-6) & image_valid
    return pixels, valid, vehicle_points


def draw_path_overlay(
    image,
    path_points,
    pose_x_m: float,
    pose_y_m: float,
    pose_z_m: float,
    pose_yaw_rad: float,
    camera: CameraMount = CameraMount(),
    horizontal_fov_deg: float = 90.0,
    color: Tuple[int, int, int] = (255, 0, 255),
    thickness: int = 3,
    min_forward_m: float = 0.5,
    max_forward_m: float = 45.0,
    max_lateral_m: float = 35.0,
    use_path_z: bool = False,
    max_segment_length_m: float = 5.0,
):
    """Draw the visible route on a BGR camera image and return statistics."""

    if cv2 is None or np is None:
        raise RuntimeError("draw_path_overlay requires python3-opencv and numpy")
    if image is None or image.ndim != 3:
        raise ValueError("image must be a BGR image")
    height, width = image.shape[:2]
    intrinsics = CameraIntrinsics(width, height, horizontal_fov_deg)
    xyz = points_to_xyz(path_points, default_z=pose_z_m)
    if not use_path_z and len(xyz) > 0:
        # The competition route is a ground reference. Its z may be an
        # absolute map elevation while the live localizer's z is ENU-relative.
        # Keeping the route on the current road plane avoids a false vertical
        # displacement in the camera image. Enable use_path_z only after both
        # values have been confirmed to share the same vertical datum.
        xyz[:, 2] = float(pose_z_m)
    overlay = image.copy()
    if len(xyz) < 2:
        return overlay, PathOverlayStats(0, 0, 0)

    pixels, valid, vehicle_points = project_path_to_image(
        xyz,
        pose_x_m,
        pose_y_m,
        pose_z_m,
        pose_yaw_rad,
        camera,
        intrinsics,
        min_forward_m=min_forward_m,
        max_forward_m=max_forward_m,
        max_lateral_m=max_lateral_m,
    )
    segments_drawn = 0
    for index in range(len(xyz) - 1):
        if not (valid[index] and valid[index + 1]):
            continue
        if np.linalg.norm(vehicle_points[index + 1] - vehicle_points[index]) > max_segment_length_m:
            continue
        start = tuple(np.round(pixels[index]).astype(np.int32))
        end = tuple(np.round(pixels[index + 1]).astype(np.int32))
        cv2.line(overlay, start, end, color, int(thickness), cv2.LINE_AA)
        segments_drawn += 1

    projected_count = int(np.count_nonzero(valid))
    cv2.putText(
        overlay,
        "route_overlay points={}/{} pose=({:.1f},{:.1f}) yaw={:.1f}deg".format(
            projected_count,
            len(xyz),
            pose_x_m,
            pose_y_m,
            math.degrees(pose_yaw_rad),
        ),
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        color,
        2,
        cv2.LINE_AA,
    )
    return overlay, PathOverlayStats(
        points_in_front=int(np.count_nonzero(
            (vehicle_points[:, 0] >= float(min_forward_m))
            & (vehicle_points[:, 0] <= float(max_forward_m))
            & (np.abs(vehicle_points[:, 1]) <= float(max_lateral_m))
        )),
        points_projected=projected_count,
        segments_drawn=segments_drawn,
    )
