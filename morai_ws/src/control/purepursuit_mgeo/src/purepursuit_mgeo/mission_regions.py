"""Validated, route-bound mission regions shared by the publisher and gate."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path


def _finite(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(name + " must be set to a finite number")
    if not math.isfinite(number):
        raise ValueError(name + " must be finite")
    return number


def load_mission_regions(config_file, path_file, route_length):
    """Reject mismatched routes and partial roundabout calibration."""
    config = json.loads(Path(config_file).read_text(encoding="utf-8"))
    digest = hashlib.sha256(Path(path_file).read_bytes()).hexdigest()
    if config.get("route_sha256") != digest:
        raise ValueError("route_mission_regions route_sha256 does not match path_file")
    highway = config["highway"]
    highway.setdefault("handoff_enabled", False)
    highway.setdefault("handoff_s_m", None)
    start = _finite(highway.get("start_s_m"), "highway.start_s_m")
    end = _finite(highway.get("end_s_m"), "highway.end_s_m")
    if not 0.0 <= start < end <= route_length:
        raise ValueError("highway region is outside the route")
    if _finite(highway.get("max_route_offset_m"), "highway.max_route_offset_m") <= 0:
        raise ValueError("highway route offset must be positive")
    handoff_enabled = highway.get("handoff_enabled", False)
    if not isinstance(handoff_enabled, bool):
        raise ValueError("highway.handoff_enabled must be a boolean")
    handoff_s = None
    if handoff_enabled:
        handoff_s = _finite(highway.get("handoff_s_m"), "highway.handoff_s_m")
        if not start < handoff_s <= route_length:
            raise ValueError("highway handoff must follow its start and lie on the route")
    roundabout = config["roundabout"]
    if roundabout.get("enabled"):
        required = ("request_start_s_m", "yield_s_m", "conflict_s_m", "request_end_s_m")
        values = [_finite(roundabout.get(key), "roundabout." + key) for key in required]
        if not 0.0 <= values[0] < values[1] < values[2] < values[3] <= route_length:
            raise ValueError("roundabout positions must follow route travel order")
        highway_exit = max(end, handoff_s) if handoff_enabled else end
        if not (values[3] <= start or highway_exit <= values[0]):
            raise ValueError("roundabout request region must not overlap highway region")
        xy = roundabout.get("conflict_xy_map")
        if not isinstance(xy, list) or len(xy) != 2:
            raise ValueError("roundabout.conflict_xy_map must contain [x,y]")
        [_finite(item, "roundabout.conflict_xy_map") for item in xy]
        if _finite(roundabout.get("conflict_radius_m"), "roundabout.conflict_radius_m") <= 0:
            raise ValueError("roundabout conflict radius must be positive")
        if _finite(roundabout.get("max_route_offset_m"), "roundabout.max_route_offset_m") <= 0:
            raise ValueError("roundabout route offset must be positive")
        link_ids = roundabout.get("circulating_link_ids")
        if (not isinstance(link_ids, list) or not link_ids
                or any(not isinstance(item, str) or not item for item in link_ids)):
            raise ValueError("roundabout circulating_link_ids must be configured")
    return config
