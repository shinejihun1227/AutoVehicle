#!/usr/bin/env python3
"""Locate a measured map XY or route distance on the configured MGeo route."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

# Allow invocation directly from a source checkout without a catkin build.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from purepursuit_mgeo.route_geometry import RoutePolyline


def nearby_links(link_set_file, x, y, count):
    links = json.loads(Path(link_set_file).read_text(encoding="utf-8"))
    nearest = []
    for link in links:
        try:
            polyline = RoutePolyline(link["points"])
            match = polyline.project(x, y)
        except (KeyError, TypeError, ValueError, ZeroDivisionError):
            continue
        nearest.append({
            "link_id": str(link["idx"]),
            "offset_m": round(match["distance_m"], 3),
            "link_s_m": round(match["s_m"], 3),
            "tangent_xy": [round(value, 4) for value in match["tangent"]],
        })
    nearest.sort(key=lambda item: (item["offset_m"], item["link_id"]))
    return nearest[:count]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path-file", required=True, help="global path text file")
    parser.add_argument("--link-set-file", required=True, help="MGeo link_set.json")
    point = parser.add_mutually_exclusive_group(required=True)
    point.add_argument("--xy", type=float, nargs=2, metavar=("X", "Y"),
                       help="measured map-frame XY, such as an RViz point")
    point.add_argument("--s", type=float, help="route progress in metres")
    parser.add_argument("--hint-s", type=float,
                        help="expected route progress for overlapping path sections")
    parser.add_argument("--window-m", type=float, default=80.0,
                        help="projection search distance around --hint-s")
    parser.add_argument("--link-count", type=int, default=5)
    args = parser.parse_args()
    if args.window_m <= 0.0 or args.link_count <= 0:
        parser.error("--window-m and --link-count must be positive")
    if args.hint_s is not None and args.xy is None:
        parser.error("--hint-s is only meaningful with --xy")

    route = RoutePolyline.from_path_file(args.path_file)
    if args.xy is not None:
        x, y = args.xy
        match = route.project(x, y, expected_s=args.hint_s,
                              window_m=args.window_m if args.hint_s is not None else None)
        progress = match["s_m"]
        offset = match["distance_m"]
    else:
        progress = float(args.s)
        if not math.isfinite(progress) or not 0.0 <= progress <= route.length:
            parser.error("--s must lie within the route")
        x, y = route.point_at(progress)
        offset = 0.0
    if not all(math.isfinite(value) for value in (x, y)):
        parser.error("map XY must be finite")
    result = {
        "route_sha256": hashlib.sha256(Path(args.path_file).read_bytes()).hexdigest(),
        "route_length_m": round(route.length, 3),
        "route_s_m": round(progress, 3),
        "measured_xy_map": [round(x, 3), round(y, 3)],
        "route_xy_map": [round(v, 3) for v in route.point_at(progress)],
        "route_offset_m": round(offset, 3),
        "nearby_mgeo_links": nearby_links(args.link_set_file, x, y, args.link_count),
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
