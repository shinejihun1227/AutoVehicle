#!/usr/bin/env python3
"""Read-only, ROS/network-free route and static signal preflight."""
import argparse
import json

from curvature_speed_purepursuit.planner import (
    load_path_file, clean_consecutive_duplicates, cumulative_arc_lengths,
)
from turn_signal_controller.route_context import load_route_contexts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path-file", required=True)
    parser.add_argument("--mgeo-dir", required=True)
    args = parser.parse_args()
    points = clean_consecutive_duplicates(load_path_file(args.path_file))
    contexts = load_route_contexts(args.mgeo_dir, points, cumulative_arc_lengths(points))
    print(json.dumps({
        "context_count": len(contexts),
        "unknown_count": sum(c["direction"] == "UNKNOWN" for c in contexts),
        "unverified_stopline_count": sum(c["stop_s"] is None for c in contexts),
        "note": "Static preflight only; NOT camera calibration or driving approval",
        "contexts": contexts,
    }, indent=2, allow_nan=False))
    return 0 if contexts else 2


if __name__ == "__main__":
    raise SystemExit(main())
