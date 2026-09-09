"""Bind signal permission to the competition route used by steering."""
import math


def reference_path_reason(points, message):
    """Compare the complete ordered map path, including elevation.

    A latched reference path is static configuration, not a sensor sample;
    its timestamp need not be recent. No partial nearest-point match suffices.
    """
    try:
        if message.header.frame_id != "map":
            return "reference_path_frame_mismatch"
        if len(message.poses) != len(points):
            return "reference_path_length_mismatch"
        for expected, pose in zip(points, message.poses):
            if pose.header.frame_id not in ("", "map"):
                return "reference_path_frame_mismatch"
            actual = pose.pose.position
            for axis in ("x", "y", "z"):
                a, b = float(getattr(expected, axis)), float(getattr(actual, axis))
                if not (math.isfinite(a) and math.isfinite(b)):
                    return "reference_path_invalid"
                if abs(a - b) > 1e-6:
                    return "reference_path_geometry_mismatch"
    except (AttributeError, TypeError, ValueError, OverflowError):
        return "reference_path_invalid"
    return "matched"


def route_event_key(event):
    """An authorization cannot transfer to a different route movement.

    start is excluded: a camera stop-line observation may refine its distance.
    The map link, stop boundary, exit and permitted movement remain fixed.
    """
    if event is None:
        return None
    return (event["id"], event["kind"], event["direction"], event.get("source"),
            event.get("link_id"), event.get("stop_s"), event["end"],
            tuple(sorted(str(h["id"]) for h in event.get("signal_points", []))))


def route_checked_maneuvers(maneuvers, contexts):
    """Manual turns may annotate a unique mapped movement, never override it.

    Return only lane-change events. Mapped turns already supply their own
    boundaries and signal heads; adding a second event would duplicate them.
    """
    result = []
    for maneuver in maneuvers:
        matches = [c for c in contexts if maneuver.start_s_m < c["end"]
                   and maneuver.end_s_m > c["start"]]
        if maneuver.kind == "lane_change":
            if matches:
                raise ValueError("Lane-change plans must not overlap signal-controlled contexts")
            result.append(maneuver)
            continue
        if len(matches) != 1:
            raise ValueError("Manual turn %s requires exactly one mapped route movement" % maneuver.identifier)
        context = matches[0]
        if (context.get("source") != "mgeo" or context.get("binding_status") != "matched"
                or not context.get("link_id") or context["direction"] != maneuver.direction
                or not context.get("signal_points")):
            raise ValueError("Manual turn %s conflicts with the competition route or lacks signal binding"
                             % maneuver.identifier)
    return result
