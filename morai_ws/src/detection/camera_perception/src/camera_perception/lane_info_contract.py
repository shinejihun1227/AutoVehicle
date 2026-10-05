"""Transport-independent contract for real_lane JSON (x forward, y left)."""
import copy
import math


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def fresh_payload(info, now, timeout=0.6):
    stamp = info.get('timestamp') if isinstance(info, dict) else None
    return (finite(stamp) and stamp > 0 and -0.05 <= now - stamp <= timeout
            and info.get('frame_id') == 'base_link'
            and info.get('coordinate_convention') == {'x': 'forward_m', 'y': 'left_m'})


def interpolate(points, x):
    if not isinstance(points, list) or len(points) < 2:
        return None
    if any(not isinstance(p, (list, tuple)) or len(p) < 2
           or not all(finite(v) for v in p[:2]) for p in points):
        return None
    if any(a[0] >= b[0] for a, b in zip(points, points[1:])):
        return None
    for a, b in zip(points, points[1:]):
        if a[0] <= x <= b[0]:
            return a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0])
    return None  # Never extrapolate a short camera observation to 7/14 m.


def convert(info, now, timeout=0.6, min_confidence=0.45):
    info = copy.deepcopy(info) if isinstance(info, dict) else {}
    fresh = fresh_payload(info, now, timeout)
    semantics = {k: False for k in ('dashed', 'left_solid', 'left_yellow', 'right_solid')}
    for side in ('left', 'right'):
        boundary = info.get(side + '_lane') or {}
        if (fresh and isinstance(boundary, dict) and boundary.get('detected') is True
                and not boundary.get('from_guide') and not boundary.get('coasted')):
            typ = boundary.get('type')
            semantics[side + '_solid'] = typ in ('white_solid', 'yellow')
            if side == 'left':
                semantics['dashed'] = typ == 'white_dashed' and boundary.get('dashed') is True
                semantics['left_yellow'] = typ == 'yellow'
    points = info.get('centerline_points')
    y7, y14 = interpolate(points, 7.0), interpolate(points, 14.0)
    confidence = info.get('confidence', 0.0)
    width = info.get('lane_width_m')
    straddling = info.get('straddling_lane') or {}
    lane_valid = (fresh and info.get('lane_valid') is True and finite(confidence)
                  and min_confidence <= confidence <= 1.0
                  and finite(width) and 2.7 <= width <= 4.2
                  and not straddling.get('detected') and y7 is not None and y14 is not None)
    line = info.get('stopline') or {}
    distance = info.get('stopline_distance_m')
    support = line.get('inlier_ratio', 0.0) if isinstance(line, dict) else 0.0
    stop_valid = (fresh and info.get('stopline_detected') is True
                  and isinstance(line, dict) and line.get('covers_front') is True
                  and finite(distance) and 0.0 <= distance <= 80.0
                  and finite(support) and 0.5 <= support <= 1.0)
    if not fresh:
        info.update(lane_valid=False, confidence=0.0, output_status='STALE',
                    centerline_points=None, stopline_detected=False, stopline_distance_m=None,
                    left_lane={'detected': False}, right_lane={'detected': False}, all_lanes=[])
    return info, semantics, dict(valid=bool(lane_valid),
        lateral_offset_m=-y7 if lane_valid else 0.0,
        heading_error_rad=math.atan2(y14-y7, 7.0) if lane_valid else 0.0,
        confidence=confidence if lane_valid else 0.0), dict(valid=bool(stop_valid),
        distance_m=distance if stop_valid else 0.0, confidence=support if stop_valid else 0.0)
