"""Frame-local CAM4 ROI, signal-shape and duplicate-detection helpers.

Coordinates passed to box filtering and merging are always in the original
image, including detections produced by a crop. No ROS or model dependency is
required, and no tracking history or traffic-light permissions are inferred.
"""

import math
from numbers import Integral, Real


def _finite(value):
    return isinstance(value, Real) and not isinstance(value, bool) and math.isfinite(value)


def _image_size(width, height):
    if (not all(_finite(value) and value > 0 and int(value) == value
                for value in (width, height))):
        raise ValueError('Image width and height must be positive integers')
    return int(width), int(height)


def validate_signal_roi(roi):
    """Return finite (xmin, ymin, xmax, ymax) bounds within [0, 1].

    Lower bounds must be strictly smaller than upper bounds. A malformed or
    empty ROI raises ValueError instead of silently selecting the full image.
    """
    if (not isinstance(roi, (tuple, list)) or len(roi) != 4
            or not all(_finite(value) for value in roi)):
        raise ValueError('Signal ROI requires four finite normalized numbers')
    x0, y0, x1, y1 = (float(value) for value in roi)
    if not (0.0 <= x0 < x1 <= 1.0 and 0.0 <= y0 < y1 <= 1.0):
        raise ValueError('Signal ROI requires 0 <= xmin < xmax <= 1 and 0 <= ymin < ymax <= 1')
    return x0, y0, x1, y1


def signal_crop_rect(roi, image_width, image_height):
    """Return integer half-open slice bounds for a crop at least 2 x 2 px.

    Bounds use the same integer truncation as the supplied detector. ROI
    validation and pixel-size validation raise ValueError on invalid inputs.
    """
    x0, y0, x1, y1 = validate_signal_roi(roi)
    width, height = _image_size(image_width, image_height)
    rect = int(x0 * width), int(y0 * height), int(x1 * width), int(y1 * height)
    if rect[2] - rect[0] < 2 or rect[3] - rect[1] < 2:
        raise ValueError('Signal ROI must cover at least two pixels in each dimension')
    return rect


def signal_box_allowed(x1, y1, x2, y2, image_width, image_height):
    """Accept finite, in-frame horizontal signal housings in the upper image.

    The supplied model's shape limits apply to original-image dimensions:
    width/height in [2, 5], width <= 15% W, height <= 10% H, and center y <=
    65% H. No horizontal-position filter is applied, so right-side signals
    remain visible through the full-frame pass. Crop-edge truncation must be
    rejected by the caller before converting crop-local coordinates.
    """
    try:
        width, height = _image_size(image_width, image_height)
    except (ValueError, TypeError, OverflowError):
        return False
    if not all(_finite(value) for value in (x1, y1, x2, y2)):
        return False
    if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
        return False
    box_width, box_height = x2 - x1, y2 - y1
    return (box_width >= 2.0 and box_height >= 2.0
            and (y1 + y2) * 0.5 <= height * 0.65
            and 2.0 <= box_width / box_height <= 5.0
            and box_width <= width * 0.15 and box_height <= height * 0.10)


def _record_copy(record):
    """Copy a well-formed candidate; reject invalid model output locally."""
    if not isinstance(record, dict):
        return None
    xyxy = record.get('xyxy')
    score = record.get('score')
    label = record.get('label')
    track_id = record.get('track_id')
    if (not isinstance(xyxy, (tuple, list)) or len(xyxy) != 4
            or not all(_finite(value) for value in xyxy)
            or not xyxy[0] < xyxy[2] or not xyxy[1] < xyxy[3]
            or not _finite(score) or not 0.0 <= score <= 1.0
            or not isinstance(label, str) or not label.strip()
            or record.get('source') not in ('full', 'roi')
            or (track_id is not None and
                (isinstance(track_id, bool) or not isinstance(track_id, Integral)))):
        return None
    copied = dict(record)
    copied['xyxy'] = tuple(float(value) for value in xyxy)
    copied['score'] = float(score)
    copied['track_id'] = int(track_id) if track_id is not None else None
    return copied


def _iou(first, second):
    ax1, ay1, ax2, ay2 = first['xyxy']
    bx1, by1, bx2, by2 = second['xyxy']
    intersection = (max(0.0, min(ax2, bx2) - max(ax1, bx1))
                    * max(0.0, min(ay2, by2) - max(ay1, by1)))
    union = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - intersection
    if not math.isfinite(intersection) or not math.isfinite(union) or union <= 0.0:
        return 0.0
    return intersection / union


def merge_signal_detections(records):
    """Merge same-label, IoU >= 0.6 crop/full duplicates from one frame.

    Labels are compared using stripped casefold text only: RED and LEFT or
    RED and GREEN are never collapsed. The highest-scoring candidate retains
    its geometry, score, label and source together. Its track ID may be copied
    from the unique full-frame track in that duplicate group. An overlapping
    same-label component containing different full-frame IDs is preserved in
    full, avoiding ambiguous assignment of a crop box to either signal head.
    Remaining groups use direct overlap with the winning box, not transitive
    overlap, so a chain cannot erase a nonoverlapping head. Inputs are never
    modified; malformed records are omitted from the returned copies.
    """
    candidates = []
    for record in records:
        copied = _record_copy(record)
        if copied is not None:
            candidates.append(copied)
    labels = [record['label'].strip().casefold() for record in candidates]
    neighbors = [set() for _ in candidates]
    for first in range(len(candidates)):
        for second in range(first + 1, len(candidates)):
            if labels[first] == labels[second] and _iou(candidates[first], candidates[second]) >= 0.6:
                neighbors[first].add(second)
                neighbors[second].add(first)

    ambiguous = set()
    unseen = set(range(len(candidates)))
    while unseen:
        pending = [min(unseen)]
        component = set()
        while pending:
            index = pending.pop()
            if index in component:
                continue
            component.add(index)
            unseen.discard(index)
            pending.extend(neighbors[index] - component)
        full_ids = {candidates[index]['track_id'] for index in component
                    if candidates[index]['source'] == 'full'
                    and candidates[index]['track_id'] is not None}
        if len(full_ids) > 1:
            ambiguous.update(component)

    remaining = set(range(len(candidates)))
    merged = []
    for index in sorted(remaining, key=lambda item: (-candidates[item]['score'], item)):
        if index not in remaining:
            continue
        winner = dict(candidates[index])
        group = {index} if index in ambiguous else ({index} | (neighbors[index] & remaining))
        # Ambiguous components cannot be adjacent to a nonambiguous winner,
        # but this explicit exclusion keeps the preservation rule local.
        if index not in ambiguous:
            group -= ambiguous
        full_ids = {candidates[item]['track_id'] for item in group
                    if candidates[item]['source'] == 'full'
                    and candidates[item]['track_id'] is not None}
        if len(full_ids) == 1:
            winner['track_id'] = next(iter(full_ids))
        merged.append(winner)
        remaining.difference_update(group)
    return merged
