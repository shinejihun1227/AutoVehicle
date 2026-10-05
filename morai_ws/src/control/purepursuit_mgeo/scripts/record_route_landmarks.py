#!/usr/bin/env python3
"""Press Enter while driving to record map XY and route progress for calibration."""

from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
import threading
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry
from morai_perception_msgs.msg import StopLineDetection
from std_msgs.msg import Float64

from purepursuit_mgeo.landmark_capture import project_stopline, quaternion_yaw
from purepursuit_mgeo.route_geometry import RoutePolyline


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="JSON-lines file for captured points")
    parser.add_argument("--odom-topic", default="/localization/odometry")
    parser.add_argument("--progress-topic", default="/experimental/curvature_progress")
    parser.add_argument("--stopline-topic", default="/perception/camera/stopline")
    parser.add_argument("--path-file", help="global route; enables stop-line map projection")
    parser.add_argument("--max-age-s", type=float, default=0.6)
    parser.add_argument("--pose-sync-tolerance-s", type=float, default=0.05)
    parser.add_argument("--max-stopline-route-offset-m", type=float, default=4.0,
                        help="reject projected stop lines farther from the route")
    args = parser.parse_args()
    for name, value in (("--max-age-s", args.max_age_s),
                        ("--pose-sync-tolerance-s", args.pose_sync_tolerance_s),
                        ("--max-stopline-route-offset-m", args.max_stopline_route_offset_m)):
        if not math.isfinite(value) or value <= 0.0:
            parser.error(name + " must be positive and finite")
    route = None
    route_digest = None
    if args.path_file:
        try:
            route = RoutePolyline.from_path_file(args.path_file)
            route_digest = hashlib.sha256(Path(args.path_file).read_bytes()).hexdigest()
        except (OSError, ValueError) as exc:
            parser.error("cannot load --path-file: " + str(exc))

    rospy.init_node("record_route_landmarks", anonymous=True, disable_signals=True)
    lock = threading.Lock()
    latest = {"odom": None, "odom_at": None, "progress": None, "progress_at": None}
    odom_history = deque(maxlen=200)
    progress_history = deque(maxlen=200)
    latest_stopline = {"message": None}

    def on_odom(message):
        if message.header.frame_id != "map":
            return
        with lock:
            latest["odom"] = message
            latest["odom_at"] = rospy.Time.now()
            odom_history.append(message)

    def on_progress(message):
        if not math.isfinite(float(message.data)):
            return
        with lock:
            latest["progress"] = float(message.data)
            latest["progress_at"] = rospy.Time.now()
            progress_history.append((rospy.Time.now(), float(message.data)))

    def on_stopline(message):
        with lock:
            latest_stopline["message"] = message

    rospy.Subscriber(args.odom_topic, Odometry, on_odom, queue_size=1)
    rospy.Subscriber(args.progress_topic, Float64, on_progress, queue_size=1)
    rospy.Subscriber(args.stopline_topic, StopLineDetection, on_stopline, queue_size=1)
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    print("지점에서 Enter를 누르세요. 현재 map 위치와 경로 거리를 기록합니다.", flush=True)
    if route is not None:
        print("유효하고 시각이 맞는 정지선 검출도 map 좌표/경로 거리로 함께 기록합니다.", flush=True)
    print("Ctrl+C로 종료합니다.", flush=True)
    sequence = 0
    try:
        while not rospy.is_shutdown():
            input("capture> ")
            with lock:
                snapshot = dict(latest)
                line_snapshot = dict(latest_stopline)
                poses = list(odom_history)
                progress_samples = list(progress_history)
            now = rospy.Time.now()
            odom = snapshot["odom"]
            progress = snapshot["progress"]
            source_age = (now - odom.header.stamp).to_sec() if odom is not None else float("inf")
            odom_age = ((now - snapshot["odom_at"]).to_sec()
                        if snapshot["odom_at"] is not None else float("inf"))
            progress_age = ((now - snapshot["progress_at"]).to_sec()
                            if snapshot["progress_at"] is not None else float("inf"))
            if (odom is None or progress is None or odom_age > args.max_age_s
                    or progress_age > args.max_age_s or not 0.0 <= source_age <= args.max_age_s):
                print("위치 또는 경로 진행 거리 데이터가 오래되었거나 없습니다. 저장하지 않았습니다.", flush=True)
                continue
            x, y = float(odom.pose.pose.position.x), float(odom.pose.pose.position.y)
            if not math.isfinite(x) or not math.isfinite(y):
                print("유효한 map 좌표가 아닙니다. 저장하지 않았습니다.", flush=True)
                continue
            # Freeze the sample before asking for a human-readable label.
            entry = {
                "captured_at_ros_s": round(now.to_sec(), 3),
                "map_xy": [round(x, 3), round(y, 3)],
                "route_s_m": round(progress, 3),
            }
            if route_digest is not None:
                entry["route_sha256"] = route_digest
            line = line_snapshot["message"]
            line_reason = "stopline_not_received"
            if line is not None:
                line_stamp = line.header.stamp.to_sec()
                line_age = now.to_sec() - line_stamp
                if line_stamp <= 0.0:
                    line_reason = "stopline_stamp_invalid"
                elif line.header.frame_id != "base_link":
                    line_reason = "stopline_frame_not_base_link"
                elif not line.valid or not math.isfinite(float(line.distance_m)) or line.distance_m < 0.0:
                    line_reason = "stopline_invalid"
                elif (not math.isfinite(float(line.confidence))
                      or not 0.5 <= float(line.confidence) <= 1.0):
                    line_reason = "stopline_confidence_invalid"
                elif not 0.0 <= line_age <= args.max_age_s:
                    line_reason = "stopline_stale"
                elif route is None:
                    line_reason = "path_file_not_set"
                else:
                    pose = min(poses, key=lambda item: abs(item.header.stamp.to_sec() - line_stamp),
                               default=None)
                    progress_pair = min(progress_samples,
                                        key=lambda item: abs(item[0].to_sec() - line_stamp),
                                        default=None)
                    pose_delta = (abs(pose.header.stamp.to_sec() - line_stamp)
                                  if pose is not None else float("inf"))
                    progress_delta = (abs(progress_pair[0].to_sec() - line_stamp)
                                      if progress_pair is not None else float("inf"))
                    if pose_delta > args.pose_sync_tolerance_s:
                        line_reason = "stopline_pose_unsynchronized"
                    elif progress_delta > args.max_age_s:
                        line_reason = "stopline_progress_unsynchronized"
                    else:
                        position = pose.pose.pose.position
                        orientation = pose.pose.pose.orientation
                        try:
                            yaw = quaternion_yaw(orientation.x, orientation.y,
                                                 orientation.z, orientation.w)
                            mapped = project_stopline(
                                route, position.x, position.y, yaw,
                                line.distance_m, progress_pair[1])
                            if mapped["route_offset_m"] > args.max_stopline_route_offset_m:
                                line_reason = "stopline_too_far_from_route"
                            else:
                                entry["stopline_landmark"] = {
                                    "map_xy": [round(value, 3) for value in mapped["map_xy"]],
                                    "route_s_m": round(mapped["route_s_m"], 3),
                                    "route_offset_m": round(mapped["route_offset_m"], 3),
                                    "distance_from_base_m": round(float(line.distance_m), 3),
                                    "confidence": round(float(line.confidence), 3),
                                    "source_stamp_ros_s": round(line_stamp, 3),
                                }
                                line_reason = "captured"
                        except (TypeError, ValueError, OverflowError, ZeroDivisionError):
                            line_reason = "stopline_projection_invalid"
            entry["stopline_capture"] = line_reason
            sequence += 1
            print(json.dumps(entry, ensure_ascii=False), flush=True)
            entry["label"] = input("지점 이름 (예: 양보선, 충돌중심): ").strip() or "point_%03d" % sequence
            with destination.open("a", encoding="utf-8") as output:
                output.write(json.dumps(entry, ensure_ascii=False) + "\n")
            print("저장: " + str(destination), flush=True)
    except (EOFError, KeyboardInterrupt):
        print("기록 종료", flush=True)


if __name__ == "__main__":
    main()
