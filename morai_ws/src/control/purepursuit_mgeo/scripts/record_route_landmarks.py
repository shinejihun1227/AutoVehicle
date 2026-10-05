#!/usr/bin/env python3
"""Press Enter while driving to record map XY and route progress for calibration."""

from __future__ import annotations

import argparse
import json
import math
import threading
from pathlib import Path

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="JSON-lines file for captured points")
    parser.add_argument("--odom-topic", default="/localization/odometry")
    parser.add_argument("--progress-topic", default="/experimental/curvature_progress")
    parser.add_argument("--max-age-s", type=float, default=0.6)
    args = parser.parse_args()
    if args.max_age_s <= 0.0:
        parser.error("--max-age-s must be positive")

    rospy.init_node("record_route_landmarks", anonymous=True, disable_signals=True)
    lock = threading.Lock()
    latest = {"odom": None, "odom_at": None, "progress": None, "progress_at": None}

    def on_odom(message):
        if message.header.frame_id != "map":
            return
        with lock:
            latest["odom"] = message
            latest["odom_at"] = rospy.Time.now()

    def on_progress(message):
        if not math.isfinite(float(message.data)):
            return
        with lock:
            latest["progress"] = float(message.data)
            latest["progress_at"] = rospy.Time.now()

    rospy.Subscriber(args.odom_topic, Odometry, on_odom, queue_size=1)
    rospy.Subscriber(args.progress_topic, Float64, on_progress, queue_size=1)
    destination = Path(args.output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    print("지점에서 Enter를 누르세요. 입력 즉시 위치를 고정합니다. Ctrl+C로 종료합니다.", flush=True)
    sequence = 0
    try:
        while not rospy.is_shutdown():
            input("capture> ")
            with lock:
                snapshot = dict(latest)
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
