#!/usr/bin/env python3
import csv
import math
import os
import re
from datetime import datetime

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import String


class WaypointRecorder:
    def __init__(self):
        self.pose_topic = rospy.get_param("~pose_topic", "/localization/ego_pose")
        self.twist_topic = rospy.get_param("~twist_topic", "/localization/ego_twist")
        self.output_file = rospy.get_param("~output_file", "")
        self.min_distance_m = float(rospy.get_param("~min_distance_m", 0.5))
        self.min_time_s = float(rospy.get_param("~min_time_s", 0.2))
        self.target_speed_mps = float(rospy.get_param("~target_speed_mps", 1.0))
        self.use_measured_speed = bool(rospy.get_param("~use_measured_speed", False))
        self.min_record_speed_mps = float(rospy.get_param("~min_record_speed_mps", 0.05))
        self.frame_id = rospy.get_param("~frame_id", "map")

        self.speed = 0.0
        self.last_x = None
        self.last_y = None
        self.last_time = None
        self.rows = []
        self.origin_lat = None
        self.origin_lon = None

        self.output_file = self._resolve_output_file(self.output_file)
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        self.fp = open(self.output_file, "w", newline="")
        self.writer = csv.DictWriter(self.fp, fieldnames=["x", "y", "target_speed", "lat", "lon"])
        self.writer.writeheader()
        self.fp.flush()

        self.path_pub = rospy.Publisher("/planning/recorded_path", Path, queue_size=1, latch=True)
        rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=1)
        rospy.Subscriber(self.twist_topic, TwistStamped, self._twist_cb, queue_size=1)
        rospy.Subscriber("/udp_bridge/gps_debug", String, self._gps_debug_cb, queue_size=1)

        rospy.on_shutdown(self._close)
        rospy.loginfo("Waypoint recorder writing to %s", self.output_file)

    def _resolve_output_file(self, output_file):
        if output_file:
            return os.path.expanduser(output_file)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.expanduser(f"~/morai_recorded_paths/path_{stamp}.csv")

    def _twist_cb(self, msg):
        self.speed = math.hypot(msg.twist.linear.x, msg.twist.linear.y)

    def _gps_debug_cb(self, msg):
        lat_match = re.search(r"lat=(-?\d+(?:\.\d+)?)", msg.data)
        lon_match = re.search(r"lon=(-?\d+(?:\.\d+)?)", msg.data)
        if lat_match and lon_match:
            lat = float(lat_match.group(1))
            lon = float(lon_match.group(1))
            if self.origin_lat is None:
                self.origin_lat = lat
                self.origin_lon = lon
                rospy.loginfo("record origin lat=%.8f lon=%.8f", self.origin_lat, self.origin_lon)

    def _pose_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        now = msg.header.stamp if msg.header.stamp != rospy.Time(0) else rospy.Time.now()

        if self.speed < self.min_record_speed_mps and self.rows:
            return

        if not self._should_record(x, y, now):
            return

        target_speed = self.speed if self.use_measured_speed else self.target_speed_mps
        if target_speed <= 0.0:
            target_speed = self.target_speed_mps

        row = {
            "x": "%.6f" % x,
            "y": "%.6f" % y,
            "target_speed": "%.3f" % target_speed,
            "lat": "%.8f" % self.origin_lat if self.origin_lat is not None else "",
            "lon": "%.8f" % self.origin_lon if self.origin_lon is not None else "",
        }
        self.writer.writerow(row)
        self.fp.flush()
        self.rows.append((x, y, target_speed))
        self.last_x = x
        self.last_y = y
        self.last_time = now
        self._publish_path(msg.header)
        rospy.loginfo_throttle(1.0, "recorded waypoints=%d file=%s", len(self.rows), self.output_file)

    def _should_record(self, x, y, now):
        if self.last_x is None:
            return True

        distance = math.hypot(x - self.last_x, y - self.last_y)
        elapsed = (now - self.last_time).to_sec() if self.last_time is not None else float("inf")
        return distance >= self.min_distance_m and elapsed >= self.min_time_s

    def _publish_path(self, header):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = header.frame_id or self.frame_id
        for x, y, _ in self.rows:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    def _close(self):
        if not self.fp.closed:
            self.fp.flush()
            self.fp.close()
        rospy.loginfo("Waypoint recording finished: %s waypoints=%d", self.output_file, len(self.rows))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("waypoint_recorder")
    WaypointRecorder().run()
