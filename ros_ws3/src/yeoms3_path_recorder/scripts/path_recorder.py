#!/usr/bin/env python3
import csv
import math
import os

import rospy
import tf.transformations
from geometry_msgs.msg import PoseStamped, TwistStamped


class PathRecorder:
    def __init__(self):
        self.pose_topic = rospy.get_param("~pose_topic", "/localization/ego_pose")
        self.twist_topic = rospy.get_param("~twist_topic", "/localization/ego_twist")
        self.output_file = os.path.expanduser(rospy.get_param("~output_file", "~/morai_recorded_paths/01.csv"))
        self.min_spacing = float(rospy.get_param("~min_spacing_m", 0.4))
        self.target_speed = float(rospy.get_param("~target_speed_mps", 1.0))

        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)
        self.fp = open(self.output_file, "w", newline="")
        self.writer = csv.DictWriter(self.fp, fieldnames=["stamp", "x", "y", "z", "yaw", "speed", "target_speed"])
        self.writer.writeheader()

        self.last_x = None
        self.last_y = None
        self.speed = 0.0
        self.count = 0

        rospy.Subscriber(self.pose_topic, PoseStamped, self._pose_cb, queue_size=1)
        rospy.Subscriber(self.twist_topic, TwistStamped, self._twist_cb, queue_size=1)
        rospy.on_shutdown(self._close)
        rospy.loginfo("Recording path to %s", self.output_file)

    def _twist_cb(self, msg):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.speed = math.hypot(vx, vy)

    def _pose_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        if self.last_x is not None and math.hypot(x - self.last_x, y - self.last_y) < self.min_spacing:
            return

        q = msg.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.writer.writerow({
            "stamp": "%.6f" % msg.header.stamp.to_sec(),
            "x": "%.6f" % x,
            "y": "%.6f" % y,
            "z": "%.6f" % z,
            "yaw": "%.6f" % yaw,
            "speed": "%.6f" % self.speed,
            "target_speed": "%.6f" % self.target_speed,
        })
        self.fp.flush()
        self.last_x = x
        self.last_y = y
        self.count += 1
        rospy.loginfo_throttle(2.0, "recorded %d points to %s", self.count, self.output_file)

    def _close(self):
        if not self.fp.closed:
            self.fp.flush()
            self.fp.close()


if __name__ == "__main__":
    rospy.init_node("path_recorder")
    PathRecorder()
    rospy.spin()
