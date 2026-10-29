#!/usr/bin/env python3
"""The global route around the ego as highway_lane_strategy's base path.

final_ws_curvature_signal.launch has no avoidance path manager.  This node
publishes the route of ~path_file from ~back_m behind to ~ahead_m ahead of the
ego on ~base_path_topic at ~rate_hz, and False on ~base_stop_topic, so that
highway_lane_strategy has the fresh base path it passes through outside its
zone, follows until it has the camera lane, and hands over to.
"""
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool

from purepursuit_mgeo.highway_overtake import Route
from purepursuit_mgeo.highway_section import route_window
from purepursuit_mgeo.path import load_mgeo_path


class HighwayBasePath:
    def __init__(self):
        self.route = Route([(p.x, p.y) for p in load_mgeo_path(rospy.get_param("~path_file"))])
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.back_m = float(rospy.get_param("~back_m", 10.0))
        self.ahead_m = float(rospy.get_param("~ahead_m", 150.0))
        self.pose_timeout_s = float(rospy.get_param("~pose_timeout_s", 0.5))
        self.lock = threading.Lock()
        self.pose = None
        self.path_pub = rospy.Publisher(
            rospy.get_param("~base_path_topic", "/highway_lane_strategy/base_path"), Path, queue_size=1)
        self.stop_pub = rospy.Publisher(
            rospy.get_param("~base_stop_topic", "/highway_lane_strategy/base_stop"), Bool, queue_size=1)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/localization/odometry"),
                         Odometry, self._odom_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/float(rospy.get_param("~rate_hz", 10.0))), self._tick)

    def _odom_cb(self, msg):
        position = msg.pose.pose.position
        with self.lock:
            self.pose = (rospy.get_time(), position.x, position.y)

    def _tick(self, _event):
        with self.lock:
            pose = self.pose
        if pose is None or rospy.get_time() - pose[0] > self.pose_timeout_s:
            return                      # no fresh pose: the base path goes stale
        s, _ = self.route.project(pose[1], pose[2])
        path = Path()
        path.header.stamp, path.header.frame_id = rospy.Time.now(), self.map_frame
        for x, y in route_window(self.route.points, self.route.s, s, self.back_m, self.ahead_m):
            point = PoseStamped()
            point.header = path.header
            point.pose.position.x, point.pose.position.y = x, y
            point.pose.orientation.w = 1.0
            path.poses.append(point)
        self.path_pub.publish(path)
        self.stop_pub.publish(Bool(data=False))


if __name__ == "__main__":
    rospy.init_node("highway_base_path")
    HighwayBasePath()
    rospy.spin()
