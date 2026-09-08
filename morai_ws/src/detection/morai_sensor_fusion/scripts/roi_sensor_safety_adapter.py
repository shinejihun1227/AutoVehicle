#!/usr/bin/env python3
"""Adapt ROI-branch LiDAR/camera safety signals to the current SafetyStop contract.

The ROI LiDAR tracker publishes confirmed obstacle centers in the map frame.
This node transforms those centers into ``base_link`` with the current EKF pose,
checks a conservative forward corridor, and combines that result with the
camera team's Boolean stop signals.  The driving controller itself remains the
workspace's curvature-speed PI controller.
"""

from __future__ import annotations

import math
import time
from typing import Dict, Optional

import rospy
from lidar_perception.msg import LidarObstacleArray
from morai_perception_msgs.msg import SafetyStop
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def quaternion_to_yaw(orientation) -> float:
    sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(sin_yaw, cos_yaw)


class RoiSensorSafetyAdapter:
    def __init__(self) -> None:
        rospy.init_node("roi_sensor_safety_adapter", anonymous=False)

        self.lidar_topic = rospy.get_param(
            "~lidar_topic", "/perception/lidar/tracked_obstacles_map"
        )
        self.odometry_topic = rospy.get_param(
            "~odometry_topic", "/localization/odometry"
        )
        self.safety_topic = rospy.get_param(
            "~safety_topic", "/detection/fused_safety_stop"
        )
        self.stop_distance_m = float(rospy.get_param("~stop_distance_m", 8.0))
        self.forward_half_width_m = float(
            rospy.get_param("~forward_half_width_m", 1.5)
        )
        self.input_timeout_sec = float(
            rospy.get_param("~input_timeout_sec", 0.5)
        )
        self.require_fresh_lidar = bool(
            rospy.get_param("~require_fresh_lidar", True)
        )
        self.require_fresh_camera_stops = bool(rospy.get_param("~require_fresh_camera_stops", False))
        self.require_source_stamps = bool(rospy.get_param("~require_source_stamps", False))
        self.front_reference_offset_m = float(rospy.get_param("~front_reference_offset_m", 0.0))
        self.camera_updated = {}

        self.latest_lidar: Optional[LidarObstacleArray] = None
        self.latest_odom: Optional[Odometry] = None
        self.last_lidar_at = 0.0
        self.last_odom_at = 0.0
        self.camera_stops: Dict[str, bool] = {
            "traffic_light": False,
            "pedestrian": False,
            "intersection": False,
        }

        self.publisher = rospy.Publisher(
            self.safety_topic, SafetyStop, queue_size=2
        )
        rospy.Subscriber(
            self.lidar_topic, LidarObstacleArray, self.lidar_callback, queue_size=1
        )
        rospy.Subscriber(
            self.odometry_topic, Odometry, self.odom_callback, queue_size=1
        )
        for key, parameter in (
            ("traffic_light", "~traffic_stop_topic"),
            ("pedestrian", "~pedestrian_stop_topic"),
            ("intersection", "~intersection_stop_topic"),
        ):
            topic = rospy.get_param(parameter, "")
            if topic:
                self.camera_updated[key] = None
                rospy.Subscriber(
                    topic,
                    Bool,
                    self._camera_stop_callback,
                    callback_args=key,
                    queue_size=1,
                )

        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_safety)
        rospy.loginfo(
            "ROI safety adapter: lidar=%s odom=%s output=%s distance=%.1fm",
            self.lidar_topic,
            self.odometry_topic,
            self.safety_topic,
            self.stop_distance_m,
        )

    def lidar_callback(self, message: LidarObstacleArray) -> None:
        if self.require_source_stamps and not self.source_fresh(message):
            return
        self.latest_lidar = message
        self.last_lidar_at = time.monotonic()

    def odom_callback(self, message: Odometry) -> None:
        if self.require_source_stamps and not self.source_fresh(message):
            return
        self.latest_odom = message
        self.last_odom_at = time.monotonic()

    def _camera_stop_callback(self, message: Bool, key: str) -> None:
        self.camera_stops[key] = bool(message.data)
        self.camera_updated[key] = time.monotonic()

    def source_fresh(self, message):
        age = rospy.get_time() - message.header.stamp.to_sec()
        return (message.header.stamp.to_sec() > 0 and -0.05 <= age <= self.input_timeout_sec
                and message.header.frame_id == "map")

    def nearest_forward_obstacle(self) -> Optional[float]:
        if self.latest_lidar is None or self.latest_odom is None:
            return None

        pose = self.latest_odom.pose.pose
        yaw = quaternion_to_yaw(pose.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        nearest = None

        for obstacle in self.latest_lidar.obstacles:
            dx = float(obstacle.center_x_map) - float(pose.position.x)
            dy = float(obstacle.center_y_map) - float(pose.position.y)
            forward_x = cos_yaw * dx + sin_yaw * dy
            lateral_y = -sin_yaw * dx + cos_yaw * dy
            # Rotated box extents: compare obstacle FRONT FACE to ego bumper,
            # not center-to-origin distance (which misses long crossing cars).
            relative_yaw = float(getattr(obstacle, "yaw", yaw)) - yaw
            half_length = max(0.0, float(getattr(obstacle, "length", 0.0))) * 0.5
            half_width = max(0.0, float(obstacle.width)) * 0.5
            obstacle_half_width = abs(math.sin(relative_yaw)) * half_length + abs(math.cos(relative_yaw)) * half_width
            obstacle_half_length = abs(math.cos(relative_yaw)) * half_length + abs(math.sin(relative_yaw)) * half_width
            corridor_half_width = self.forward_half_width_m + obstacle_half_width
            bumper_gap = forward_x - obstacle_half_length - self.front_reference_offset_m
            if forward_x + obstacle_half_length > 0.0 and bumper_gap <= self.stop_distance_m and abs(lateral_y) <= corridor_half_width:
                distance = max(0.0, bumper_gap)
                if nearest is None or distance < nearest:
                    nearest = distance
        return nearest

    def publish_safety(self, _event) -> None:
        now = time.monotonic()
        lidar_fresh = (
            self.latest_lidar is not None
            and now - self.last_lidar_at <= self.input_timeout_sec
            and (not self.require_source_stamps or self.source_fresh(self.latest_lidar))
        )
        odom_fresh = (
            self.latest_odom is not None
            and now - self.last_odom_at <= self.input_timeout_sec
            and (not self.require_source_stamps or self.source_fresh(self.latest_odom))
        )
        nearest = self.nearest_forward_obstacle() if lidar_fresh and odom_fresh else None

        camera_reason = next(
            (name for name, active in self.camera_stops.items() if active), None
        )
        if camera_reason is None and self.require_fresh_camera_stops:
            camera_reason = next((name + "_stale" for name, updated in self.camera_updated.items()
                                  if updated is None or not 0 <= now - updated <= self.input_timeout_sec), None)
        stop_required = nearest is not None or camera_reason is not None
        reason = "fused_clear"
        confidence = 0.0

        if camera_reason is not None:
            stop_required = True
            reason = "camera_" + camera_reason
            confidence = 1.0
        elif nearest is not None:
            reason = "roi_lidar_forward_obstacle"
            confidence = 0.8
        elif self.require_fresh_lidar and not lidar_fresh:
            stop_required = True
            reason = "roi_lidar_stale"
        elif not odom_fresh:
            stop_required = True
            reason = "roi_localization_stale"

        message = SafetyStop()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "base_link"
        message.stop_required = stop_required
        message.distance_m = float(nearest) if nearest is not None else -1.0
        message.confidence = confidence
        message.reason = reason
        self.publisher.publish(message)


if __name__ == "__main__":
    try:
        RoiSensorSafetyAdapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
