#!/usr/bin/env python3
"""카메라·LiDAR 장애물 배열을 시간과 base_link 위치 기준으로 융합한다."""

from __future__ import annotations

import copy
import math
import time
from typing import Optional

import rospy
from morai_perception_msgs.msg import ObstacleArray, SafetyStop


class ObstacleFusion:
    def __init__(self) -> None:
        rospy.init_node("obstacle_fusion", anonymous=False)
        self.lidar_topic = rospy.get_param("~lidar_topic", "/detection/lidar_obstacles")
        self.camera_topic = rospy.get_param("~camera_topic", "/detection/camera_obstacles")
        self.output_topic = rospy.get_param("~output_topic", "/detection/fused_obstacles")
        self.safety_topic = rospy.get_param("~safety_topic", "/detection/fused_safety_stop")
        self.lidar_timeout = float(rospy.get_param("~lidar_timeout_sec", 0.5))
        self.camera_timeout = float(rospy.get_param("~camera_timeout_sec", 0.5))
        self.stop_distance = float(rospy.get_param("~stop_distance_m", 6.0))
        self.match_distance = float(rospy.get_param("~match_distance_m", 1.5))
        self.require_lidar = bool(rospy.get_param("~require_lidar", True))

        self.lidar: Optional[ObstacleArray] = None
        self.camera: Optional[ObstacleArray] = None
        self.lidar_time = 0.0
        self.camera_time = 0.0
        self.obstacle_pub = rospy.Publisher(self.output_topic, ObstacleArray, queue_size=2)
        self.safety_pub = rospy.Publisher(self.safety_topic, SafetyStop, queue_size=2)
        rospy.Subscriber(self.lidar_topic, ObstacleArray, self.lidar_callback, queue_size=2)
        rospy.Subscriber(self.camera_topic, ObstacleArray, self.camera_callback, queue_size=2)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish)

    def lidar_callback(self, msg: ObstacleArray) -> None:
        self.lidar = msg
        self.lidar_time = time.monotonic()

    def camera_callback(self, msg: ObstacleArray) -> None:
        self.camera = msg
        self.camera_time = time.monotonic()

    @staticmethod
    def close(a, b, threshold: float) -> bool:
        return math.hypot(a.longitudinal_m - b.longitudinal_m, a.lateral_m - b.lateral_m) <= threshold

    def publish(self, _event) -> None:
        now = time.monotonic()
        lidar_fresh = self.lidar is not None and now - self.lidar_time <= self.lidar_timeout
        camera_fresh = self.camera is not None and now - self.camera_time <= self.camera_timeout
        fused = ObstacleArray()
        fused.header.stamp = rospy.Time.now()
        fused.header.frame_id = "base_link"

        if lidar_fresh:
            fused.obstacles = [copy.deepcopy(obstacle) for obstacle in self.lidar.obstacles]
        if camera_fresh:
            for camera_obstacle in self.camera.obstacles:
                match = next(
                    (candidate for candidate in fused.obstacles if self.close(candidate, camera_obstacle, self.match_distance)),
                    None,
                )
                if match is None:
                    fused.obstacles.append(copy.deepcopy(camera_obstacle))
                else:
                    match.confidence = min(1.0, max(match.confidence, camera_obstacle.confidence) + 0.1)
                    match.source = "camera+lidar"
                    match.stop_required = match.stop_required or camera_obstacle.stop_required

        self.obstacle_pub.publish(fused)

        nearest = min((obstacle.distance_m for obstacle in fused.obstacles), default=float("inf"))
        stop_required = any(obstacle.stop_required for obstacle in fused.obstacles)
        if nearest <= self.stop_distance and fused.obstacles:
            stop_required = True
        if self.require_lidar and not lidar_fresh:
            stop_required = True

        safety = SafetyStop()
        safety.header = fused.header
        safety.stop_required = stop_required
        safety.distance_m = nearest if math.isfinite(nearest) else -1.0
        safety.confidence = max((obstacle.confidence for obstacle in fused.obstacles), default=0.0)
        if not lidar_fresh and self.require_lidar:
            safety.reason = "lidar_stale"
        elif stop_required:
            safety.reason = "fused_obstacle"
        else:
            safety.reason = "fused_clear"
        self.safety_pub.publish(safety)


if __name__ == "__main__":
    try:
        ObstacleFusion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
