#!/usr/bin/env python3
"""PointCloud2를 base_link 기준 전방 ROI로 필터링하는 초기 장애물 검출기.

정교한 군집화 전 단계로, ROI 안의 유효 point 수와 가장 가까운 point를
이용해 안전정지 요청을 만든다. 실제 센서 장착 TF와 ROI는 현장 데이터로 튜닝한다.
"""

from __future__ import annotations

import math
from typing import Iterable, List, Optional, Tuple

import rospy
import tf2_ros
from morai_perception_msgs.msg import Obstacle, ObstacleArray, SafetyStop
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32


def rotate_vector(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    # quaternion * vector * quaternion inverse
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


class LidarObstacleDetector:
    def __init__(self) -> None:
        rospy.init_node("lidar_obstacle_detector", anonymous=False)
        self.point_topic = rospy.get_param("~point_topic", "/lidar3D")
        self.obstacle_topic = rospy.get_param("~obstacle_topic", "/detection/lidar_obstacles")
        self.safety_topic = rospy.get_param("~safety_topic", "/detection/lidar_safety_stop")
        self.nearest_topic = rospy.get_param("~nearest_topic", "/perception/lidar/nearest_obstacle_m")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.input_frame = rospy.get_param("~input_frame", "velodyne")
        self.x_min = float(rospy.get_param("~roi_x_min_m", 0.2))
        self.x_max = float(rospy.get_param("~roi_x_max_m", 20.0))
        self.y_abs_max = float(rospy.get_param("~roi_y_abs_max_m", 3.0))
        self.z_min = float(rospy.get_param("~roi_z_min_m", 0.05))
        self.z_max = float(rospy.get_param("~roi_z_max_m", 2.0))
        self.stop_distance = float(rospy.get_param("~stop_distance_m", 6.0))
        self.min_points = int(rospy.get_param("~min_points", 5))
        self.tf_timeout_sec = float(rospy.get_param("~tf_timeout_sec", 0.05))
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.obstacle_pub = rospy.Publisher(self.obstacle_topic, ObstacleArray, queue_size=2)
        self.safety_pub = rospy.Publisher(self.safety_topic, SafetyStop, queue_size=2)
        self.nearest_pub = rospy.Publisher(self.nearest_topic, Float32, queue_size=2)
        rospy.Subscriber(self.point_topic, PointCloud2, self.callback, queue_size=1)

        rospy.loginfo(
            "LiDAR detector point=%s frame=%s->%s ROI x=[%.1f, %.1f] y_abs=%.1f stop=%.1fm",
            self.point_topic, self.input_frame, self.base_frame,
            self.x_min, self.x_max, self.y_abs_max, self.stop_distance,
        )

    def transform_points(self, msg: PointCloud2, points: Iterable[Tuple[float, float, float]]) -> Optional[List[Tuple[float, float, float]]]:
        source_frame = msg.header.frame_id or self.input_frame
        if source_frame == self.base_frame:
            return list(points)
        try:
            transform = self.buffer.lookup_transform(
                self.base_frame,
                source_frame,
                msg.header.stamp,
                rospy.Duration(self.tf_timeout_sec),
            )
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "LiDAR TF lookup 실패 %s -> %s: %s", source_frame, self.base_frame, exc)
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        transformed = []
        for x, y, z in points:
            rx, ry, rz = rotate_vector(x, y, z, rotation.x, rotation.y, rotation.z, rotation.w)
            transformed.append((rx + translation.x, ry + translation.y, rz + translation.z))
        return transformed

    def callback(self, msg: PointCloud2) -> None:
        raw_points = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        )
        points = self.transform_points(msg, raw_points)
        if points is None:
            return

        candidates: List[Tuple[float, float, float, float]] = []
        for x, y, z in points:
            if not (self.x_min <= x <= self.x_max):
                continue
            if abs(y) > self.y_abs_max or not (self.z_min <= z <= self.z_max):
                continue
            distance = math.hypot(x, y)
            candidates.append((distance, x, y, z))

        candidates.sort(key=lambda value: value[0])
        nearest = candidates[0][0] if candidates else float("inf")
        stop_required = len(candidates) >= self.min_points and nearest <= self.stop_distance

        obstacle_array = ObstacleArray()
        obstacle_array.header = msg.header
        if not obstacle_array.header.frame_id:
            obstacle_array.header.frame_id = self.base_frame
        if candidates:
            _, x, y, z = candidates[0]
            obstacle = Obstacle()
            obstacle.header = obstacle_array.header
            obstacle.distance_m = nearest
            obstacle.longitudinal_m = x
            obstacle.lateral_m = y
            obstacle.height_m = z
            obstacle.confidence = min(1.0, len(candidates) / float(max(self.min_points * 4, 1)))
            obstacle.stop_required = stop_required
            obstacle.source = "lidar"
            obstacle_array.obstacles.append(obstacle)
        self.obstacle_pub.publish(obstacle_array)

        safety = SafetyStop()
        safety.header = obstacle_array.header
        safety.stop_required = stop_required
        safety.distance_m = nearest if math.isfinite(nearest) else -1.0
        safety.confidence = obstacle_array.obstacles[0].confidence if obstacle_array.obstacles else 0.0
        safety.reason = "lidar_obstacle" if stop_required else "lidar_clear"
        self.safety_pub.publish(safety)
        self.nearest_pub.publish(Float32(data=nearest if math.isfinite(nearest) else -1.0))


if __name__ == "__main__":
    try:
        LidarObstacleDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
