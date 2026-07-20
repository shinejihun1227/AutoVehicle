#!/usr/bin/env python3
import math
import time

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Float32, String


ROI_FIELDS = [
    PointField("x", 0, PointField.FLOAT32, 1),
    PointField("y", 4, PointField.FLOAT32, 1),
    PointField("z", 8, PointField.FLOAT32, 1),
    PointField("intensity", 12, PointField.FLOAT32, 1),
]


class LidarObstacleDetector:
    def __init__(self):
        self.input_topic = self._param("input_topic", "/sensors/lidar/points")
        self.roi_topic = self._param("roi_topic", "/perception/lidar/roi_points")
        self.nearest_topic = self._param("nearest_topic", "/perception/lidar/nearest_obstacle_m")
        self.stop_required_topic = self._param("stop_required_topic", "/perception/lidar/obstacle_stop_required")
        self.summary_topic = self._param("summary_topic", "/perception/lidar/obstacle_summary")
        self.frame_id = self._param("frame_id", "velodyne")
        self.process_rate_hz = float(self._param("process_rate_hz", 10.0))
        self.x_min = float(self._param("x_min_m", 0.5))
        self.x_max = float(self._param("x_max_m", 30.0))
        self.y_abs_max = float(self._param("y_abs_max_m", 3.0))
        self.z_min = float(self._param("z_min_m", -1.5))
        self.z_max = float(self._param("z_max_m", 2.0))
        self.stop_distance = float(self._param("stop_distance_m", 7.0))
        self.min_roi_points = int(self._param("min_roi_points", 5))
        self.publish_roi_points = bool(self._param("publish_roi_points", True))
        self.last_process_time = 0.0

        self.nearest_pub = rospy.Publisher(self.nearest_topic, Float32, queue_size=1)
        self.stop_pub = rospy.Publisher(self.stop_required_topic, Bool, queue_size=1)
        self.summary_pub = rospy.Publisher(self.summary_topic, String, queue_size=1)
        self.roi_pub = rospy.Publisher(self.roi_topic, PointCloud2, queue_size=1)
        rospy.Subscriber(self.input_topic, PointCloud2, self._cloud_cb, queue_size=1)
        rospy.loginfo("LiDAR obstacle detector subscribed to %s", self.input_topic)

    def _cloud_cb(self, msg):
        if not self._rate_limit_allows_process():
            return

        roi_points = []
        nearest = float("inf")
        for point in pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            x, y, z, intensity = point
            if not self._inside_roi(x, y, z):
                continue
            distance = math.hypot(x, y)
            nearest = min(nearest, distance)
            roi_points.append((x, y, z, intensity))

        enough_points = len(roi_points) >= self.min_roi_points
        nearest_value = nearest if enough_points else -1.0
        stop_required = enough_points and nearest <= self.stop_distance

        self.nearest_pub.publish(Float32(nearest_value))
        self.stop_pub.publish(Bool(stop_required))
        summary = "roi_points=%d nearest_m=%.3f stop_required=%s roi=x[%.1f,%.1f] y_abs<=%.1f z[%.1f,%.1f]" % (
            len(roi_points),
            nearest_value,
            stop_required,
            self.x_min,
            self.x_max,
            self.y_abs_max,
            self.z_min,
            self.z_max,
        )
        self.summary_pub.publish(String(summary))
        rospy.loginfo_throttle(1.0, summary)

        if self.publish_roi_points:
            self._publish_roi_points(msg.header, roi_points)

    def _inside_roi(self, x, y, z):
        return (
            self.x_min <= x <= self.x_max
            and abs(y) <= self.y_abs_max
            and self.z_min <= z <= self.z_max
        )

    def _publish_roi_points(self, header, roi_points):
        out_header = rospy.Header()
        out_header.stamp = header.stamp
        out_header.frame_id = header.frame_id or self.frame_id
        cloud = pc2.create_cloud(out_header, ROI_FIELDS, roi_points)
        self.roi_pub.publish(cloud)

    def _rate_limit_allows_process(self):
        if self.process_rate_hz <= 0.0:
            return True
        now = time.time()
        if now - self.last_process_time < 1.0 / self.process_rate_hz:
            return False
        self.last_process_time = now
        return True

    @staticmethod
    def _param(name, default):
        private_name = "~" + name
        if rospy.has_param(private_name):
            return rospy.get_param(private_name)
        return rospy.get_param("/lidar_obstacle_detector/" + name, default)


if __name__ == "__main__":
    rospy.init_node("lidar_obstacle_detector")
    LidarObstacleDetector()
    rospy.spin()
