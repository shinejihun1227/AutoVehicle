#!/usr/bin/env python3
"""Timestamp-preserving real_lane JSON -> ROS lane/stopline/semantic messages."""
import json
import threading
import rospy
from std_msgs.msg import Bool, Float64, String
from morai_perception_msgs.msg import LaneDetection, StopLineDetection
from camera_perception.lane_info_contract import convert, fresh_payload


class LaneInfoContractNode:
    def __init__(self):
        self.timeout = float(rospy.get_param('~timeout_sec', 0.6))
        self.confidence = float(rospy.get_param('~min_confidence', 0.45))
        self.lock = threading.RLock()
        self.last_stamp = 0.0
        self.last_info = {}
        self.output = rospy.Publisher('/perception/camera/lane_info', String, queue_size=1)
        self.lane = rospy.Publisher('/detection/lane', LaneDetection, queue_size=1)
        self.line = rospy.Publisher('/perception/camera/stopline', StopLineDetection, queue_size=1)
        self.bools = {key: rospy.Publisher('/perception/camera/' + suffix, Bool, queue_size=1)
                      for key, suffix in dict(dashed='dashed_lane_detected', left_solid='left_solid_lane_detected',
                          left_yellow='left_yellow_solid_lane_detected', right_solid='right_solid_lane_detected').items()}
        self.detected = rospy.Publisher('/perception/camera/stopline_detected', Bool, queue_size=1)
        self.distance = rospy.Publisher('/perception/camera/stopline_distance_m', Float64, queue_size=1)
        rospy.Subscriber('/perception/camera/lane_info_raw', String, self.callback, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.expire)

    def publish(self, info):
        clean, semantics, lane, line = convert(info, rospy.get_time(), self.timeout, self.confidence)
        stamp = rospy.Time.from_sec(float(info.get('timestamp', 0.0)))
        for pub, cls, values in ((self.lane, LaneDetection, lane), (self.line, StopLineDetection, line)):
            msg = cls()
            msg.header.frame_id, msg.header.stamp = 'base_link', stamp
            msg.header.seq = max(0, int(info.get('sequence', 0)))
            for key, value in values.items(): setattr(msg, key, value)
            pub.publish(msg)
        for key, pub in self.bools.items(): pub.publish(Bool(data=semantics[key]))
        self.detected.publish(Bool(data=line['valid']))
        self.distance.publish(Float64(data=line['distance_m'] if line['valid'] else float('nan')))
        self.output.publish(String(data=json.dumps(clean, allow_nan=False)))

    def callback(self, msg):
        with self.lock:
            try:
                info = json.loads(msg.data, parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))
                if not fresh_payload(info, rospy.get_time(), self.timeout):
                    raise ValueError('stale stamp, wrong frame or coordinate convention')
                if info['timestamp'] <= self.last_stamp:
                    return  # Replaying JSON cannot renew its observation time.
                self.publish(info)
                self.last_info, self.last_stamp = info, info['timestamp']
            except (TypeError, ValueError, KeyError, AttributeError, OverflowError) as exc:
                rospy.logwarn_throttle(2.0, 'lane_info rejected: %s', exc)
                self.last_info = {}
                self.publish({})

    def expire(self, _):
        with self.lock:
            now = rospy.get_time()
            if now < self.last_stamp - 0.05:
                self.last_stamp, self.last_info = 0.0, {}
            if not fresh_payload(self.last_info, now, self.timeout):
                self.publish(self.last_info)  # Invalid heartbeat keeps ORIGINAL source stamp.


if __name__ == '__main__':
    rospy.init_node('lane_info_contract')
    LaneInfoContractNode()
    rospy.spin()
