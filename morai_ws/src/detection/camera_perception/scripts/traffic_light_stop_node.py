#!/usr/bin/env python3
"""Publish directional evidence and conservative timestamp-checked stop state."""

import math
import threading
import time

import rospy
from common.msg import ObjectInfoArray
from morai_perception_msgs.msg import TrafficLight
from std_msgs.msg import Bool

from camera_perception.traffic_signal import (
    TrafficSignalStopLatch, directional_observation, straight_observation,
)


class TrafficLightStopNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.shutting_down = False
        self.last_ros_now = None
        self.input_timeout = float(rospy.get_param("~input_timeout_sec", 0.8))
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.5))
        if (not math.isfinite(self.input_timeout) or self.input_timeout <= 0
                or not math.isfinite(self.min_confidence) or not 0 <= self.min_confidence <= 1):
            raise ValueError("Invalid traffic-light timeout/confidence")
        input_topic = rospy.get_param("~input_topic", "/detection/traffic_light")
        output_topic = rospy.get_param(
            "~output_topic", "/perception/traffic_light/stop_required"
        )
        state_topic = rospy.get_param(
            "~state_topic", "/perception/traffic_light/state"
        )
        clear_confirmation_s = float(
            rospy.get_param("~clear_confirmation_s", 0.5)
        )
        self.publisher = rospy.Publisher(output_topic, Bool, queue_size=1, latch=True)
        self.state_publisher = rospy.Publisher(
            state_topic, TrafficLight, queue_size=1, latch=True
        )
        self.directional_publisher = rospy.Publisher(
            rospy.get_param("~directional_state_topic", "/perception/traffic_light/directional_state"),
            TrafficLight, queue_size=1,
        )
        self.latch = TrafficSignalStopLatch(clear_confirmation_s, self.input_timeout)
        self.stop_required = False
        rospy.on_shutdown(self.shutdown)
        self.publisher.publish(Bool(data=False))
        self.state_publisher.publish(self.state_message("UNKNOWN", 0.0, False))
        rospy.Subscriber(input_topic, ObjectInfoArray, self.callback, queue_size=32)
        rospy.logwarn(
            "Traffic-light stop: input=%s output=%s explicit_green_confirmation=%.2fs",
            input_topic,
            output_topic,
            clear_confirmation_s,
        )

    def callback(self, message):
        with self.lock:
            if not self.shutting_down:
                self.process_observation(message)

    def process_observation(self, message):
        state, confidence = directional_observation(message.objects, self.min_confidence)
        self.directional_publisher.publish(self.state_message(
            state, confidence, state != "UNKNOWN", getattr(message, "header", None)))
        class_names = [str(getattr(item, "class_name", "UNKNOWN")) for item in message.objects]
        state, confidence = straight_observation(message.objects, self.min_confidence)
        header = getattr(message, "header", None)
        stamp = header.stamp.to_sec() if header is not None else 0.0
        ros_now, received = rospy.get_time(), time.monotonic()
        reset = self.last_ros_now is not None and ros_now < self.last_ros_now
        self.last_ros_now = ros_now
        if reset or not (math.isfinite(stamp) and math.isfinite(ros_now)
                         and stamp > 0 and -0.05 <= ros_now - stamp <= self.input_timeout):
            stop_required = self.latch.revoke(reset_clock=reset)
            state, confidence = "UNKNOWN", 0.0
        else:
            stop_required = self.latch.observe(state, stamp, received, detected=bool(class_names))
        if stop_required != self.stop_required:
            rospy.logwarn(
                "[TRAFFIC LIGHT] %s classes=%s",
                "STOP/WAIT" if stop_required else "GREEN CONFIRMED",
                ",".join(class_names) if class_names else "none",
            )
        self.stop_required = stop_required
        self.publisher.publish(Bool(data=stop_required))
        # This topic describes the current frame; the controller independently
        # confirms GREEN. The Bool latch is never evidence of permission.
        self.state_publisher.publish(
            self.state_message(
                state, confidence, state != "UNKNOWN",
                getattr(message, "header", None),
            )
        )

    @staticmethod
    def observed_signal(objects):
        return straight_observation(objects)

    @staticmethod
    def state_message(state, confidence, valid, source_header=None):
        output = TrafficLight()
        if source_header is not None:
            output.header.seq = source_header.seq
            output.header.stamp = source_header.stamp
            output.header.frame_id = source_header.frame_id
        # Missing/zero stamps are unknown, not evidence observed at callback
        # time. Preserve old stamps too so consumers can enforce freshness.
        if not output.header.frame_id:
            output.header.frame_id = "front_camera"
        output.state = str(state)
        output.confidence = float(confidence)
        output.valid = bool(valid)
        return output

    def shutdown(self):
        with self.lock:
            self.shutting_down = True
            self.latch.revoke()
            self.publisher.publish(Bool(data=True))
            self.state_publisher.publish(self.state_message("UNKNOWN", 0.0, False))
            self.directional_publisher.publish(self.state_message("UNKNOWN", 0.0, False))


def main():
    rospy.init_node("traffic_light_stop")
    TrafficLightStopNode()
    rospy.spin()


if __name__ == "__main__":
    main()
