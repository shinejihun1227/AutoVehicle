#!/usr/bin/env python3
"""YOLO 신호등 객체를 GREEN 우선 정지 Bool 토픽으로 변환한다."""

import math
import time

import rospy
from common.msg import ObjectInfoArray
from morai_perception_msgs.msg import TrafficLight
from std_msgs.msg import Bool

from camera_perception.traffic_signal import TrafficSignalStopLatch, directional_observation


class TrafficLightStopNode:
    def __init__(self):
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
        self.latch = TrafficSignalStopLatch(clear_confirmation_s)
        self.stop_required = False
        rospy.Subscriber(input_topic, ObjectInfoArray, self.callback, queue_size=1)
        rospy.on_shutdown(self.shutdown)
        self.publisher.publish(Bool(data=False))
        self.state_publisher.publish(self.state_message("UNKNOWN", 0.0, False))
        rospy.logwarn(
            "Traffic-light stop: input=%s output=%s priority=GREEN>RED-only/Yellow/Amber "
            "clear_confirmation=%.2fs",
            input_topic,
            output_topic,
            clear_confirmation_s,
        )

    def callback(self, message):
        state, confidence = directional_observation(message.objects)
        self.directional_publisher.publish(self.state_message(
            state, confidence, state != "UNKNOWN", getattr(message, "header", None)))
        class_names = [item.class_name for item in message.objects]
        stop_required = self.latch.update(class_names, time.monotonic())
        if stop_required != self.stop_required:
            rospy.logwarn(
                "[TRAFFIC LIGHT] %s classes=%s",
                "STOP (RED-only/YELLOW)" if stop_required else "GO (GREEN priority/clear)",
                ",".join(class_names) if class_names else "none",
            )
        self.stop_required = stop_required
        self.publisher.publish(Bool(data=stop_required))
        # Timestamped state describes this observation, never the Bool latch's
        # history. Keep the existing GREEN/yellow/red-only class policy.
        state, confidence = self.observed_signal(message.objects)
        self.state_publisher.publish(
            self.state_message(
                state, confidence, state != "UNKNOWN",
                getattr(message, "header", None),
            )
        )

    @staticmethod
    def observed_signal(objects):
        normalized = [(str(item.class_name).strip().lower(), item) for item in objects]
        for state, matches in (
            ("GREEN", lambda name: "green" in name),
            ("YELLOW", lambda name: "yellow" in name or "amber" in name),
            ("RED", lambda name: name == "red"),
        ):
            evidence = [item for name, item in normalized if matches(name)]
            if evidence:
                scores = [float(item.conf) for item in evidence]
                confidence = max(
                    (score for score in scores if math.isfinite(score) and score >= 0.0),
                    default=0.0,
                )
                return state, confidence
        return "UNKNOWN", 0.0

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
        self.publisher.publish(Bool(data=False))
        self.state_publisher.publish(self.state_message("UNKNOWN", 0.0, False))


def main():
    rospy.init_node("traffic_light_stop")
    TrafficLightStopNode()
    rospy.spin()


if __name__ == "__main__":
    main()
