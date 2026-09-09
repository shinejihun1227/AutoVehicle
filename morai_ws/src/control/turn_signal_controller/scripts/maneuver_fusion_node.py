#!/usr/bin/env python3
"""Final type-1 command overlay: route, directional lights, stop line and safety.

The route remains the sole steering plan. This node owns stop-line braking
when enabled, downstream of lane fallback, and transmits the separate MORAI
lamp protocol. No network transmission occurs when lamp_output_enabled=false.
"""

import copy
import json
import math
import socket
import threading
import time
from collections import deque
from pathlib import Path

import rospy
from morai_msgs.msg import CtrlCmd
from morai_perception_msgs.msg import TrafficLight, StopLineDetection, SafetyStop, SensorQuality
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import String
from common.msg import ObjectInfoArray

from curvature_speed_purepursuit.planner import (
    load_path_file, clean_consecutive_duplicates, cumulative_arc_lengths, nearest_projection,
)
from stopline_control.core import AccelRiseLimiter, StopLineControllerCore, Sample, Decision, finite, clamp
from turn_signal_controller.fusion import (
    IndicatorLead, build_lamp_packet, route_intent, signal_permits, signal_allowed_directions,
)
from turn_signal_controller.route_contract import reference_path_reason, route_event_key, route_checked_maneuvers
from turn_signal_controller.scheduler import parse_maneuvers
from turn_signal_controller.route_context import load_route_contexts
from turn_signal_controller.turn_motion import TurnMotion, TurnMotionPlanner, quaternion_yaw
from turn_signal_controller.signal_association import (
    Selection, project_signal, associate, SignalConfirmation,
)


def overlay(nominal, decision):
    output = copy.deepcopy(nominal)
    output.longlCmdType = 1
    output.velocity = output.acceleration = 0.0
    output.accel = min(clamp(float(nominal.accel), 0.0, 1.0), decision.accel_limit)
    output.brake = max(clamp(float(nominal.brake), 0.0, 1.0), decision.brake)
    if output.brake > 0:
        output.accel = 0.0
    return output


class ManeuverFusionNode:
    def __init__(self):
        rospy.init_node("maneuver_fusion", anonymous=False)
        self.lock = threading.RLock()
        self.points = clean_consecutive_duplicates(load_path_file(rospy.get_param("~path_file")))
        self.s_values = cumulative_arc_lengths(self.points)
        self.timeout = float(rospy.get_param("~input_timeout_sec", 0.5))
        self.signal_timeout = float(rospy.get_param("~signal_timeout_sec", 0.8))
        self.preview_m = float(rospy.get_param("~route_preview_m", 30.0))
        self.turn_threshold = float(rospy.get_param("~turn_threshold_deg", 25.0))
        self.max_route_error = float(rospy.get_param("~max_route_error_m", 3.0))
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.timeout, self.signal_timeout, self.preview_m, self.turn_threshold, self.max_route_error)):
            raise ValueError("Fusion distances, thresholds and timeouts must be positive and finite")
        self.require_quality = bool(rospy.get_param("~require_sensor_quality", True))
        self.require_safety = bool(rospy.get_param("~require_fresh_safety", True))
        self.require_camera_stream = bool(rospy.get_param("~require_fresh_camera_stream", True))
        self.right_on_green = bool(rospy.get_param("~right_on_green", True))
        # Strict mode never uses screen-centre/route-curvature as a signal ID.
        self.require_context = bool(rospy.get_param("~require_route_signal_context", True))
        self.require_reference_path = bool(rospy.get_param("~require_reference_path_match", self.require_context))
        self.reference_path_match = False
        self.reference_path_reason = "reference_path_not_received"
        self.allow_blackout_lane = bool(rospy.get_param("~allow_blackout_lane_corridor", False))
        self.blackout_max_duration = float(rospy.get_param("~blackout_max_duration_sec", 15.0))
        self.blackout_max_distance = float(rospy.get_param("~blackout_max_distance_m", 30.0))
        if not all(finite(v) and v > 0 for v in (self.blackout_max_duration, self.blackout_max_distance)):
            raise ValueError("Blackout corridor budgets must be positive and finite")
        self.corridor_anchor = None
        self.corridor_distance = 0.0
        self.corridor_last = None
        self.corridor_fault = False
        self.corridor_reason = "disabled"
        self.camera = rospy.get_param("~signal_camera", {})
        self.contexts, self.map_heads = [], []
        self.context_load_error = ""
        mgeo_dir = rospy.get_param("~signal_mgeo_path", "")
        if self.require_context and mgeo_dir:
            try:
                self.contexts = load_route_contexts(mgeo_dir, self.points, self.s_values)
                with (Path(mgeo_dir) / "traffic_light_set.json").open(encoding="utf-8-sig") as stream:
                    self.map_heads = [dict(id=str(h["idx"]), x=h["point"][0], y=h["point"][1], z=h["point"][2])
                                      for h in json.load(stream) if h.get("type") == "car"]
            except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
                self.contexts, self.map_heads = [], []
                self.context_load_error = str(exc)
                rospy.logerr("Signal map unavailable; fusion will brake: %s", exc)
        self.pose_history = deque(maxlen=100)
        self.confirmation = SignalConfirmation()
        self.selection = Selection("UNKNOWN", 0.0, False, None, "route_context_unavailable")
        self.selection_stamp = 0.0
        self.selection_context = None
        # Keep callback order until the control tick can associate every image.
        # Latest-only caching can erase a RED between two permissive images.
        self.signal_observations = deque()
        self.signal_queue_limit = 32
        self.signal_queue_fault = None
        self.signal_queue_overflows = 0
        self.unmapped_signal_seen = False
        self.next_guard_id = None
        self.next_guard_core = None
        self.lamp_enabled = bool(rospy.get_param("~lamp_output_enabled", False))
        self.remote = (rospy.get_param("~remote_ip", "192.168.0.151"),
                       int(rospy.get_param("~remote_port", 9097)))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if self.lamp_enabled else None
        self.lead = IndicatorLead(float(rospy.get_param("~lead_time_sec", 5.0)))
        self.core_params = dict(
            front_reference_offset_m=float(rospy.get_param("~front_reference_offset_m", 3.845)),
            hold_distance_m=float(rospy.get_param("~hold_distance_m", 0.5)),
            max_decel_mps2=float(rospy.get_param("~max_decel_mps2", 1.5)),
            planning_decel_mps2=float(rospy.get_param("~planning_decel_mps2", 1.0)),
            stop_tolerance_m=float(rospy.get_param("~stop_tolerance_m", 0.03)),
            signal_timeout_sec=self.signal_timeout,
        )
        self.core = StopLineControllerCore(**self.core_params)
        turn_defaults = dict(left_speed_kph=15., right_speed_kph=10., lateral_accel_mps2=1.2,
                             max_heading_error_deg=60., max_lateral_error_m=1.5,
                             exit_heading_error_deg=15., exit_lateral_error_m=.75,
                             exit_overrun_m=3.)
        self.turn_motion = TurnMotionPlanner(self.points, self.s_values,
            **{key: float(rospy.get_param("~turn_" + key, value)) for key, value in turn_defaults.items()},
            planning_decel_mps2=self.core.planning_decel_mps2,
            max_decel_mps2=self.core.max_decel_mps2,
            reaction_time_sec=self.core.reaction_time_sec)
        # Stopping clearance uses the bumper; signal entry uses the front axle.
        # Both offsets are measured from the same rear-axle base_link origin.
        self.front_axle_offset_m = float(rospy.get_param("~front_axle_offset_m", 3.0))
        if (not finite(self.front_axle_offset_m)
                or not 0 <= self.front_axle_offset_m <= self.core.front_reference_offset_m):
            raise ValueError("front_axle_offset_m must be finite and between zero and the bumper offset")
        self.accel_limiter = AccelRiseLimiter(
            float(rospy.get_param("~accel_rise_rate_per_sec", 0.5)), self.core.max_update_gap_sec)
        self.manual = parse_maneuvers(rospy.get_param("~maneuvers", []))
        if any(m.end_s_m is None or m.end_s_m <= m.start_s_m for m in self.manual):
            raise ValueError("Fusion route events require end_s_m > start_s_m")
        if any(m.kind not in ("turn", "lane_change") or m.end_s_m > self.s_values[-1] for m in self.manual):
            raise ValueError("Maneuver kind must be turn/lane_change and endpoints must be on the route")
        if len({m.identifier for m in self.manual}) != len(self.manual):
            raise ValueError("Duplicate maneuver id")
        if any(a.end_s_m > b.start_s_m for a, b in zip(self.manual, self.manual[1:])):
            raise ValueError("Overlapping route maneuvers")
        if self.require_context:
            self.manual = route_checked_maneuvers(self.manual, self.contexts)
        self.samples, self.last_stamps = {}, {}
        self.nominal = None
        self.nominal_at = None
        self.progress = self.segment = self.previous_ros = None
        self.last_projection = None
        self.event = None
        self.completed = set()
        self.ignore_line_until_s = -1.0
        self.last_observed_line_stamp = 0.0
        self.enter_permission = False
        self.entry_ticket = None
        self.merge_gap = None
        self.merge_gap_at = None
        self.state_pub = rospy.Publisher("/control/maneuver_status", String, queue_size=1)
        self.lamp_pub = rospy.Publisher("/control/turn_signal_state", String, queue_size=1)
        self.output_pub = rospy.Publisher(rospy.get_param("~output_command_topic", "/control/maneuver_cmd"), CtrlCmd, queue_size=1)
        for key, topic, message_type in (
            ("odom", rospy.get_param("~odometry_topic", "/localization/odometry"), Odometry),
            ("signal", rospy.get_param("~signal_topic", "/perception/traffic_light/directional_state"), TrafficLight),
            ("line", rospy.get_param("~stopline_topic", "/perception/camera/stopline"), StopLineDetection),
            ("safety", rospy.get_param("~safety_topic", "/detection/fused_safety_stop"), SafetyStop),
            ("quality", "/localization/sensor_quality", SensorQuality),
            ("objects", rospy.get_param("~signal_objects_topic", "/detection/traffic_light"), ObjectInfoArray),
        ):
            rospy.Subscriber(topic, message_type, self.observe, callback_args=key,
                             queue_size=self.signal_queue_limit if key in ("signal", "objects") else 1)
        rospy.Subscriber(rospy.get_param("~nominal_command_topic", "/control/camera_fallback_cmd"),
                         CtrlCmd, self.command_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param("~reference_path_topic", "/experimental/curvature_reference_path"),
                         RosPath, self.reference_path_callback, queue_size=1)
        rospy.Subscriber("/morai/lidar/merge_gap/results", String, self.merge_gap_callback, queue_size=1)
        if self.allow_blackout_lane:
            rospy.Subscriber("/stability/camera_fallback_status", String, self.fallback_status_callback, queue_size=1)
        rospy.on_shutdown(self.shutdown)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)

    def observe(self, message, key):
        stamp, now, ros_now = message.header.stamp.to_sec(), time.monotonic(), rospy.get_time()
        sample = Sample(stamp, now, message)
        with self.lock:
            timeout = self.signal_timeout if key in ("signal", "line", "objects") else self.timeout
            ordered_key = "objects" if self.require_context else "signal"
            previous = self.samples.get(key)
            if (key == ordered_key and previous is not None and stamp == previous.stamp
                    and sample.fresh(ros_now, now, timeout)
                    and self.signal_evidence(message, key) != self.signal_evidence(previous.value, key)):
                # Same source frame cannot simultaneously prove RED and GREEN.
                self.signal_observations.clear()
                self.signal_queue_fault = "signal_duplicate_conflict"
                self.revoke_entry_permission()
                return
            if sample.fresh(ros_now, now, timeout) and stamp > self.last_stamps.get(key, 0.0):
                self.samples[key] = sample
                self.last_stamps[key] = stamp
                if key == "odom":
                    self.pose_history.append(sample)
                if key == ordered_key:
                    if len(self.signal_observations) >= self.signal_queue_limit:
                        self.signal_observations.clear()
                        self.signal_queue_fault = "signal_queue_overflow"
                        self.signal_queue_overflows += 1
                        self.revoke_entry_permission()
                    self.signal_observations.append(sample)

    @staticmethod
    def signal_evidence(message, key):
        """Compare detection evidence, not Python identity or a transport seq."""
        if key == "signal":
            return message.state, message.confidence, message.valid
        fields = ("class_name", "conf", "x_center", "y_center", "width", "height")
        return (message.header.frame_id,
                tuple(tuple(getattr(obj, field, None) for field in fields) for obj in message.objects))

    def reference_path_callback(self, message):
        with self.lock:
            self.reference_path_reason = reference_path_reason(self.points, message)
            self.reference_path_match = self.reference_path_reason == "matched"
            if self.require_reference_path and not self.reference_path_match:
                # Revoke in the callback, even if a matching path returns before
                # the next tick. Old signal evidence must not revive permission.
                self.enter_permission = False
                self.entry_ticket = None
                self.core.revoke_permission()
                self.confirmation.reset()
                self.selection = Selection(reason=self.reference_path_reason)
                self.selection_stamp = max(self.selection_stamp, rospy.get_time())
                self.signal_observations.clear()
                self.lead.reset()
                self.corridor_anchor = self.corridor_last = None

    def command_callback(self, message):
        with self.lock:
            self.nominal, self.nominal_at = copy.deepcopy(message), time.monotonic()

    def merge_gap_callback(self, message):
        with self.lock:
            try:
                payload = json.loads(message.data)
                if not isinstance(payload, dict):
                    raise ValueError("gap result must be an object")
                self.merge_gap = payload
                self.merge_gap_at = time.monotonic()
            except (ValueError, TypeError):
                self.merge_gap = self.merge_gap_at = None

    def fallback_status_callback(self, message):
        with self.lock:
            now, ros_now = time.monotonic(), rospy.get_time()
            try:
                data = json.loads(message.data)
                stamp = float(data["stamp"])
                if not isinstance(data, dict) or not isinstance(data.get("mode"), str):
                    raise ValueError("invalid fallback status")
                sample = Sample(stamp, now, data)
                if sample.fresh(ros_now, now, self.timeout) and stamp > self.last_stamps.get("fallback", 0.):
                    self.samples["fallback"] = sample
                    self.last_stamps["fallback"] = stamp
            except (ValueError, KeyError, TypeError):
                self.samples.pop("fallback", None)

    def blackout_corridor(self, quality, fallback, speed, route_valid, line, objects, now, ros_now):
        """Bounded lane keeping only outside any route decision/visible signal.

        This is not permission to cross a junction with bad GPS. It only keeps
        a recently verified, map-aligned approach corridor alive at low speed.
        """
        self.corridor_reason = "disabled"
        if not self.allow_blackout_lane:
            return False
        if self.require_reference_path and not self.reference_path_match:
            self.corridor_reason = self.reference_path_reason
            return False
        if route_valid:
            self.corridor_anchor = (now, ros_now, self.progress)
            self.corridor_last = (now, ros_now)
            self.corridor_distance = 0.
            self.corridor_fault = False
            self.corridor_reason = "normal_route"
            return False
        self.corridor_reason = "no_verified_normal_anchor"
        if self.corridor_anchor is None or quality is None or fallback is None or speed is None:
            return False
        q, f = quality.value, fallback.value
        if self.corridor_last is not None:
            dt, wall_dt = ros_now - self.corridor_last[1], now - self.corridor_last[0]
            self.corridor_fault |= not (0 <= dt <= self.timeout and 0 <= wall_dt <= self.timeout)
            self.corridor_distance += speed * max(0., dt)
        self.corridor_last = (now, ros_now)
        elapsed = max(now - self.corridor_anchor[0], ros_now - self.corridor_anchor[1])
        self.corridor_fault |= (elapsed > self.blackout_max_duration
                                or self.corridor_distance > self.blackout_max_distance
                                or abs(self.progress - self.corridor_anchor[2]) > self.blackout_max_distance)
        self.corridor_reason = "blackout_corridor_budget_or_clock"
        if self.corridor_fault:
            return False
        self.corridor_reason = "fallback_not_verified"
        modes = ("gps_blackout_camera_fallback", "lane_loss_braking", "recovery_camera", "recovery_blend")
        if (q.state not in ("GPS_BLACKOUT", "NORMAL") or getattr(q, "imu_stale", True)
                or (q.state == "GPS_BLACKOUT" and not getattr(q, "gps_blackout", False))
                or (q.state == "NORMAL" and (not getattr(q, "gps_valid", False)
                    or getattr(q, "gps_blackout", True) or getattr(q, "gps_recovering", True)))
                or f.get("mode") not in modes or f.get("nominal_fresh") is not True
                or f.get("blackout_budget_exhausted") is not False
                or not all(finite(f.get(k)) and 0 <= f[k] <= 1. for k in ("accel", "brake"))):
            return False
        if f["mode"] != "recovery_blend" and f.get("camera_used") is not True:
            return False
        if f["mode"] in ("gps_blackout_camera_fallback", "recovery_camera") and f.get("lane_primary_usable") is not True:
            return False
        if f["mode"] == "recovery_blend" and q.state != "NORMAL":
            return False
        if f["mode"] == "lane_loss_braking" and not (f.get("accel") == 0 and finite(f.get("brake"))
                                                      and 0 < f["brake"] <= 1.):
            return False
        self.corridor_reason = "junction_or_observation_guard"
        if (not self.require_context or not self.contexts or self.event is not None
                or line is None or objects is None or line.value.valid or objects.value.objects
                or self.unmapped_signal_seen):
            return False
        horizon = max(40., speed * 6. + speed**2 / (2. * self.core.planning_decel_mps2))
        guards = list(self.contexts) + [dict(id=m.identifier, start=m.start_s_m, end=m.end_s_m) for m in self.manual]
        if any(c["id"] not in self.completed and c["end"] > self.progress
               and c["start"] - self.progress <= horizon for c in guards):
            return False
        self.corridor_reason = "bounded_lane_corridor"
        return True

    def lane_change_clear(self, direction, now):
        # Missing right-side results cannot be interpreted as a free right lane.
        return bool(self.merge_gap is not None and self.merge_gap_at is not None
                    and 0 <= now - self.merge_gap_at <= self.timeout
                    and self.merge_gap.get("valid") is True
                    and isinstance(self.merge_gap.get(direction.lower()), dict)
                    and self.merge_gap[direction.lower()].get("confirmed_available") is True)

    def fresh(self, key, now, ros_now):
        sample = self.samples.get(key)
        timeout = self.signal_timeout if key in ("signal", "line", "objects") else self.timeout
        return sample if sample is not None and sample.fresh(ros_now, now, timeout) else None

    def revoke_entry_permission(self):
        """A later green must not revive an approach-side entry ticket."""
        if self.event is not None and not self.event["committed"]:
            self.enter_permission = False
            self.entry_ticket = None
            self.core.revoke_permission()

    def consume_legacy_signals(self, direction, ready, route_valid, now, ros_now):
        """Legacy mode also consumes every accepted callback, once and in order."""
        if self.event is not None and self.event["committed"]:
            self.signal_observations.clear()
            self.signal_queue_fault = None
            return  # Existing committed-turn and live-stream guards own this phase.
        if self.signal_queue_fault:
            self.signal_queue_fault = None
            self.signal_observations.clear()
            self.core.revoke_permission()
            self.revoke_entry_permission()
            return
        while self.signal_observations:
            sample = self.signal_observations.popleft()
            if not sample.fresh(ros_now, now, self.signal_timeout):
                self.core.revoke_permission()
                self.revoke_entry_permission()
                continue
            message = sample.value
            valid = (message.valid and finite(message.confidence)
                     and .5 <= message.confidence <= 1.)
            allowed = (valid and ready and route_valid
                       and signal_permits(message.state, direction, self.right_on_green))
            if not allowed:
                self.revoke_entry_permission()
            self.core.observe_signal("GREEN" if allowed else "RED", message.confidence,
                                     valid, sample.stamp, sample.received, ros_now)

    def synchronize_route_intent(self, event):
        """Reset signal evidence and indicator lead before evaluating new intent."""
        context_id = route_event_key(event)
        if context_id != self.selection_context:
            if self.selection_context is not None:
                self.revoke_entry_permission()
                self.lead.reset()
                self.signal_observations.clear()
                self.selection_stamp = max(self.selection_stamp, rospy.get_time())
            self.confirmation.reset()
            self.selection = Selection(reason="route_intent_changed")
            self.selection_context = context_id

    def selected_signal(self, objects, event, route_valid):
        """Consume ordered images at image-time poses; defer, never skip, a hole."""
        self.synchronize_route_intent(event)
        reason = None
        if not route_valid or not event or not event.get("signal_points"):
            reason = "route_context_unavailable"
        elif not isinstance(self.camera, dict) or self.camera.get("calibrated") is not True:
            reason = "signal_camera_uncalibrated"
        elif objects is None:
            reason = "signal_observation_stale"
        if reason:
            self.confirmation.reset()
            self.signal_observations.clear()
            self.revoke_entry_permission()
            self.selection = Selection("UNKNOWN", 0.0, False, None, reason)
            return self.selection
        if self.signal_queue_fault:
            fault, self.signal_queue_fault = self.signal_queue_fault, None
            self.signal_observations.clear()
            self.confirmation.reset()
            self.selection_stamp = objects.stamp
            self.revoke_entry_permission()
            self.selection = Selection(reason=fault)
            return self.selection
        # Reconsider a still-fresh cached frame after context acquisition. The
        # finalized watermark prevents counting an image twice on timer ticks.
        if not self.signal_observations and objects.stamp > self.selection_stamp:
            self.signal_observations.append(objects)
        now, ros_now = time.monotonic(), rospy.get_time()
        while self.signal_observations:
            observation = self.signal_observations[0]
            if observation.stamp <= self.selection_stamp:
                self.signal_observations.popleft()
                continue
            if not observation.fresh(ros_now, now, self.signal_timeout):
                self.signal_observations.popleft()
                self.selection_stamp = observation.stamp
                self.confirmation.reset()
                self.revoke_entry_permission()
                self.selection = Selection(reason="signal_observation_stale")
                continue
            selection = self.associate_signal_frame(observation, event)
            if selection is None:
                # Preserve the original stamp and receipt time. Later odometry
                # may complete this pair, but cannot refresh the image's age.
                self.confirmation.reset()
                self.revoke_entry_permission()
                self.selection = Selection(reason="signal_pose_unsynchronized")
                return self.selection
            self.signal_observations.popleft()
            self.selection_stamp = observation.stamp
            self.selection = self.confirmation.update(selection, observation.stamp)
            if not (self.selection.valid and signal_permits(
                    self.selection.state, event["direction"], self.right_on_green)):
                self.revoke_entry_permission()
        return self.selection

    def associate_signal_frame(self, objects, event):
        """None means await matching pose, not consume an UNKNOWN image."""
        # Closest timestamp must be within 50ms; no extrapolation across turns.
        pose_sample = min(self.pose_history, key=lambda p: abs(p.stamp - objects.stamp), default=None)
        if (pose_sample is None or pose_sample.value.header.frame_id != "map"
                or abs(pose_sample.stamp - objects.stamp) > 0.05):
            return None
        pose = pose_sample.value.pose.pose
        try:
            q = pose.orientation
            norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
            if not math.isfinite(norm) or abs(norm - 1.0) > 0.01:
                raise ValueError("invalid pose quaternion")
            roll = math.atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x*q.x + q.y*q.y))
            pitch = math.asin(clamp(2*(q.w*q.y - q.z*q.x), -1.0, 1.0))
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            ego = dict(x=pose.position.x, y=pose.position.y, z=pose.position.z,
                       yaw=yaw, roll=roll, pitch=pitch)
            targets = {h["id"]: project_signal(h, ego, self.camera) for h in event["signal_points"]}
            others = {h["id"]: project_signal(h, ego, self.camera) for h in self.map_heads
                      if h["id"] not in targets}
            selection = associate(objects.value.objects, targets, others)
        except (ValueError, TypeError, AttributeError, KeyError):
            selection = Selection("UNKNOWN", 0.0, False, None, "signal_projection_invalid")
        return selection

    def send_lamp(self, direction, now, ros_now):
        success = False
        if self.socket is not None:
            try:
                packet = build_lamp_packet(direction)
                success = self.socket.sendto(packet, self.remote) == len(packet)
            except OSError as exc:
                rospy.logerr_throttle(2.0, "LampControl send failed: %s", exc)
        self.lamp_requested, self.lamp_transmit_ok = direction, success
        self.lead.sent(direction, now, ros_now, success)
        # This is transmitted state, never a claim that MORAI acknowledged it.
        self.lamp_pub.publish(String(data=direction if success else "OFF"))

    def update_route(self, odom, now, ros_now):
        if odom is None or odom.value.header.frame_id != "map":
            return None
        pose, velocity = odom.value.pose.pose, odom.value.twist.twist.linear
        x, y = float(pose.position.x), float(pose.position.y)
        speed = math.hypot(velocity.x, velocity.y)
        if not all(finite(v) for v in (x, y, speed)):
            return None
        projection = nearest_projection(self.points, self.s_values, x, y,
                                        start_segment=max(0, (self.segment or 0) - 250),
                                        end_segment=None if self.segment is None else self.segment + 250)
        if projection.distance_m > self.max_route_error:
            return None
        if self.last_projection is not None:
            previous_s, previous_time, previous_speed = self.last_projection
            dt = ros_now - previous_time
            if dt < 0 or abs(projection.progress_s - previous_s) > max(speed, previous_speed) * dt + 2.0:
                return None
        if self.progress is not None and projection.progress_s < self.progress - 2.0:
            return None  # Require restart after a teleport/backtrack, not replay old events.
        self.progress = projection.progress_s
        self.segment = projection.segment_index
        self.last_projection = (self.progress, ros_now, speed)
        return speed

    def discover_event(self, line_sample, speed, now, ros_now):
        if self.event is not None or self.progress is None:
            return
        if self.require_context:
            horizon = max(40.0, speed * (self.lead.lead_sec + 1.0) + speed**2 / 2.0)
            candidates = [dict(c, kind="turn", committed=False) for c in self.contexts
                          if c["id"] not in self.completed and c["end"] > self.progress]
            candidates.extend(dict(id=m.identifier, start=m.start_s_m, end=m.end_s_m,
                                   kind=m.kind, direction=m.direction, committed=False)
                              for m in self.manual
                              if m.identifier not in self.completed and m.end_s_m > self.progress)
            if candidates:
                candidate = min(candidates, key=lambda c: (c["start"], c.get("source") != "mgeo"))
                objects = self.fresh("objects", now, ros_now)
                visible = objects is not None and bool(objects.value.objects)
                if (candidate["start"] - self.progress <= horizon
                        or (visible and candidate.get("source") == "mgeo")):
                    # A known head may be visible before the braking horizon.
                    # Pre-arm its map context; do not latch a false "unmapped"
                    # stop merely because the line is still >40m away.
                    self.event = candidate
                    if candidate.get("source") == "mgeo":
                        self.unmapped_signal_seen = False
            return
        for maneuver in self.manual:
            if maneuver.identifier in self.completed:
                continue
            if self.progress >= maneuver.end_s_m:
                self.completed.add(maneuver.identifier)
                continue
            # Enough horizon to signal and brake BEFORE the PP lookahead reaches
            # a lane-change start. Events describe the existing route only.
            horizon = max(25.0, speed * (self.lead.lead_sec + 1.0) + speed**2 / 2.0)
            if maneuver.start_s_m - self.progress <= horizon:
                self.event = dict(id=maneuver.identifier, direction=maneuver.direction,
                                  start=maneuver.start_s_m, end=maneuver.end_s_m,
                                  kind=maneuver.kind, committed=False)
            break
        if self.require_context:
            # No semantic map match: do not turn a missing line into STRAIGHT.
            return
        if self.event is not None or line_sample is None or self.progress < self.ignore_line_until_s:
            return
        line = line_sample.value
        if (not line.valid or line.header.frame_id != "base_link"
                or not finite(line.distance_m) or not 0 <= line.distance_m <= 50.0
                or not finite(line.confidence) or line.confidence < 0.5):
            return
        line_s = self.line_route_s(line_sample)
        if line_s is None:
            return
        intent = route_intent(self.points, self.s_values, line_s, self.preview_m, self.turn_threshold)
        self.event = dict(id="junction_%.1f" % line_s, start=line_s, end=intent.end_s_m,
                          direction=intent.direction, kind="turn", committed=False)

    def line_route_s(self, line_sample):
        """Match a camera distance to the route pose at its source timestamp.

        A current pose minus latest_speed*delay is wrong while braking. Use
        the same bounded 50ms pose matching as signal-head association.
        """
        pose_sample = min(self.pose_history, key=lambda p: abs(p.stamp - line_sample.stamp), default=None)
        if (pose_sample is None or abs(pose_sample.stamp - line_sample.stamp) > 0.05
                or pose_sample.value.header.frame_id != "map"):
            return None
        point = pose_sample.value.pose.pose.position
        if not all(finite(v) for v in (point.x, point.y, line_sample.value.distance_m)):
            return None
        projection = nearest_projection(self.points, self.s_values, point.x, point.y,
                                        start_segment=max(0, (self.segment or 0) - 250),
                                        end_segment=None if self.segment is None else self.segment + 250)
        if projection.distance_m > self.max_route_error:
            return None
        return projection.progress_s + line_sample.value.distance_m

    def crossing_s(self, event):
        if event["kind"] == "lane_change":
            return event["start"]
        # Camera noise may bring the STOP guard upstream, but cannot move a
        # verified map crossing upstream and bypass a still-relevant signal.
        line_s = event.get("stop_s")
        return (event["start"] if line_s is None else line_s) - self.front_axle_offset_m

    def turn_assessment(self, odom, speed):
        if not self.event or self.event["kind"] != "turn" or self.event["direction"] not in ("LEFT", "RIGHT"):
            return TurnMotion()
        pose = odom.value.pose.pose if odom is not None else None
        position = getattr(pose, "position", None)
        return self.turn_motion.evaluate(self.event, self.progress, speed,
            getattr(position, "x", None), getattr(position, "y", None),
            quaternion_yaw(getattr(pose, "orientation", None)), self.crossing_s(self.event))

    def entry_candidate(self, event, permitted, ready, route_valid, now, ros_now):
        """Provisional entry only; the final safety checks must also approve.

        A short-lived ticket proves permission on the approach side. A current
        red/unknown light, lamp failure, time gap or mid-junction restart cannot
        retroactively authorize a crossing. Faults persist for this event.
        """
        if event["committed"]:
            return True
        if not route_valid or self.progress <= self.crossing_s(event):
            return False
        ticket = self.entry_ticket
        valid = bool(not event.get("entry_fault") and permitted and ready
                     and self.enter_permission and ticket is not None
                     and ticket[0] == route_event_key(event) and ticket[1] <= self.crossing_s(event)
                     and 0 <= now - ticket[2] <= self.timeout
                     and 0 < ros_now - ticket[3] <= self.timeout)
        if not valid:
            event["entry_fault"] = True
        return valid

    def tick(self, _event):
        with self.lock:
            now, ros_now = time.monotonic(), rospy.get_time()
            reset = self.previous_ros is not None and ros_now < self.previous_ros
            self.previous_ros = ros_now
            if reset:
                self.samples.clear()
                self.last_stamps.clear()
                self.progress = self.segment = self.event = None
                self.last_projection = None
                self.merge_gap = self.merge_gap_at = None
                self.completed.clear()
                self.ignore_line_until_s = -1.0
                self.enter_permission = False
                self.entry_ticket = None
                self.lead.reset()
                self.core = StopLineControllerCore(**self.core_params)
                self.last_observed_line_stamp = 0.0
                self.pose_history.clear()
                self.confirmation.reset()
                self.selection_context, self.selection_stamp = None, 0.0
                self.signal_observations.clear()
                self.signal_queue_fault = None
                self.unmapped_signal_seen = False
                self.next_guard_id = self.next_guard_core = None
                self.corridor_anchor = self.corridor_last = None
                self.corridor_distance = 0.
                self.corridor_fault = False

            odom = self.fresh("odom", now, ros_now)
            speed = self.update_route(odom, now, ros_now)
            quality = self.fresh("quality", now, ros_now)
            route_valid = (speed is not None
                           and (not self.require_reference_path or self.reference_path_match))
            if self.require_quality:
                route_valid = route_valid and quality is not None and quality.value.state == "NORMAL"
            fallback = self.fresh("fallback", now, ros_now) if self.allow_blackout_lane else None
            if self.allow_blackout_lane:
                route_valid = bool(route_valid and quality is not None and quality.value.state == "NORMAL"
                                   and not quality.value.imu_stale
                                   and quality.value.gps_valid and not quality.value.gps_blackout
                                   and not quality.value.gps_recovering and fallback is not None
                                   and fallback.value.get("mode") == "normal_nominal"
                                   and fallback.value.get("nominal_fresh") is True)
            line = self.fresh("line", now, ros_now)
            signal = self.fresh("signal", now, ros_now)
            objects = self.fresh("objects", now, ros_now)
            corridor_ok = self.blackout_corridor(quality, fallback, speed, route_valid, line, objects, now, ros_now)
            if route_valid:
                self.discover_event(line, speed, now, ros_now)

            motion = self.turn_assessment(odom, speed)
            completed_this_tick = None
            if (self.event and self.event["committed"] and route_valid
                    and self.progress >= self.event["end"]
                    and (motion.phase == "NONE" or (motion.exit_ready and not motion.fault))):
                completed_this_tick = self.event["id"]
                self.completed.add(self.event["id"])
                self.ignore_line_until_s = self.progress + 3.0
                self.event = None
                self.enter_permission = False
                self.entry_ticket = None
                self.lead.reset()
                self.core = StopLineControllerCore(**self.core_params)
                # Only a genuinely newer line may arm the next intersection.
                self.last_observed_line_stamp = line.stamp if line else self.last_observed_line_stamp
                self.confirmation.reset()
                self.selection_context = None
                # Discover a close following junction in this SAME tick.
                if self.require_context:
                    self.discover_event(line, speed, now, ros_now)
                motion = self.turn_assessment(odom, speed)

            # A pose fault before entry also cancels previously accumulated
            # signal permission. Recovery must confirm new observations.
            if motion.fault and self.event and not self.event["committed"]:
                self.revoke_entry_permission()
                self.confirmation.reset()

            if self.require_context:
                self.synchronize_route_intent(self.event)
            direction = self.event["direction"] if self.event else ("UNKNOWN" if self.require_context else "STRAIGHT")
            lamp = direction if direction in ("LEFT", "RIGHT") else "OFF"
            if (self.require_context and self.event and self.progress is not None
                    and self.event["start"] - self.progress > max(40., (speed or 0.) * 6. + (speed or 0.)**2 / 2.)):
                lamp = "OFF"  # Early map association is not early lamp activation.
            # A localization fault brakes the vehicle, but does not cancel an
            # already entered turn's indication before its exit is verified.
            keep_lamp = bool(self.event and self.event["committed"])
            self.send_lamp(lamp if route_valid or keep_lamp else "OFF", now, ros_now)
            ready = direction == "STRAIGHT" or self.lead.ready(lamp, now, ros_now)
            signal_ok = (signal is not None and signal.value.valid
                         and finite(signal.value.confidence) and 0.5 <= signal.value.confidence <= 1.0)
            permitted = bool(signal_ok and signal_permits(signal.value.state, direction, self.right_on_green))
            event = self.event
            if not self.require_context:
                self.consume_legacy_signals(direction, ready, route_valid and not motion.fault, now, ros_now)
            if self.require_context:
                chosen = self.selected_signal(objects, event, route_valid and not motion.fault)
                signal_ok = chosen.valid
                permitted = chosen.valid and signal_permits(chosen.state, direction, self.right_on_green)
                if event is None and ((objects is not None and objects.value.objects)
                                      or (line is not None and line.value.valid)):
                    # Latch until a mapped context is acquired/restart: one missed
                    # subsequent image must not release an unresolved junction.
                    self.unmapped_signal_seen = True
            committed = bool(event and event["committed"])

            if event and event["kind"] == "lane_change":
                # The PP lookahead is 4 + 0.35*v. Stop the rear axle before it
                # reaches the start, until five continuous seconds have elapsed.
                guard = max(5.0, 4.0 + 0.35 * (speed or 0.0))
                virtual_distance = (event["start"] - guard - self.progress
                                    + self.core.front_reference_offset_m + self.core.hold_distance_m)
                if not committed:
                    self.core.observe_line(max(0.0, virtual_distance), 1.0, True, ros_now, now, ros_now,
                                           target_id=event["id"])
                gap_clear = self.lane_change_clear(direction, now)
                # Do not use a lane-change event to bypass a visible red light.
                permitted = not signal_ok or signal_permits(signal.value.state, "STRAIGHT", self.right_on_green)
                if self.require_context:
                    # Manual lane changes have no assigned traffic-light head.
                    # UNKNOWN association must not mean a visible red is absent.
                    permitted = (objects is not None and not objects.value.objects
                                 and line is not None and not line.value.valid)
                committed = self.entry_candidate(event, permitted and gap_clear, ready, route_valid, now, ros_now)
                effective = "GREEN" if ready and route_valid and gap_clear and permitted else "RED"
                self.core.observe_signal(effective, 1.0, True, ros_now, now, ros_now)
            else:
                if self.require_context and event and route_valid and not committed:
                    # Map target survives camera loss. Unverified entry is only
                    # a conservative guard, NOT a measured stop-line location.
                    distance = event["start"] - self.progress
                    if line is not None and line.value.valid and line.value.header.frame_id == "base_link":
                        measured = line.value.distance_m
                        if (finite(measured) and 0 <= measured <= 50
                                and finite(line.value.confidence) and 0.5 <= line.value.confidence <= 1.0):
                            candidate = self.line_route_s(line)
                            if candidate is not None and abs(candidate-event["start"]) <= 2.0:
                                # A noisy observation cannot push the guard ahead.
                                event["start"] = min(event["start"], candidate)
                                event["camera_line_confirmed"] = True
                                distance = event["start"] - self.progress
                    self.core.observe_line(max(0.0, distance), 1.0, True, ros_now, now, ros_now,
                                           target_id=event["id"])
                    permitted = permitted and (event.get("stop_s") is not None
                                               or event.get("camera_line_confirmed", False))
                elif (line is not None and line.stamp > self.last_observed_line_stamp
                        and line.value.header.frame_id == "base_link" and not committed
                        and (self.progress is None or self.progress >= self.ignore_line_until_s)):
                    self.core.observe_line(line.value.distance_m, line.value.confidence, line.value.valid,
                                           line.stamp, line.received, ros_now)
                    self.last_observed_line_stamp = line.stamp
                if event:
                    committed = self.entry_candidate(event, permitted, ready, route_valid, now, ros_now)
                if committed and route_valid:
                    # Camera arrows may leave view DURING the turn. Retain the
                    # previously granted maneuver; obstacles still override below.
                    self.core.observe_signal("GREEN", 1.0, True, ros_now, now, ros_now)
                elif self.require_context and event:
                    effective = "GREEN" if permitted and ready and route_valid else "RED"
                    self.core.observe_signal(effective, 1.0, True, ros_now, now, ros_now)
                elif signal is not None:
                    effective = "GREEN" if permitted and ready and route_valid else "RED"
                    self.core.observe_signal(effective, signal.value.confidence, signal_ok,
                                             signal.stamp, signal.received, ros_now)
                if event and not ready:
                    self.core.stop_requested = True

            decision = self.core.update(now, ros_now, speed)
            # The front bumper can reach a following line before the rear axle
            # exits the active junction. Its old GREEN must not bypass that line.
            future = list(self.contexts) + [dict(id=m.identifier, start=m.start_s_m, end=m.end_s_m,
                                                kind=m.kind) for m in self.manual]
            future = sorted((c for c in future if event and c["id"] != event["id"]
                             and c["id"] not in self.completed and c["start"] > event["start"]
                             and c["end"] > (self.progress or 0)), key=lambda c: c["start"])
            upcoming = future[0] if future else None
            if self.require_context and event and route_valid and upcoming:
                if self.next_guard_id != upcoming["id"]:
                    self.next_guard_id = upcoming["id"]
                    self.next_guard_core = StopLineControllerCore(**self.core_params)
                guard_distance = upcoming["start"] - self.progress
                if upcoming.get("kind") == "lane_change":
                    guard_distance += (self.core.front_reference_offset_m + self.core.hold_distance_m
                                       - max(5., 4. + 0.35 * speed))
                self.next_guard_core.observe_line(max(0., guard_distance),
                                                  1., True, ros_now, now, ros_now,
                                                  target_id=upcoming["id"])
                self.next_guard_core.observe_signal("RED", 1., True, ros_now, now, ros_now)
                guard = self.next_guard_core.update(now, ros_now, speed)
                if guard.accel_limit < decision.accel_limit or guard.brake > decision.brake:
                    decision = Decision(guard.mode, "next_junction_guard",
                                        min(decision.accel_limit, guard.accel_limit),
                                        max(decision.brake, guard.brake), guard.target_speed_kph, guard.distance_m)
            else:
                self.next_guard_id = self.next_guard_core = None
            # An allowed arrow controls entry, not cornering speed. Retain the
            # original core mode so a speed limit cannot erase an entry ticket.
            decision = motion.constrain(decision)
            reasons = [motion.fault] if motion.fault else []
            # UNKNOWN/no line is a valid new image observation. No new stamped
            # observations is a dead camera/inference stream, including mid-turn.
            if self.require_camera_stream and (signal is None or line is None):
                reasons.append("camera_observation_stream_stale")
            if self.require_reference_path and not self.reference_path_match:
                reasons.append(self.reference_path_reason)
            if self.require_context:
                if objects is None:
                    reasons.append("signal_observation_stale")
                if not self.contexts:
                    reasons.append("route_context_unavailable")
                if self.unmapped_signal_seen:
                    reasons.append("unmapped_signal_or_stopline")
                if not route_valid and not corridor_ok:
                    reasons.append("signal_localization_unreliable")
                if (event and not committed and event["kind"] != "lane_change"
                        and objects is not None and objects.value.objects
                        and self.selection.selected_id is None):
                    reasons.append("unassociated_visible_signal")
                if (event and route_valid and line is not None and line.value.valid
                        and line.value.header.frame_id == "base_link"
                        and finite(line.value.distance_m) and 0 <= line.value.distance_m <= 50
                        and finite(line.value.confidence) and 0.5 <= line.value.confidence <= 1.):
                    observed_s = self.line_route_s(line)
                    if observed_s is None:
                        reasons.append("stopline_pose_unsynchronized")
                    elif not committed and observed_s < event["start"] - 2.:
                        reasons.append("unmapped_stopline_before_context")
                    elif (committed and line.value.distance_m > self.core.front_reference_offset_m
                          and not any(abs(observed_s-c["start"]) <= 2. for c in self.contexts
                                      if c["id"] not in self.completed)):
                        reasons.append("unmapped_stopline_during_maneuver")
            if speed is None:
                reasons.append("odometry_or_route_unavailable")
            if self.allow_blackout_lane and fallback is None:
                reasons.append("camera_fallback_status_stale")
            if self.allow_blackout_lane and not route_valid and not corridor_ok:
                reasons.append("blackout_corridor_unavailable")
            if event and not route_valid:
                reasons.append("maneuver_localization_unreliable")
            if event and direction == "UNKNOWN":
                reasons.append("route_direction_unknown")
            if event and event["kind"] == "lane_change" and not self.lane_change_clear(direction, now):
                reasons.append("adjacent_lane_gap_unavailable_or_stale")
            safety = self.fresh("safety", now, ros_now)
            if self.require_safety and safety is None:
                reasons.append("fused_safety_stale")
            if safety is not None and safety.value.stop_required:
                reasons.append(safety.value.reason or "fused_obstacle_stop")
            nominal = self.nominal
            valid_nominal = (nominal is not None and self.nominal_at is not None
                             and 0 <= now - self.nominal_at <= self.timeout
                             and nominal.longlCmdType == 1
                             and all(finite(float(getattr(nominal, k))) for k in ("accel", "brake", "steering")))
            if not valid_nominal:
                reasons.append("nominal_stale_or_not_type1")
                nominal = CtrlCmd()
            if event and committed and not event["committed"]:
                # The tentative crossing must not survive a current obstacle,
                # dead stream, invalid pose or invalid upstream command.
                if reasons:
                    event["entry_fault"] = True
                    committed = False
                else:
                    event["committed"] = True
            if event and event.get("entry_fault"):
                reasons.append("entry_without_current_permission")
            self.enter_permission = bool(event and route_valid and ready and (permitted or committed)
                                         and decision.mode == "NOMINAL" and not reasons)
            self.entry_ticket = ((route_event_key(event), self.progress, now, ros_now)
                                 if self.enter_permission and not committed
                                 and self.progress <= self.crossing_s(event) else None)
            if reasons:
                decision = Decision("SAFE_STOP", ",".join(reasons), 0.0, 1.0, 0.0, decision.distance_m)
            if corridor_ok:
                # CtrlCmd has no header: status and command can arrive one tick
                # apart. A fresh braking status must constrain an older pedal.
                decision = Decision(decision.mode, decision.reason,
                                    min(decision.accel_limit, fallback.value["accel"]),
                                    max(decision.brake, fallback.value["brake"]),
                                    decision.target_speed_kph, decision.distance_m)
            output = overlay(nominal, decision)
            requested_accel = output.accel
            output.accel = self.accel_limiter.limit(output.accel, output.brake, now, ros_now)
            self.output_pub.publish(output)
            permission_state = (self.selection.state if self.require_context
                                else signal.value.state if signal else "UNKNOWN")
            allowed_directions = sorted(signal_allowed_directions(permission_state, self.right_on_green)) if signal_ok else []
            self.state_pub.publish(String(data=json.dumps({
                "mode": decision.mode, "reason": decision.reason, "event": event,
                "direction": direction, "signal": signal.value.state if signal else "STALE",
                "route_direction": direction, "signal_allowed_directions": allowed_directions,
                "route_signal_compatible": direction in allowed_directions,
                "reference_path_required": self.require_reference_path,
                "reference_path_match": self.reference_path_match,
                "reference_path_reason": self.reference_path_reason,
                "indicator_ready": ready, "lamp_udp_enabled": self.lamp_enabled,
                "lamp_requested": self.lamp_requested, "lamp_transmit_ok": self.lamp_transmit_ok,
                "permission": self.enter_permission, "progress_s_m": self.progress,
                "entry_fault": bool(event and event.get("entry_fault")),
                "turn_phase": ("TURNING" if committed and motion.phase == "APPROACH" else motion.phase),
                "turn_speed_limit_kph": motion.speed_limit_kph,
                "turn_curve_speed_kph": motion.curve_speed_kph,
                "turn_heading_error_deg": motion.heading_error_deg,
                "turn_lateral_error_m": motion.lateral_error_m,
                "turn_exit_heading_error_deg": motion.exit_heading_error_deg,
                "completed_event_id": completed_this_tick,
                "blackout_lane_corridor": corridor_ok,
                "blackout_corridor_reason": self.corridor_reason,
                "blackout_corridor_distance_m": self.corridor_distance,
                "entry_reference_offset_m": self.front_axle_offset_m,
                "entry_crossing_s_m": self.crossing_s(event) if event else None,
                "front_bumper_distance_m": decision.distance_m,
                "target_clearance_m": self.core.hold_distance_m,
                "stopline_tracking_fault": self.core.tracking_fault,
                "accel_rise_limited": output.accel < requested_accel,
                "route_context_count": len(self.contexts),
                "route_context_error": self.context_load_error,
                "next_junction_guard_id": self.next_guard_id,
                "signal_selection_reason": self.selection.reason if self.require_context else "legacy_unassociated",
                "signal_pending_frames": len(self.signal_observations),
                "signal_queue_overflows": self.signal_queue_overflows,
                "signal_processed_stamp": self.selection_stamp if self.require_context else self.core.last_signal_stamp,
                "selected_signal_id": self.selection.selected_id if self.require_context else None,
                "selected_signal_state": self.selection.state if self.require_context else None,
                "accel": output.accel, "brake": output.brake,
            }, allow_nan=False)))

    def shutdown(self):
        with self.lock:
            if self.socket is not None:
                try:
                    self.socket.sendto(build_lamp_packet("OFF"), self.remote)
                except OSError:
                    pass
                self.socket.close()
                self.socket = None


if __name__ == "__main__":
    ManeuverFusionNode()
    rospy.spin()
