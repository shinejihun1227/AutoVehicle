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
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from common.msg import ObjectInfoArray

from curvature_speed_purepursuit.planner import (
    load_path_file, clean_consecutive_duplicates, cumulative_arc_lengths, nearest_projection,
)
from stopline_control.core import StopLineControllerCore, Sample, Decision, finite, clamp
from turn_signal_controller.fusion import IndicatorLead, build_lamp_packet, route_intent, signal_permits
from turn_signal_controller.scheduler import parse_maneuvers
from turn_signal_controller.route_context import load_route_contexts
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
            signal_timeout_sec=self.signal_timeout,
        )
        self.core = StopLineControllerCore(**self.core_params)
        self.manual = parse_maneuvers(rospy.get_param("~maneuvers", []))
        if any(m.end_s_m is None or m.end_s_m <= m.start_s_m for m in self.manual):
            raise ValueError("Fusion route events require end_s_m > start_s_m")
        if any(m.kind not in ("turn", "lane_change") or m.end_s_m > self.s_values[-1] for m in self.manual):
            raise ValueError("Maneuver kind must be turn/lane_change and endpoints must be on the route")
        if len({m.identifier for m in self.manual}) != len(self.manual):
            raise ValueError("Duplicate maneuver id")
        if any(a.end_s_m > b.start_s_m for a, b in zip(self.manual, self.manual[1:])):
            raise ValueError("Overlapping route maneuvers")
        if self.require_context and any(
                m.kind == "lane_change" and m.start_s_m < c["end"] and m.end_s_m > c["start"]
                for m in self.manual for c in self.contexts):
            raise ValueError("Lane-change plans must not overlap signal-controlled contexts")
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
            rospy.Subscriber(topic, message_type, self.observe, callback_args=key, queue_size=1)
        rospy.Subscriber(rospy.get_param("~nominal_command_topic", "/control/camera_fallback_cmd"),
                         CtrlCmd, self.command_callback, queue_size=1)
        rospy.Subscriber("/morai/lidar/merge_gap/results", String, self.merge_gap_callback, queue_size=1)
        rospy.on_shutdown(self.shutdown)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)

    def observe(self, message, key):
        stamp, now, ros_now = message.header.stamp.to_sec(), time.monotonic(), rospy.get_time()
        sample = Sample(stamp, now, message)
        with self.lock:
            timeout = self.signal_timeout if key in ("signal", "line", "objects") else self.timeout
            if sample.fresh(ros_now, now, timeout) and stamp > self.last_stamps.get(key, 0.0):
                self.samples[key] = sample
                self.last_stamps[key] = stamp
                if key == "odom":
                    self.pose_history.append(sample)

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

    def selected_signal(self, objects, event, route_valid):
        """Associate at the image timestamp, not with a newer turning pose."""
        context_id = event["id"] if event else None
        if context_id != self.selection_context:
            self.confirmation.reset()
            self.selection_context, self.selection_stamp = context_id, 0.0
        reason = None
        if not route_valid or not event or not event.get("signal_points"):
            reason = "route_context_unavailable"
        elif not isinstance(self.camera, dict) or self.camera.get("calibrated") is not True:
            reason = "signal_camera_uncalibrated"
        elif objects is None:
            reason = "signal_observation_stale"
        if reason:
            self.confirmation.reset()
            self.selection = Selection("UNKNOWN", 0.0, False, None, reason)
            return self.selection
        if objects.stamp == self.selection_stamp:
            return self.selection
        self.selection_stamp = objects.stamp
        # Closest timestamp must be within 50ms; no extrapolation across turns.
        pose_sample = min(self.pose_history, key=lambda p: abs(p.stamp - objects.stamp), default=None)
        if (pose_sample is None or pose_sample.value.header.frame_id != "map"
                or abs(pose_sample.stamp - objects.stamp) > 0.05):
            self.confirmation.reset()
            self.selection = Selection("UNKNOWN", 0.0, False, None, "signal_pose_unsynchronized")
            return self.selection
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
        self.selection = self.confirmation.update(selection, objects.stamp)
        return self.selection

    def send_lamp(self, direction, now, ros_now):
        success = False
        if self.socket is not None:
            try:
                self.socket.sendto(build_lamp_packet(direction), self.remote)
                success = True
            except OSError as exc:
                rospy.logerr_throttle(2.0, "LampControl send failed: %s", exc)
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
        line_s = self.progress + line.distance_m - speed * max(0.0, ros_now - line_sample.stamp)
        intent = route_intent(self.points, self.s_values, line_s, self.preview_m, self.turn_threshold)
        self.event = dict(id="junction_%.1f" % line_s, start=line_s, end=intent.end_s_m,
                          direction=intent.direction, kind="turn", committed=False)

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
                self.lead.reset()
                self.core = StopLineControllerCore(**self.core_params)
                self.last_observed_line_stamp = 0.0
                self.pose_history.clear()
                self.confirmation.reset()
                self.selection_context, self.selection_stamp = None, 0.0
                self.unmapped_signal_seen = False
                self.next_guard_id = self.next_guard_core = None

            speed = self.update_route(self.fresh("odom", now, ros_now), now, ros_now)
            quality = self.fresh("quality", now, ros_now)
            route_valid = speed is not None
            if self.require_quality:
                route_valid = route_valid and quality is not None and quality.value.state == "NORMAL"
            line = self.fresh("line", now, ros_now)
            signal = self.fresh("signal", now, ros_now)
            objects = self.fresh("objects", now, ros_now)
            if route_valid:
                self.discover_event(line, speed, now, ros_now)

            if self.event and route_valid and self.progress >= self.event["end"]:
                self.completed.add(self.event["id"])
                self.ignore_line_until_s = self.progress + 3.0
                self.event = None
                self.enter_permission = False
                self.lead.reset()
                self.core = StopLineControllerCore(**self.core_params)
                # Only a genuinely newer line may arm the next intersection.
                self.last_observed_line_stamp = line.stamp if line else self.last_observed_line_stamp
                self.confirmation.reset()
                self.selection_context = None
                # Discover a close following junction in this SAME tick.
                if self.require_context:
                    self.discover_event(line, speed, now, ros_now)

            direction = self.event["direction"] if self.event else ("UNKNOWN" if self.require_context else "STRAIGHT")
            lamp = direction if direction in ("LEFT", "RIGHT") else "OFF"
            if (self.require_context and self.event and self.progress is not None
                    and self.event["start"] - self.progress > max(40., (speed or 0.) * 6. + (speed or 0.)**2 / 2.)):
                lamp = "OFF"  # Early map association is not early lamp activation.
            self.send_lamp(lamp if route_valid else "OFF", now, ros_now)
            ready = direction == "STRAIGHT" or self.lead.ready(lamp, now, ros_now)
            signal_ok = (signal is not None and signal.value.valid
                         and finite(signal.value.confidence) and 0.5 <= signal.value.confidence <= 1.0)
            permitted = bool(signal_ok and signal_permits(signal.value.state, direction, self.right_on_green))
            event = self.event
            if self.require_context:
                chosen = self.selected_signal(objects, event, route_valid)
                signal_ok = chosen.valid
                permitted = chosen.valid and signal_permits(chosen.state, direction, self.right_on_green)
                if event is None and ((objects is not None and objects.value.objects)
                                      or (line is not None and line.value.valid)):
                    # Latch until a mapped context is acquired/restart: one missed
                    # subsequent image must not release an unresolved junction.
                    self.unmapped_signal_seen = True
            committed = bool(event and event["committed"])
            if event and route_valid:
                crossing_s = event["start"] - (0.0 if event["kind"] == "lane_change"
                                                   else self.core.front_reference_offset_m)
                if self.enter_permission and self.progress > crossing_s:
                    event["committed"] = committed = True

            if event and event["kind"] == "lane_change":
                # The PP lookahead is 4 + 0.35*v. Stop the rear axle before it
                # reaches the start, until five continuous seconds have elapsed.
                guard = max(5.0, 4.0 + 0.35 * (speed or 0.0))
                virtual_distance = (event["start"] - guard - self.progress
                                    + self.core.front_reference_offset_m + self.core.hold_distance_m)
                if not committed:
                    self.core.observe_line(max(0.0, virtual_distance), 1.0, True, ros_now, now, ros_now)
                gap_clear = self.lane_change_clear(direction, now)
                # Do not use a lane-change event to bypass a visible red light.
                permitted = not signal_ok or signal_permits(signal.value.state, "STRAIGHT", self.right_on_green)
                if self.require_context:
                    # Manual lane changes have no assigned traffic-light head.
                    # UNKNOWN association must not mean a visible red is absent.
                    permitted = (objects is not None and not objects.value.objects
                                 and line is not None and not line.value.valid)
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
                            candidate = self.progress + measured - (speed or 0)*max(0, ros_now-line.stamp)
                            if abs(candidate-event["start"]) <= 2.0:
                                # A noisy observation cannot push the guard ahead.
                                event["start"] = min(event["start"], candidate)
                                event["camera_line_confirmed"] = True
                                distance = event["start"] - self.progress
                    self.core.observe_line(max(0.0, distance), 1.0, True, ros_now, now, ros_now)
                    permitted = permitted and (event.get("stop_s") is not None
                                               or event.get("camera_line_confirmed", False))
                elif (line is not None and line.stamp > self.last_observed_line_stamp
                        and line.value.header.frame_id == "base_link" and not committed
                        and (self.progress is None or self.progress >= self.ignore_line_until_s)):
                    self.core.observe_line(line.value.distance_m, line.value.confidence, line.value.valid,
                                           line.stamp, line.received, ros_now)
                    self.last_observed_line_stamp = line.stamp
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
                                                  1., True, ros_now, now, ros_now)
                self.next_guard_core.observe_signal("RED", 1., True, ros_now, now, ros_now)
                guard = self.next_guard_core.update(now, ros_now, speed)
                if guard.accel_limit < decision.accel_limit or guard.brake > decision.brake:
                    decision = Decision(guard.mode, "next_junction_guard",
                                        min(decision.accel_limit, guard.accel_limit),
                                        max(decision.brake, guard.brake), guard.target_speed_kph, guard.distance_m)
            else:
                self.next_guard_id = self.next_guard_core = None
            reasons = []
            # UNKNOWN/no line is a valid new image observation. No new stamped
            # observations is a dead camera/inference stream, including mid-turn.
            if self.require_camera_stream and (signal is None or line is None):
                reasons.append("camera_observation_stream_stale")
            if self.require_context:
                if objects is None:
                    reasons.append("signal_observation_stale")
                if not self.contexts:
                    reasons.append("route_context_unavailable")
                if self.unmapped_signal_seen:
                    reasons.append("unmapped_signal_or_stopline")
                if not route_valid:
                    reasons.append("signal_localization_unreliable")
                if (event and not committed and event["kind"] != "lane_change"
                        and objects is not None and objects.value.objects
                        and self.selection.selected_id is None):
                    reasons.append("unassociated_visible_signal")
                if (event and route_valid and line is not None and line.value.valid
                        and line.value.header.frame_id == "base_link"
                        and finite(line.value.distance_m) and 0 <= line.value.distance_m <= 50
                        and finite(line.value.confidence) and 0.5 <= line.value.confidence <= 1.):
                    observed_s = (self.progress + line.value.distance_m
                                  - speed * max(0., ros_now - line.stamp))
                    if not committed and observed_s < event["start"] - 2.:
                        reasons.append("unmapped_stopline_before_context")
                    elif (committed and line.value.distance_m > self.core.front_reference_offset_m
                          and not any(abs(observed_s-c["start"]) <= 2. for c in self.contexts
                                      if c["id"] not in self.completed)):
                        reasons.append("unmapped_stopline_during_maneuver")
            if speed is None:
                reasons.append("odometry_or_route_unavailable")
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
            self.enter_permission = bool(event and route_valid and ready and (permitted or committed)
                                         and decision.mode == "NOMINAL" and not reasons)
            if reasons:
                decision = Decision("SAFE_STOP", ",".join(reasons), 0.0, 1.0, 0.0, decision.distance_m)
            output = overlay(nominal, decision)
            self.output_pub.publish(output)
            self.state_pub.publish(String(data=json.dumps({
                "mode": decision.mode, "reason": decision.reason, "event": event,
                "direction": direction, "signal": signal.value.state if signal else "STALE",
                "indicator_ready": ready, "lamp_udp_enabled": self.lamp_enabled,
                "permission": self.enter_permission, "progress_s_m": self.progress,
                "front_bumper_distance_m": decision.distance_m,
                "target_clearance_m": self.core.hold_distance_m,
                "route_context_count": len(self.contexts),
                "route_context_error": self.context_load_error,
                "next_junction_guard_id": self.next_guard_id,
                "signal_selection_reason": self.selection.reason if self.require_context else "legacy_unassociated",
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
