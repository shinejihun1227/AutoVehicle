#!/usr/bin/env python3
"""Summarize which integrated driving layer is requesting or causing a stop."""

from __future__ import annotations

import json
import math
import threading
import time

import rospy
from morai_perception_msgs.msg import SafetyStop
from std_msgs.msg import Bool, String


STATUS_TOPICS = {
    "/control/mux_status": "mux",
    "/control/curvature_status": "adaptive_pp",
    "/control/stopline_status": "stopline",
    "/highway_lane_strategy/state": "highway_strategy",
    "/route_mission_gate/status": "route_gate",
    "/roundabout_merge_gate/status": "roundabout_gate",
    "/avoidance_path_manager/plan_status": "avoidance_manager",
    "/bypass_lane_guard/plan_status": "bypass_guard",
    "/avoidance_frenet_debug/plan_status": "frenet_planner",
}
STATUS_TOPICS_BY_NAME = {name: topic for topic, name in STATUS_TOPICS.items()}

BOOL_TOPICS = {
    "/highway_lane_strategy/stop_required": "highway_strategy_stop",
    "/roundabout_merge_gate/stop_required": "roundabout_gate_stop",
    "/roundabout_merge_gate/allowed": "roundabout_gate_allowed",
    "/route_mission_gate/ready": "route_mission_ready",
    "/planning/highway_route_active": "highway_route_active",
    "/planning/highway_lane_change_request": "highway_lane_change_request",
    "/planning/highway_handoff_due": "highway_handoff_due",
    "/planning/merge_request": "roundabout_merge_request",
    "/perception/traffic_light/stop_required": "traffic_light_stop",
    "/perception/pedestrian_crossing/stop_required": "pedestrian_stop",
    "/perception/intersection/driving_unavailable": "intersection_stop",
    "/avoidance_path_manager/stop_required": "avoidance_stop",
}
BOOL_TOPICS_BY_NAME = {name: topic for topic, name in BOOL_TOPICS.items()}


def _compact_status(name, data):
    keys = {
        "mux": ("mode", "reasons"),
        "adaptive_pp": ("stop", "reason", "target_speed_kph", "measured_speed_kph"),
        "stopline": ("mode", "reason", "stop_requested", "holding", "distance_m"),
        "highway_strategy": ("state", "active", "stop", "reason", "target_speed_mps",
                            "lane_changes_done", "handoff_permitted", "lead_emergency_brake"),
        "route_gate": ("ready", "progress_s_m", "highway_route", "camera_highway",
                       "highway_request", "handoff_configured", "handoff_due",
                       "handoff_unconfigured_at_end", "merge_request", "roundabout_calibrated"),
        "roundabout_gate": ("state", "reason", "requested", "calibrated", "committed",
                            "stop_required", "allowed", "route_progress_s_m", "blockers"),
        "avoidance_manager": ("state", "stop_required", "active_source", "freshness",
                              "planner", "commit"),
        "bypass_guard": ("planner_ready", "avoidance_required", "safe_path_available",
                         "selected_kind", "selected_side", "seq"),
        "frenet_planner": ("planner_ready", "avoidance_required", "safe_path_available",
                           "selected_kind", "selected_side", "seq"),
    }.get(name, ())
    return {key: data[key] for key in keys if key in data}


def _number_or_none(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


class StopCauseMonitor:
    def __init__(self):
        rospy.init_node("final_ws_stop_cause_monitor", anonymous=False)
        self.timeout_s = max(0.2, float(rospy.get_param("~status_timeout_s", 1.0)))
        self.rate_hz = max(0.2, float(rospy.get_param("~report_rate_hz", 2.0)))
        self.lock = threading.RLock()
        self.status = {}
        self.flags = {}
        self.safety = None
        self.last_signature = None
        self.last_report_at = 0.0
        self.report_pub = rospy.Publisher(
            "/diagnostics/drive_stop_report", String, queue_size=1, latch=True
        )

        for topic, name in STATUS_TOPICS.items():
            rospy.Subscriber(topic, String, self._status_cb, callback_args=name, queue_size=1)
        for topic, name in BOOL_TOPICS.items():
            rospy.Subscriber(topic, Bool, self._bool_cb, callback_args=name, queue_size=1)
        rospy.Subscriber(
            "/detection/fused_safety_stop", SafetyStop, self._safety_cb, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._report)
        rospy.loginfo("Stop cause monitor publishing /diagnostics/drive_stop_report")

    def _status_cb(self, msg, name):
        now = time.monotonic()
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                data = {"raw": msg.data[:300]}
        except (TypeError, ValueError):
            data = {"raw": msg.data[:300]}
        with self.lock:
            self.status[name] = {"at": now, "data": data}

    def _bool_cb(self, msg, name):
        with self.lock:
            self.flags[name] = {"at": time.monotonic(), "value": bool(msg.data)}

    def _safety_cb(self, msg):
        with self.lock:
            self.safety = {
                "at": time.monotonic(),
                "stop_required": bool(getattr(msg, "stop_required", False)),
                "distance_m": _number_or_none(getattr(msg, "distance_m", None)),
                "confidence": _number_or_none(getattr(msg, "confidence", None)),
                "reason": str(getattr(msg, "reason", "")),
            }

    def _report(self, _event):
        now = time.monotonic()
        with self.lock:
            status = dict(self.status)
            flags = dict(self.flags)
            safety = None if self.safety is None else dict(self.safety)

        component_status = {}
        causes = []
        missing_status = []
        stale_status = []
        for name, entry in status.items():
            age = now - entry["at"]
            data = entry["data"]
            component_status[name] = {
                "age_s": round(age, 2),
                "stale": age > self.timeout_s,
                "status": _compact_status(name, data),
            }
            if age > self.timeout_s:
                stale_status.append(STATUS_TOPICS_BY_NAME[name])
                continue
            if name == "mux" and data.get("mode") == "stop":
                reasons = data.get("reasons") or ["mux_stop"]
                causes.append({"source": name, "reason": reasons})
            elif name == "adaptive_pp" and data.get("stop"):
                causes.append({"source": name, "reason": data.get("reason", "controller_stop")})
            elif name == "stopline" and data.get("mode") not in (None, "NOMINAL", "disabled"):
                causes.append({"source": name, "reason": data.get("reason", data.get("mode"))})
            elif name == "highway_strategy" and data.get("stop"):
                causes.append({"source": name, "reason": data.get("reason", "highway_stop")})
            elif name == "avoidance_manager" and data.get("stop_required"):
                causes.append({
                    "source": name,
                    "reason": data.get("active_source") or data.get("state") or "avoidance_stop",
                    "state": data.get("state"),
                    "planner": data.get("planner", {}),
                    "freshness": data.get("freshness", {}),
                })
            elif (name in ("bypass_guard", "frenet_planner")
                  and data.get("avoidance_required")
                  and not data.get("safe_path_available")):
                causes.append({"source": name, "reason": "no_safe_avoidance_path"})
            elif name == "route_gate" and data.get("ready") is False:
                reason = ("highway_handoff_unconfigured_at_end"
                          if data.get("handoff_unconfigured_at_end")
                          else "route_mission_not_ready")
                causes.append({"source": name, "reason": reason})
            elif name == "roundabout_gate" and data.get("stop_required"):
                causes.append({"source": name, "reason": data.get("reason", "merge_gate_stop")})

        for topic, name in STATUS_TOPICS.items():
            if name not in component_status:
                component_status[name] = {
                    "age_s": None, "stale": True, "no_message": True, "status": {}
                }
                missing_status.append(topic)

        flag_summary = {}
        stale_flags = []
        for name, entry in flags.items():
            age = now - entry["at"]
            value = entry["value"]
            flag_summary[name] = {"value": value, "age_s": round(age, 2), "stale": age > self.timeout_s}
            if age > self.timeout_s:
                stale_flags.append(BOOL_TOPICS_BY_NAME[name])
            if age <= self.timeout_s and value and name.endswith("_stop"):
                causes.append({"source": name, "reason": "stop_flag_true"})

        missing_flags = []
        for topic, name in BOOL_TOPICS.items():
            if name not in flag_summary:
                flag_summary[name] = {
                    "value": None, "age_s": None, "stale": True, "no_message": True
                }
                missing_flags.append(topic)

        safety_summary = None
        missing_safety = safety is None
        stale_safety = False
        if safety is not None:
            age = now - safety["at"]
            stale_safety = age > self.timeout_s
            safety_summary = {
                "stop_required": safety["stop_required"],
                "distance_m": safety["distance_m"],
                "confidence": safety["confidence"],
                "reason": safety["reason"],
                "age_s": round(age, 2),
                "stale": stale_safety,
            }
            if not stale_safety and safety["stop_required"]:
                causes.append({
                    "source": "fused_safety_stop",
                    "reason": safety["reason"] or "safety_stop_true",
                })
        else:
            safety_summary = {"no_message": True, "stale": True}

        report = {
            "stamp": rospy.Time.now().to_sec(),
            "active_stop_causes": causes,
            "statuses": component_status,
            "missing_status_topics": missing_status,
            "stale_status_topics": stale_status,
            "flags": flag_summary,
            "missing_flag_topics": missing_flags,
            "stale_flag_topics": stale_flags,
            "fused_safety_missing": missing_safety,
            "fused_safety_stale": stale_safety,
            "fused_safety": safety_summary,
        }
        payload = json.dumps(report, ensure_ascii=False, separators=(",", ":"))
        self.report_pub.publish(String(data=payload))
        signature = json.dumps({
            "causes": causes,
            "missing_status": missing_status,
            "stale_status": stale_status,
            "missing_flags": missing_flags,
            "stale_flags": stale_flags,
            "missing_safety": missing_safety,
            "stale_safety": stale_safety,
        }, ensure_ascii=False, sort_keys=True)
        if signature != self.last_signature or now - self.last_report_at >= 5.0:
            if causes:
                rospy.logwarn("DRIVE STOP CAUSES: %s", json.dumps(causes, ensure_ascii=False))
            elif (missing_status or stale_status or missing_flags or stale_flags
                  or missing_safety or stale_safety):
                rospy.logwarn(
                    "DRIVE STATUS INPUTS MISSING/STALE: %s",
                    json.dumps({
                        "missing_status": missing_status, "stale_status": stale_status,
                        "missing_flags": missing_flags, "stale_flags": stale_flags,
                        "missing_safety": missing_safety, "stale_safety": stale_safety,
                    }, ensure_ascii=False),
                )
            else:
                rospy.loginfo("DRIVE STOP CAUSES: none currently asserted")
            self.last_signature = signature
            self.last_report_at = now


if __name__ == "__main__":
    try:
        StopCauseMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
