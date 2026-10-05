"""Deterministic practice scoring, independent of ROS and vehicle control.

Missing input is an unassessed interval, never proof of a clean lap. Observations
must arrive in a single, monotonically increasing evaluation clock domain.
"""

import math
from .course import circle_entry, finite_number
from .rules import SpeedRule, LaneContactRule


class CompetitionRun:
    TERMINAL = {"FINISHED", "FAILED", "INVALID", "ABORTED"}

    def __init__(self, course, auto_start=True):
        self.course, self.rules = course, course.rules
        self.tuning = self.rules["practice"]
        for key in ("time_limit_sec", "start_deadline_sec", "speed_limit_kph", "speed_penalty_sec",
                    "speed_repeat_sec", "lane_contact_sec", "lane_penalty_sec", "collision_penalty_sec",
                    "signal_penalty_sec", "recovery_penalty_sec"):
            if finite_number(self.rules[key]) <= 0:
                raise ValueError("Rule must be positive: " + key)
        if not 0 < self.rules["start_progress_ratio"] < 1:
            raise ValueError("Invalid start progress ratio")
        self.auto_start = auto_start
        self.state, self.reason = "WAITING", "waiting_at_start"
        self.start_time = self.end_time = self.last_clock = None
        self.pose = None
        self.progress = 0.0
        self.max_progress = 0.0
        self.passed = []
        self.start_mission = "WAITING"
        self.events, self.unassessed = [], set()
        self.penalty = 0.0
        self.observed, self.sources = {}, {}
        self.collision_objects = set()
        self.signal_crossings = set()
        self.speed_rule = SpeedRule(self.rules["speed_limit_kph"], self.rules["speed_penalty_sec"], self.rules["speed_repeat_sec"])
        self.lane_rule = LaneContactRule(self.rules["lane_contact_sec"], self.rules["lane_contact_sec"], self.rules["lane_penalty_sec"])

    @property
    def active(self):
        return self.state in ("RUNNING", "NEEDS_REVIEW")

    def event(self, now, rule, reason, penalty=0.0, source="estimated", **details):
        entry = {"sequence": len(self.events)+1, "timestamp_sec": now, "rule": rule,
                 "reason": reason, "penalty_sec": penalty, "source": source, "details": details}
        self.events.append(entry)
        self.penalty += penalty
        return entry

    def unknown(self, now, reason):
        if self.active and reason not in self.unassessed:
            self.unassessed.add(reason)
            self.event(now, "coverage", reason, source="unassessed")

    def tick(self, now):
        now = finite_number(now)
        if self.state in self.TERMINAL:
            return
        if self.last_clock is not None and now < self.last_clock - 1e-6:
            self.end_time = self.last_clock
            self.state, self.reason = "INVALID", "clock_moved_backwards_reset_required"
            self.event(self.last_clock, "run", self.reason)
            return
        self.last_clock = now
        if not self.active:
            return
        elapsed = now-self.start_time
        if elapsed > self.rules["time_limit_sec"]:
            self.end(now, "FAILED", "driving_time_limit_exceeded")
        elif self.start_mission != "SUCCESS" and elapsed > self.rules["start_deadline_sec"]:
            self.start_mission = "FAILED"
            self.end(now, "FAILED", "start_5_percent_deadline_missed")

    def end(self, now, state, reason):
        if self.state in self.TERMINAL:
            return
        self.state, self.reason, self.end_time = state, reason, now
        self.event(now, "run", reason)

    def start(self, now):
        self.tick(now)
        if self.state != "WAITING" or self.pose is None:
            return False, "fresh_start_pose_required"
        if now-self.pose[0] > self.tuning["input_timeout_sec"] or math.dist(self.pose[1:3], self.course.start[:2]) > self.course.radius:
            return False, "vehicle_not_at_start_or_pose_stale"
        self.state, self.reason, self.start_time = "RUNNING", "departure", now
        self.start_mission = "RUNNING"
        self.event(now, "run", "lap_started", source="estimated_pose", start_mode="motion" if self.auto_start else "manual")
        return True, "lap_started"

    def observe_pose(self, now, x, y, yaw, speed_kph=None):
        values = [finite_number(v) for v in (now,x,y,yaw)]
        now,x,y,yaw = values
        self.tick(now)
        if self.state in self.TERMINAL:
            return False
        previous = self.pose
        if self.state == "WAITING":
            match = self.course.route.project((x,y), upper=self.tuning["initial_search_m"])
            self.pose = (now,x,y,yaw)
            self.progress = match["s_m"]
            if self.auto_start and speed_kph is not None and speed_kph >= self.tuning["auto_start_speed_kph"]:
                self.start(now)
            return True
        if previous is None:
            return False
        dt = now-previous[0]
        if dt <= 0:
            return False
        if dt > self.tuning["max_pose_gap_sec"]:
            self.unknown(now, "pose_gap_checkpoint_continuity_unverified")
            self.state, self.reason = "NEEDS_REVIEW", "pose_gap_requires_recovery_or_reset"
        if self.state == "NEEDS_REVIEW":
            self.pose = (now,x,y,yaw)
            return False
        if speed_kph is None:
            self.unknown(now, "speed_missing_for_pose_continuity")
            return False
        speed = abs(finite_number(speed_kph))/3.6
        travel = math.dist(previous[1:3], (x,y))
        bound = speed*dt + self.tuning["projection_margin_m"]
        if travel > bound:
            self.state, self.reason = "NEEDS_REVIEW", "pose_jump_or_teleport"
            self.unknown(now, self.reason)
            self.pose = (now,x,y,yaw)
            return False
        match = self.course.route.project((x,y), max(0,self.progress-bound), self.progress+bound)
        progress = match["s_m"]
        if match["distance_m"] > self.tuning["route_corridor_m"]:
            self.state, self.reason = "NEEDS_REVIEW", "route_departure_estimated"
            self.unknown(now, self.reason)
            self.pose = (now,x,y,yaw)
            return False
        # Projection at a search-window edge must not hide an impossible jump.
        if abs(progress-self.progress) > bound+1e-6:
            return False
        self.progress = progress
        self.max_progress = max(self.max_progress, progress)
        self.pose = (now,x,y,yaw)
        self.observed["pose"] = now
        target = self.course.route.length*self.rules["start_progress_ratio"]
        if self.start_mission != "SUCCESS" and progress >= target:
            self.start_mission = "SUCCESS"
            self.event(now, "start", "start_5_percent_passed", progress_m=progress)
        checkpoints = self.course.checkpoints
        while len(self.passed) < len(checkpoints):
            cp = checkpoints[len(self.passed)]
            crossed = circle_entry(previous[1:3],(x,y),cp["xyz"],self.course.radius)
            if crossed is None or abs(progress-cp["s_m"]) > travel+self.course.radius+1:
                break
            self.passed.append(cp["id"])
            self.event(now, "checkpoint", "checkpoint_passed", checkpoint=cp["id"], progress_m=progress)
        if len(self.passed) < len(checkpoints):
            expected = checkpoints[len(self.passed)]
            if progress > expected["s_m"] + self.course.radius + 2:
                self.state, self.reason = "NEEDS_REVIEW", "checkpoint_skipped"
                self.event(now, "mission", self.reason, expected_checkpoint=expected["id"])
                return False
        finished = len(self.passed) == len(checkpoints) and progress >= self.course.route.length-self.course.radius
        if finished and circle_entry(previous[1:3],(x,y),self.course.start,self.course.radius) is not None:
            self.end(now, "FINISHED", "all_checkpoints_and_finish_reached")
        return True

    def _sample(self, channel, now, source):
        if not self.active:
            return False
        previous = self.observed.get(channel)
        if previous is not None and now <= previous:
            return False
        if previous is not None and now-previous > self.tuning["input_timeout_sec"]:
            self.unknown(now, channel+"_input_gap")
            if channel == "speed":
                self.speed_rule.update(now,0.0)
            if channel == "lane":
                self.lane_rule.update(now,False)
        self.observed[channel], self.sources[channel] = now, source
        return True

    def observe_speed(self, now, speed_kph, highway=None, source="competition_status_x"):
        speed = abs(finite_number(speed_kph))
        if not self._sample("speed",now,source):
            return
        if highway is None:
            self.unknown(now,"speed_region_unavailable")
            self.speed_rule.update(now,0.0)
            return
        for event in self.speed_rule.update(now,speed,highway):
            self.event(event.timestamp_sec,"speed",event.reason,event.penalty_sec,source,**event.details)

    def observe_lane(self, now, contact, exempt=None, source="estimated_wheel_map"):
        if not self._sample("lane",now,source):
            return
        if exempt is None or contact is None:
            self.unknown(now,"lane_contact_or_blackout_unavailable")
            self.lane_rule.update(now,False)
            return
        for event in self.lane_rule.update(now,bool(contact),bool(exempt)):
            self.event(event.timestamp_sec,"lane",event.reason,event.penalty_sec,source,**event.details)

    def observe_collision(self, now, keys, source="collision_udp"):
        if not self._sample("collision",now,source):
            return
        current = set(str(key) for key in keys)
        for key in sorted(current-self.collision_objects):
            self.event(now,"collision","collision_contact_started",self.rules["collision_penalty_sec"],source,object_key=key)
        # Only a new actual snapshot releases contact. A timeout is NOT release.
        self.collision_objects = current

    def observe_signal(self, now, crossing_id, permitted, source="estimated_camera_signal", **details):
        if not self.active or crossing_id in self.signal_crossings:
            return
        self.signal_crossings.add(crossing_id)
        if permitted is None:
            self.unknown(now,"signal_crossing_unassessed:"+crossing_id)
        else:
            self.event(now,"signal","signal_crossing_allowed" if permitted else "signal_crossing_violation",
                       0.0 if permitted else self.rules["signal_penalty_sec"],source,crossing_id=crossing_id,**details)

    def approve_recovery(self, now):
        if self.state != "NEEDS_REVIEW" or self.pose is None:
            return False, "no_pending_recovery"
        target = self.course.checkpoints[len(self.passed)-1] if self.passed else dict(xyz=self.course.start,s_m=0.0)
        if now-self.pose[0] > self.tuning["input_timeout_sec"] or math.dist(self.pose[1:3],target["xyz"][:2]) > self.course.radius:
            return False,"operator_must_restore_vehicle_to_last_checkpoint_first"
        self.progress = target["s_m"]
        self.state, self.reason = "RUNNING", "operator_recovery_approved"
        self.event(now,"recovery",self.reason,self.rules["recovery_penalty_sec"],"operator",checkpoint=self.passed[-1] if self.passed else "START")
        self.speed_rule.update(now,0.0)
        self.lane_rule.update(now,False)
        return True,self.reason

    def snapshot(self, now):
        stop = self.end_time if self.end_time is not None else now
        elapsed = None if self.start_time is None else max(0.0,stop-self.start_time)
        missing = []
        for name in ("pose","speed","lane","collision"):
            stamp = self.observed.get(name)
            if stamp is None or stop-stamp > self.tuning["input_timeout_sec"]:
                missing.append(name)
        return {"rules_version":self.rules["version"],"official_score":False,"assessment":"PRACTICE_ESTIMATE",
                "state":self.state,"reason":self.reason,"completed":self.state=="FINISHED",
                "start_mission":self.start_mission,"lap_time_sec":elapsed,"penalty_time_sec":self.penalty,
                "adjusted_time_sec":None if elapsed is None else elapsed+self.penalty,
                "progress_m":self.progress,"max_progress_m":self.max_progress,"route_length_m":self.course.route.length,
                "checkpoints_passed":list(self.passed),"checkpoint_count":len(self.course.checkpoints),
                "highway_speed_exception":self.course.highway_active(self.progress),
                "speed_limit_kph":None if self.course.highway_active(self.progress) else self.rules["speed_limit_kph"],
                "currently_missing_inputs":missing,"unassessed_reasons":sorted(self.unassessed),
                "input_sources":dict(self.sources),"event_count":len(self.events),
                "coverage_complete":not missing and not self.unassessed}


def best_attempt(attempts):
    """Practice ranking: valid completion first, then checkpoints and total time.

    INVALID/ABORTED/WAITING runs are not competition attempts. FAILED includes
    the published 60s/900s mission failures and retains checkpoint attainment.
    """
    candidates = [a for a in attempts if a.get("state") in ("FINISHED","FAILED") and a.get("adjusted_time_sec") is not None]
    if not candidates:
        return None
    return min(candidates,key=lambda a:(not a["completed"],0 if a["completed"] else -len(a["checkpoints_passed"]),a["adjusted_time_sec"]))
