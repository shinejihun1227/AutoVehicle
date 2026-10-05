#!/usr/bin/env python3
"""Read-only local practice judge; never publishes vehicle control commands."""

import csv
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import uuid

import rospy
from morai_msgs.msg import EgoVehicleStatus
from morai_perception_msgs.msg import SensorQuality, TrafficLight
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse

from mission_evaluator.course import Course, Route, LaneMap, finite_number
from mission_evaluator.competition import CompetitionRun, best_attempt


def json_text(value):
    return json.dumps(value,ensure_ascii=False,allow_nan=False,sort_keys=True)


class CompetitionLapNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.rules_file = Path(rospy.get_param("~rules_file"))
        self.rules = json.loads(self.rules_file.read_text(encoding="utf-8"))
        self.path_file = Path(rospy.get_param("~path_file"))
        self.course = Course(Route.load(self.path_file),self.rules,rospy.get_param("~mgeo_path"))
        self.auto_start = bool(rospy.get_param("~auto_start",True))
        self.output_root = Path(rospy.get_param("~output_dir",str(Path.home()/".ros"/"competition_runs"))).expanduser()
        self.output_root.mkdir(parents=True,exist_ok=True)
        self.timeout = self.rules["practice"]["input_timeout_sec"]
        self.lane_mode = rospy.get_param("~lane_mode","map_estimate")
        if self.lane_mode not in ("map_estimate","external","disabled"):
            raise ValueError("lane_mode must be map_estimate, external or disabled")
        self.lane_map = None
        self.contexts, self.signal_context_error = [], None
        signal_map = rospy.get_param("~signal_mgeo_path")
        if self.lane_mode == "map_estimate":
            try:
                self.lane_map = LaneMap(Path(signal_map)/"lane_boundary_set.json",self.rules["practice"])
            except (ValueError,OSError,KeyError) as exc:
                rospy.logerr("Lane evaluation unavailable: %s",exc)
        try:
            from turn_signal_controller.route_context import load_route_contexts
            self.contexts = load_route_contexts(signal_map,self.course.route.points,self.course.route.s)
        except (ImportError,ValueError,OSError,KeyError) as exc:
            self.signal_context_error = str(exc)
            rospy.logerr("Signal evaluation unavailable: %s",exc)
        self.attempts = []
        self.status_pub = rospy.Publisher("/evaluation/status",String,queue_size=10,latch=True)
        self.event_pub = rospy.Publisher("/evaluation/event",String,queue_size=100)
        self.result_pub = rospy.Publisher("/evaluation/result",String,queue_size=1,latch=True)
        self.best_pub = rospy.Publisher("/evaluation/best_attempt",String,queue_size=1,latch=True)
        self.finished_pub = rospy.Publisher("/evaluation/finished",Bool,queue_size=1,latch=True)
        self.new_run()
        rospy.Subscriber(rospy.get_param("~odometry_topic","/localization/odometry"),Odometry,self.pose_callback,queue_size=100)
        rospy.Subscriber(rospy.get_param("~ego_topic","/Ego_topic"),EgoVehicleStatus,self.speed_callback,queue_size=100)
        rospy.Subscriber(rospy.get_param("~quality_topic","/localization/sensor_quality"),SensorQuality,self.quality_callback,queue_size=10)
        rospy.Subscriber(rospy.get_param("~collision_topic","/evaluation/collision_snapshot"),String,self.collision_callback,queue_size=100)
        rospy.Subscriber(rospy.get_param("~maneuver_topic","/control/maneuver_status"),String,self.maneuver_callback,queue_size=50)
        rospy.Subscriber(rospy.get_param("~traffic_topic","/perception/traffic_light/directional_state"),TrafficLight,self.traffic_callback,queue_size=50)
        if self.lane_mode == "external":
            rospy.Subscriber(rospy.get_param("~lane_topic","/evaluation/lane_snapshot"),String,self.lane_callback,queue_size=100)
        rospy.Service("/evaluation/start",Trigger,self.start_service)
        rospy.Service("/evaluation/reset",Trigger,self.reset_service)
        rospy.Service("/evaluation/abort",Trigger,self.abort_service)
        rospy.Service("/evaluation/approve_recovery",Trigger,self.recovery_service)
        rospy.on_shutdown(self.shutdown)
        self.thread = threading.Thread(target=self.publish_loop,daemon=True)
        self.thread.start()
        rospy.logwarn("PRACTICE estimate only. Route %.3fm, highway %.3f..%.3fm; outputs %s",
                      self.course.route.length,*self.course.highway,str(self.directory))
        rospy.logwarn("Evaluation clock is ROS time. Without /clock, pausing MORAI does NOT pause the stopwatch.")

    def new_run(self):
        self.run = CompetitionRun(self.course,self.auto_start)
        self.stamps, self.values = {}, {}
        self.collision_sequence = None
        self.reported = 0
        self.saved = False
        self.run_id = time.strftime("%Y%m%d_%H%M%S")+"_"+uuid.uuid4().hex[:10]
        self.directory = self.output_root/self.run_id
        self.directory.mkdir(exist_ok=False)
        self.journal = (self.directory/"events.jsonl").open("x",encoding="utf-8")
        metadata = {"run_id":self.run_id,"rules":self.rules,"course":self.course.report(),
                    "route_sha256":hashlib.sha256(self.path_file.read_bytes()).hexdigest(),
                    "rules_sha256":hashlib.sha256(self.rules_file.read_bytes()).hexdigest(),
                    "clock":"ROS /clock" if rospy.get_param("/use_sim_time",False) else "ROS wall time",
                    "lane_mode":self.lane_mode,"signal_context_count":len(self.contexts),
                    "signal_context_error":self.signal_context_error,"official_score":False}
        (self.directory/"metadata.json").write_text(json_text(metadata),encoding="utf-8")
        self.finished_pub.publish(Bool(data=False))
        self.result_pub.publish(String(data=json_text({"run_id":self.run_id,"state":"WAITING","completed":False})))

    def accept_stamp(self, channel, stamp, now):
        try:
            stamp = finite_number(stamp)
        except (ValueError,TypeError):
            self.run.unknown(now,channel+"_invalid_timestamp")
            return False
        if stamp < 0 or now-stamp > self.timeout or stamp-now > 0.05 or (channel in self.stamps and stamp <= self.stamps[channel]):
            self.run.unknown(now,channel+"_stale_duplicate_or_out_of_order")
            return False
        self.stamps[channel] = stamp
        return True

    def cached(self, channel, now, timeout=None):
        value = self.values.get(channel)
        return value[1] if value is not None and 0 <= now-value[0] <= (self.timeout if timeout is None else timeout) else None

    def speed_callback(self, message):
        with self.lock:
            now = rospy.get_time()
            self.run.tick(now)
            if not self.accept_stamp("ego",message.header.stamp.to_sec(),now):
                return
            try:
                # Competition Vehicle Status provides only longitudinal vx.
                speed = abs(finite_number(message.velocity.x))*3.6
            except (ValueError,TypeError):
                self.run.unknown(now,"ego_nonfinite_velocity_x")
                return
            self.values["speed"] = (now,speed)
            fresh_pose = self.run.pose is not None and now-self.run.pose[0] <= self.timeout and self.run.state == "RUNNING"
            highway = self.course.highway_active(self.run.progress) if fresh_pose else None
            if fresh_pose:
                # A speed packet may precede the pose for the same instant. Do
                # not charge +15s using the preceding side of a zone boundary.
                uncertainty = speed/3.6*(now-self.run.pose[0]) + self.rules["practice"]["speed_region_pose_error_m"]
                if min(abs(self.run.progress-boundary) for boundary in self.course.highway) <= uncertainty:
                    highway = None
                    self.run.unknown(now,"speed_region_boundary_time_alignment_uncertain")
            self.run.observe_speed(now,speed,highway)

    def pose_callback(self, message):
        with self.lock:
            now = rospy.get_time()
            if not self.accept_stamp("odometry",message.header.stamp.to_sec(),now):
                return
            if message.header.frame_id.lstrip("/") != "map":
                self.run.unknown(now,"odometry_not_in_map_frame")
                return
            try:
                position,q = message.pose.pose.position,message.pose.pose.orientation
                x,y = finite_number(position.x),finite_number(position.y)
                qx,qy,qz,qw = [finite_number(v) for v in (q.x,q.y,q.z,q.w)]
                norm = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
                if abs(norm-1) > 0.05:
                    raise ValueError("Invalid orientation quaternion")
                yaw = math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
                previous = self.run.pose
                previous_s = self.run.progress
                accepted = self.run.observe_pose(now,x,y,yaw,self.cached("speed",now))
                if not accepted:
                    return
                if self.run.active:
                    if previous is not None:
                        self.evaluate_signals(now,previous,(now,x,y,yaw),previous_s)
                    if self.lane_map is not None:
                        contact = self.lane_map.contact(x,y,yaw)
                        self.run.observe_lane(now,bool(contact),self.blackout(now),"estimated_wheel_map")
                        if not self.rules["practice"]["wheel_geometry_calibrated"]:
                            self.run.unknown(now,"wheel_geometry_not_calibrated")
                        if self.lane_map.skipped_mixed:
                            self.run.unknown(now,"mixed_lane_boundaries_not_evaluated")
            except (ValueError,TypeError,KeyError) as exc:
                self.run.unknown(now,"pose_or_map_invalid")
                rospy.logwarn_throttle(2,"Pose evaluation rejected: %s",exc)

    def quality_callback(self,message):
        with self.lock:
            now = rospy.get_time()
            if self.accept_stamp("quality",message.header.stamp.to_sec(),now):
                self.values["blackout"] = (now,bool(message.gps_blackout or message.state=="GPS_BLACKOUT"))

    def blackout(self,now):
        geographic = self.course.blackout(self.run.progress)
        if geographic is not None:
            return geographic
        self.run.unknown(now,"blackout_region_uses_sensor_estimate")
        return self.cached("blackout",now)

    def collision_callback(self,message):
        with self.lock:
            now = rospy.get_time()
            try:
                data = json.loads(message.data)
                if data.get("valid") is not True or not self.accept_stamp("collision",data["timestamp_sec"],now):
                    return
                if not isinstance(data["keys"],list) or any(not isinstance(k,str) or not k for k in data["keys"]):
                    raise ValueError("Invalid collision keys")
                sequence = int(data["sequence"])
                if self.collision_sequence is not None and sequence <= self.collision_sequence:
                    self.run.unknown(now,"collision_sequence_reset_or_reorder")
                    return
                self.collision_sequence = sequence
                self.run.tick(now)
                if data.get("protocol_verified") is not True:
                    self.run.unknown(now,"collision_wire_protocol_not_validated_in_simulator")
                if data.get("at_capacity",False):
                    self.run.unknown(now,"collision_packet_capacity_reached")
                self.run.observe_collision(now,data["keys"])
            except (ValueError,TypeError,KeyError):
                self.run.unknown(now,"collision_payload_invalid")

    def lane_callback(self,message):
        with self.lock:
            now = rospy.get_time()
            try:
                data = json.loads(message.data)
                if data.get("valid") is not True or not self.accept_stamp("external_lane",data["timestamp_sec"],now):
                    return
                if not isinstance(data["contact"],bool) or not isinstance(data["blackout_exempt"],bool):
                    raise ValueError("Explicit contact and geographic exemption required")
                self.run.tick(now)
                self.run.observe_lane(now,data["contact"],data["blackout_exempt"],"external_wheel_contact")
            except (ValueError,TypeError,KeyError):
                self.run.unknown(now,"lane_payload_invalid")

    def maneuver_callback(self,message):
        with self.lock:
            try:
                data = json.loads(message.data)
                if not isinstance(data,dict):
                    return
                self.values["maneuver"] = (rospy.get_time(),data)
            except (ValueError,TypeError):
                pass

    def traffic_callback(self,message):
        with self.lock:
            now = rospy.get_time()
            if self.accept_stamp("traffic",message.header.stamp.to_sec(),now):
                self.values["traffic"] = (message.header.stamp.to_sec(),message.state if message.valid else "UNKNOWN")

    def evaluate_signals(self,now,previous,current,previous_s):
        tuning = self.rules["practice"]
        for context in self.contexts:
            stop_s = context.get("stop_s")
            if stop_s is None:
                if previous_s < context["entry_s"] <= self.run.progress:
                    self.run.observe_signal(now,context["id"],None)
                continue
            if not previous_s-5 <= stop_s <= self.run.progress+5:
                continue
            route = self.course.route
            stop = route.point_at(stop_s)
            before,after = route.point_at(stop_s-0.3),route.point_at(stop_s+0.3)
            dx,dy = after.x-before.x,after.y-before.y
            length = math.hypot(dx,dy)
            if length <= 1e-9:
                continue
            normal = (dx/length,dy/length)
            def signed(p):
                # Front AXLE (3m), not front bumper (3.845m).
                fx = p[1]+tuning["wheelbase_m"]*math.cos(p[3])-stop.x
                fy = p[2]+tuning["wheelbase_m"]*math.sin(p[3])-stop.y
                return fx*normal[0]+fy*normal[1]
            if not signed(previous) < 0 <= signed(current):
                continue
            status = self.cached("maneuver",now,tuning["signal_source_timeout_sec"])
            traffic = self.cached("traffic",now,tuning["signal_source_timeout_sec"])
            permitted,state = None,"UNKNOWN"
            if status and traffic and isinstance(status.get("event"),dict):
                state = status.get("selected_signal_state")
                identity_ok = status["event"].get("id")==context["id"] and status.get("selected_signal_id") in context["signal_ids"]
                if identity_ok and context["direction"] in ("STRAIGHT","LEFT","RIGHT") and state==traffic and state in ("RED","YELLOW","GREEN","LEFT","RIGHT","GREEN_LEFT","GREEN_RIGHT","RED_LEFT","RED_RIGHT"):
                    allowed = {"GREEN":{"STRAIGHT"}|({"RIGHT"} if tuning["right_on_green"] else set()),
                               "GREEN_LEFT":{"STRAIGHT","LEFT"}|({"RIGHT"} if tuning["right_on_green"] else set()),
                               "GREEN_RIGHT":{"STRAIGHT","RIGHT"},"LEFT":{"LEFT"},"RED_LEFT":{"LEFT"},"RIGHT":{"RIGHT"},"RED_RIGHT":{"RIGHT"}}
                    permitted = context["direction"] in allowed.get(state,set())
                    self.run.unknown(now,"signal_uses_controller_receipt_time_not_judge_phase")
            self.run.observe_signal(now,context["id"],permitted,state=state,direction=context["direction"],front_reference="front_axle")

    def flush_events(self):
        for event in self.run.events[self.reported:]:
            encoded = json_text(dict(event,run_id=self.run_id))
            self.journal.write(encoded+"\n")
            self.journal.flush()
            self.event_pub.publish(String(data=encoded))
            if event["penalty_sec"]:
                rospy.logwarn("%s +%.1fs (total %.1fs)",event["reason"],event["penalty_sec"],self.run.penalty)
        self.reported = len(self.run.events)

    def result(self,now):
        value = self.run.snapshot(now)
        value.update(run_id=self.run_id,result_directory=str(self.directory))
        return value

    def save_result(self,now):
        if self.saved:
            return
        self.flush_events()
        result = self.result(now)
        (self.directory/"result.json").write_text(json_text(dict(result,events=self.run.events)),encoding="utf-8")
        with (self.directory/"events.csv").open("x",newline="",encoding="utf-8-sig") as stream:
            writer = csv.DictWriter(stream,fieldnames=["sequence","timestamp_sec","rule","reason","penalty_sec","source","details"])
            writer.writeheader()
            for event in self.run.events:
                writer.writerow(dict(event,details=json_text(event["details"])))
        self.attempts.append(result)
        self.saved = True
        self.result_pub.publish(String(data=json_text(result)))
        self.finished_pub.publish(Bool(data=result["completed"]))
        self.best_pub.publish(String(data=json_text({"attempts":len(self.attempts),"best":best_attempt(self.attempts)})))
        rospy.logwarn("LAP RESULT %s raw=%s penalty=%.3f adjusted=%s saved=%s",result["state"],result["lap_time_sec"],result["penalty_time_sec"],result["adjusted_time_sec"],self.directory)

    def publish_loop(self):
        while not self.stop.wait(0.05) and not rospy.is_shutdown():
            with self.lock:
                now = rospy.get_time()
                self.run.tick(now)
                if self.run.active and now-self.run.start_time > self.timeout:
                    for name in self.run.snapshot(now)["currently_missing_inputs"]:
                        self.run.unknown(now,name+"_input_unavailable")
                    if not self.contexts:
                        self.run.unknown(now,"signal_map_context_unavailable")
                try:
                    self.flush_events()
                    self.status_pub.publish(String(data=json_text(self.result(now))))
                    if self.run.state in self.run.TERMINAL:
                        self.save_result(now)
                except (OSError,ValueError) as exc:
                    rospy.logerr_throttle(2,"Evaluation output failed: %s",exc)

    def start_service(self,_request):
        with self.lock:
            return TriggerResponse(*self.run.start(rospy.get_time()))

    def reset_service(self,_request):
        with self.lock:
            now = rospy.get_time()
            self.run.end(now,"ABORTED","operator_reset")
            self.save_result(now)
            self.journal.close()
            self.new_run()
            return TriggerResponse(True,"new attempt armed; vehicle/control/planner are NOT reset")

    def abort_service(self,_request):
        with self.lock:
            self.run.end(rospy.get_time(),"ABORTED","operator_abort")
            return TriggerResponse(True,"evaluation aborted; vehicle control unchanged")

    def recovery_service(self,_request):
        with self.lock:
            self.run.tick(rospy.get_time())
            return TriggerResponse(*self.run.approve_recovery(rospy.get_time()))

    def shutdown(self):
        self.stop.set()
        with self.lock:
            try:
                self.run.end(rospy.get_time(),"ABORTED","node_shutdown")
                self.save_result(rospy.get_time())
            except (OSError,ValueError) as exc:
                rospy.logerr("Unable to save final evaluation: %s",exc)
            finally:
                self.journal.close()


if __name__ == "__main__":
    rospy.init_node("competition_lap_evaluator")
    CompetitionLapNode()
    rospy.spin()
