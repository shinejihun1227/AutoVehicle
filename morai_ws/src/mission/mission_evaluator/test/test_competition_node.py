"""ROS callback contract tests with stubs: no ROS master, UDP or vehicle required."""

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import threading
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

from test_competition import PACKAGE, started


def fake_module(name,**attributes):
    module = ModuleType(name)
    module.__dict__.update(attributes)
    return module


class Message:
    def __init__(self,data=None):
        self.data = data


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self,message):
        self.messages.append(message)


fake_ros = fake_module("rospy",get_time=lambda:0.0,logwarn=lambda *a:None,logwarn_throttle=lambda *a:None)
stubs = {"rospy":fake_ros,"morai_msgs":fake_module("morai_msgs"),
         "morai_msgs.msg":fake_module("morai_msgs.msg",EgoVehicleStatus=object),
         "morai_perception_msgs":fake_module("morai_perception_msgs"),
         "morai_perception_msgs.msg":fake_module("morai_perception_msgs.msg",SensorQuality=object,TrafficLight=object),
         "nav_msgs":fake_module("nav_msgs"),"nav_msgs.msg":fake_module("nav_msgs.msg",Odometry=object),
         "std_msgs":fake_module("std_msgs"),"std_msgs.msg":fake_module("std_msgs.msg",String=Message,Bool=Message),
         "std_srvs":fake_module("std_srvs"),"std_srvs.srv":fake_module("std_srvs.srv",Trigger=object,TriggerResponse=lambda a,b:(a,b))}
with patch.dict("sys.modules",stubs):
    spec = importlib.util.spec_from_file_location("competition_node_under_test",PACKAGE/"scripts"/"competition_lap_node.py")
    node_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(node_module)


def header(stamp,frame="map"):
    return NS(stamp=NS(to_sec=lambda:stamp),frame_id=frame)


class NodeTests(unittest.TestCase):
    def setUp(self):
        self.node = node_module.CompetitionLapNode.__new__(node_module.CompetitionLapNode)
        self.node.lock = threading.RLock()
        self.node.run = started()
        self.node.course = self.node.run.course
        self.node.course.blackout = lambda progress:None
        self.node.rules = self.node.run.rules
        self.node.timeout = .7
        self.node.stamps,self.node.values = {},{}
        self.node.collision_sequence = None
        self.node.lane_map = None
        self.node.contexts = []
        self.clock = 0.0
        fake_ros.get_time = lambda:self.clock

    def ego(self,stamp,vx,vy=0):
        return NS(header=header(stamp),velocity=NS(x=vx,y=vy,z=0))

    def pose(self,stamp,x=0,y=0,frame="map",q=None):
        return NS(header=header(stamp,frame),pose=NS(pose=NS(position=NS(x=x,y=y,z=0),
                  orientation=q or NS(x=0,y=0,z=0,w=1))))

    def test_speed_uses_only_x_mps_to_kph(self):
        self.clock = .1
        self.node.speed_callback(self.ego(.1,10,10000))
        self.assertEqual(self.node.values["speed"][1],36)
        self.assertEqual(self.node.run.penalty,0)

    def test_reverse_speed_absolute(self):
        self.clock = .1
        self.node.speed_callback(self.ego(.1,-20))
        self.assertEqual(self.node.run.penalty,15)

    def test_zone_boundary_not_scored_using_previous_pose(self):
        self.clock = .1
        self.node.run.progress = 29.9
        self.node.speed_callback(self.ego(.1,20))
        self.assertEqual(self.node.run.penalty,0)
        self.assertIn("speed_region_boundary_time_alignment_uncertain",self.node.run.unassessed)

    def test_stale_ego_is_not_freshened_on_receipt(self):
        self.clock = 10
        self.node.speed_callback(self.ego(0,100))
        self.assertNotIn("speed",self.node.values)
        self.assertEqual(self.node.run.penalty,0)

    def test_duplicate_and_future_stamps_rejected(self):
        self.assertTrue(self.node.accept_stamp("x",1,1))
        self.assertFalse(self.node.accept_stamp("x",1,1))
        self.assertFalse(self.node.accept_stamp("x",2,1))

    def test_nonfinite_speed_does_not_score(self):
        self.clock = .1
        self.node.speed_callback(self.ego(.1,float("nan")))
        self.assertNotIn("speed",self.node.values)

    def test_pose_must_use_map_frame(self):
        self.clock = .1
        self.node.values["speed"] = (.1,36)
        self.node.pose_callback(self.pose(.1,1,frame="odom"))
        self.assertEqual(self.node.run.progress,0)

    def test_invalid_quaternion_rejected(self):
        self.clock = .1
        self.node.values["speed"] = (.1,36)
        self.node.pose_callback(self.pose(.1,1,q=NS(x=0,y=0,z=0,w=0)))
        self.assertEqual(self.node.run.progress,0)

    def test_pose_callback_updates_route_independently_of_planner(self):
        self.clock = .1
        self.node.values["speed"] = (.1,36)
        self.node.pose_callback(self.pose(.1,1))
        self.assertEqual(self.node.run.progress,1)

    def test_collision_clear_and_rehit_between_timer_ticks(self):
        for time_value,seq,keys in ((.1,1,["1:0"]),(.2,2,[]),(.3,3,["1:0"])):
            self.clock = time_value
            self.node.collision_callback(Message(json.dumps(dict(timestamp_sec=time_value,sequence=seq,keys=keys,valid=True,protocol_verified=True))))
        self.assertEqual(self.node.run.penalty,30)

    def test_collision_reordered_sequence_does_not_release(self):
        for time_value,seq,keys in ((.1,10,["1:1"]),(.2,9,[]),(.3,11,["1:1"])):
            self.clock = time_value
            self.node.collision_callback(Message(json.dumps(dict(timestamp_sec=time_value,sequence=seq,keys=keys,valid=True))))
        self.assertEqual(self.node.run.penalty,15)
        self.assertIn("collision_sequence_reset_or_reorder",self.node.run.unassessed)

    def test_external_lane_requires_explicit_boolean(self):
        self.clock = .1
        self.node.lane_callback(Message(json.dumps(dict(timestamp_sec=.1,valid=True,contact="false",blackout_exempt=False))))
        self.assertNotIn("lane",self.node.run.observed)

    def test_missing_blackout_does_not_grant_exemption(self):
        self.assertIsNone(self.node.blackout(.1))

    def signal(self,state="RED",traffic="RED",identity="J1",direction="STRAIGHT",source_time=.2):
        self.node.contexts = [dict(id="J1",stop_s=10,entry_s=10,direction=direction,signal_ids=["head1"])]
        self.node.run.progress = 8
        self.node.values["maneuver"] = (source_time,dict(event=dict(id=identity),selected_signal_id="head1",selected_signal_state=state))
        self.node.values["traffic"] = (source_time,traffic)
        self.node.evaluate_signals(.2,(.1,6,0,0),(.2,8,0,0),6)

    def test_red_at_front_axle_crossing_penalized_once(self):
        self.signal()
        self.signal()
        self.assertEqual(self.node.run.penalty,15)

    def test_green_crossing_no_penalty(self):
        self.signal("GREEN","GREEN")
        self.assertEqual(self.node.run.penalty,0)
        self.assertEqual(self.node.run.events[-1]["reason"],"signal_crossing_allowed")

    def test_other_intersection_cannot_grant_green(self):
        self.signal("GREEN","GREEN",identity="OTHER")
        self.assertIn("signal_crossing_unassessed:J1",self.node.run.unassessed)

    def test_stale_or_disagreeing_signal_unassessed(self):
        self.signal("GREEN","RED")
        self.assertIn("signal_crossing_unassessed:J1",self.node.run.unassessed)

    def test_unknown_direction_not_false_violation(self):
        self.signal(direction="UNKNOWN")
        self.assertEqual(self.node.run.penalty,0)
        self.assertIn("signal_crossing_unassessed:J1",self.node.run.unassessed)

    def test_report_files_and_frozen_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            node = self.node
            node.directory = Path(directory)
            node.run_id = "unit-test"
            node.journal = (node.directory/"events.jsonl").open("x",encoding="utf-8")
            node.reported,node.saved,node.attempts = 0,False,[]
            node.event_pub,node.result_pub,node.finished_pub,node.best_pub = Publisher(),Publisher(),Publisher(),Publisher()
            node.run.observe_collision(.1,["1:4"])
            node.run.end(2,"FAILED","test_termination")
            node.save_result(2)
            node.save_result(3)
            node.journal.close()
            data = json.loads((node.directory/"result.json").read_text(encoding="utf-8"))
            self.assertEqual(data["lap_time_sec"],2)
            self.assertEqual(data["adjusted_time_sec"],17)
            self.assertEqual(len(node.attempts),1)
            self.assertTrue((node.directory/"events.csv").exists())
            self.assertFalse(data["official_score"])


if __name__ == "__main__":
    unittest.main()
