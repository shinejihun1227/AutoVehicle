"""Exercise real dynamic-path control callbacks with ROS transport replaced."""
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import unittest
from test_node_startup import Message
from curvature_speed_purepursuit.planner import PathPoint


class AdaptivePathTest(unittest.TestCase):
    def setUp(self):
        self.now=100.0
        ros=Mock(); ros.get_param.side_effect=lambda key,default=None: default
        ros.Time.now.side_effect=lambda: NS(to_sec=lambda:self.now)
        ros.get_time.side_effect=lambda:self.now
        ros.Publisher.side_effect=lambda *args,**kw: Mock()
        modules={'rospy':ros,'geometry_msgs.msg':NS(PointStamped=Message,PoseStamped=Message),
                 'morai_msgs.msg':NS(CtrlCmd=Message),'nav_msgs.msg':NS(Odometry=Message,Path=Message),
                 'std_msgs.msg':NS(Bool=Message,Float64=Message,String=Message)}
        spec=importlib.util.spec_from_file_location('adaptive_under_test',Path(__file__).resolve().parents[1]/'scripts/adaptive_curvature_purepursuit_node.py')
        self.module=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,modules): spec.loader.exec_module(self.module)
        self.module.time=NS(monotonic=lambda:self.now)
        self.module.load_path_file=lambda _: [PathPoint(float(x),0.0) for x in range(101)]
        self.n=self.module.AdaptiveCurvaturePurePursuit()

    def path(self, curve=False):
        msg=Message(); msg.header=NS(frame_id='map',seq=1,stamp=NS(to_sec=lambda:self.now))
        for i in range(101):
            p=Message()
            p.pose.position.x=10*math.sin(i*0.01) if curve else float(i)
            p.pose.position.y=10*(1-math.cos(i*0.01)) if curve else 0.0
            msg.poses.append(p)
        return msg

    def tick(self, stop=False):
        self.now+=0.05
        odom=Message(); odom.pose=NS(pose=odom.pose)
        odom.header=NS(frame_id='map',stamp=NS(to_sec=lambda:self.now))
        odom.twist=NS(twist=NS(linear=NS(x=0.0,y=0.0)))
        self.n._odom_cb(odom); self.n._stop_cb(Message(stop)); self.n._control_cb(None)
        return self.n.command_pub.publish.call_args.args[0]

    def test_no_odometry_brakes_instead_of_silently_publishing_nothing(self):
        self.n._control_cb(None)
        self.assertEqual(self.n.command_pub.publish.call_args.args[0].brake,1.0)

    def test_managed_stop_resets_speed_and_restart_ramps_from_zero(self):
        self.n._active_path_cb(self.path())
        for _ in range(8): self.assertGreater(self.tick().accel,0.0)
        command=self.tick(stop=True)
        self.assertEqual((command.accel,command.brake),(0.0,1.0))
        self.assertEqual(self.n.command_speed_mps,0.0)
        self.tick()
        self.assertLessEqual(self.n.command_speed_mps,0.051)

    def test_replacement_curve_reduces_speed_and_turns_left(self):
        self.n._active_path_cb(self.path())
        straight=self.n.active_speed_profile[0]
        self.n._active_path_cb(self.path(curve=True))
        self.assertLess(self.n.active_speed_profile[0],straight)
        self.assertGreater(self.tick().steering,0.0)

    def test_empty_or_nonfinite_path_immediately_invalidates_previous_path(self):
        self.n._active_path_cb(self.path())
        p=self.path(); p.poses=[]; self.n._active_path_cb(p)
        self.assertEqual(self.tick().brake,1.0)
        self.n._active_path_cb(self.path())
        p=self.path(); p.poses[5].pose.position.y=float('nan'); self.n._active_path_cb(p)
        self.assertEqual(self.tick().brake,1.0)

    def test_expired_manager_signal_cannot_keep_accelerating(self):
        self.n._active_path_cb(self.path()); self.tick()
        self.now+=2.0; self.n._control_cb(None)
        self.assertEqual(self.n.command_pub.publish.call_args.args[0].brake,1.0)


if __name__=='__main__': unittest.main()
