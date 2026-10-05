"""Active-path control contracts using real geometry/PI and mocked ROS I/O."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'src'))
sys.path.insert(0, str(PACKAGE.parents[1] / 'control/purepursuit_mgeo/src'))
from curvature_speed_purepursuit.planner import PathPoint


class Message:
    def __init__(self, data=None):
        self.data = data
        self.header = NS(stamp=None, frame_id='map', seq=1)
        self.poses = []
        self.pose = NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))
        self.point = NS(x=0., y=0., z=0.)
        self.longlCmdType = 1
        self.accel = self.brake = self.steering = self.velocity = self.acceleration = 0.


class AdaptiveTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        params = {'~max_speed_kph': 30., '~use_target_speed_override': True, '~enable_merge_gate': True}
        ros = Mock()
        ros.get_param.side_effect = lambda key, default=None: params.get(key, default)
        ros.Time.now.side_effect = lambda: NS(to_sec=lambda: self.now)
        modules = {'rospy': ros, 'geometry_msgs.msg': NS(PointStamped=Message, PoseStamped=Message),
                   'morai_msgs.msg': NS(CtrlCmd=Message), 'nav_msgs.msg': NS(Odometry=Message, Path=Message),
                   'std_msgs.msg': NS(Bool=Message, Float64=Message, String=Message)}
        ros.Publisher.side_effect = lambda *a, **kw: Mock()
        spec = importlib.util.spec_from_file_location('adaptive_test_node', PACKAGE / 'scripts/adaptive_curvature_purepursuit_node.py')
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(self.module)
        self.module.time = NS(monotonic=lambda: self.now)
        self.module.load_path_file = lambda _: [PathPoint(float(x), 0.) for x in range(101)]
        self.node = self.module.AdaptiveCurvaturePurePursuit()

    def path(self, start=0., y=0.):
        path = Message()
        for x in range(21):
            p = Message()
            p.pose.position.x, p.pose.position.y = start + x, y
            path.poses.append(p)
        return path

    def tick(self, stop=False, merge=False, speed_limit=30. / 3.6, x=0.):
        self.now += .05
        self.node._active_path_cb(self.path(start=x))
        self.node._stop_cb(Message(stop))
        self.node._merge_stop_cb(Message(merge))
        self.node._target_speed_cb(Message(speed_limit))
        self.node._odom_cb(NS(pose=NS(pose=NS(position=NS(x=x, y=0.), orientation=NS(x=0., y=0., z=0., w=1.))),
                              twist=NS(twist=NS(linear=NS(x=0., y=0.)))))
        self.node._control_cb(None)
        return self.node.command_pub.publish.call_args.args[0]

    def test_rolling_path_start_and_horizon_do_not_limit_speed_to_zero_or_two_mps(self):
        for _ in range(80):
            command = self.tick()
        self.assertGreater(command.accel, 0.)
        self.assertEqual(command.brake, 0.)
        self.assertGreater(self.node.command_speed_mps, 2.)
        self.assertAlmostEqual(self.node.active_speed_profile[-1], 30. / 3.6)

    def test_global_route_remains_immutable_after_lateral_path_update(self):
        self.node._active_path_cb(self.path(y=3.5))
        self.node.global_reference_pub.publish.assert_called_once()
        global_path = self.node.global_reference_pub.publish.call_args.args[0]
        active_path = self.node.reference_pub.publish.call_args.args[0]
        self.assertEqual([p.pose.position.y for p in global_path.poses], [0.] * 101)
        self.assertEqual([p.pose.position.y for p in active_path.poses], [3.5] * 21)

    def test_waiting_for_gap_resets_departure_ramp_and_resumes_gradually(self):
        for _ in range(80):
            command = self.tick(merge=True)
        self.assertEqual((command.accel, command.brake, self.node.command_speed_mps), (0., 1., 0.))
        self.tick()
        self.assertGreater(self.node.command_speed_mps, 0.)
        self.assertLessEqual(self.node.command_speed_mps, .051)

    def test_target_speed_limit_and_real_route_goal_are_enforced(self):
        for _ in range(60):
            self.tick(speed_limit=1.)
        self.assertAlmostEqual(self.node.command_speed_mps, 1.)
        command = self.tick(x=100.)
        self.assertEqual((command.accel, command.brake), (0., 1.))

    def test_each_managed_input_must_be_fresh(self):
        for attribute, reason in [('active_path_wall_time', 'active_path_stale'),
                                  ('stop_status_wall_time', 'stop_status_stale'),
                                  ('target_speed_override_wall_time', 'target_speed_override_stale'),
                                  ('merge_status_wall_time', 'merge_gate_stale')]:
            with self.subTest(attribute=attribute):
                self.tick()
                setattr(self.node, attribute, self.now - 2.)
                self.assertEqual(self.node._managed_fault(self.now), reason)
                self.node._control_cb(None)
                command = self.node.command_pub.publish.call_args.args[0]
                self.assertEqual((command.accel, command.brake), (0., 1.))


if __name__ == '__main__':
    unittest.main()
