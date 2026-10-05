"""Real speed planning/PI callbacks with only ROS messages and time replaced."""
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "src"))
sys.path.insert(0, str(PACKAGE.parents[1] / "control/purepursuit_mgeo/src"))
from curvature_speed_purepursuit.planner import PathPoint


class Message:
    def __init__(self, data=None):
        self.data = data
        self.header = NS(stamp=None, frame_id="")
        self.poses = []
        self.pose = NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))
        self.point = NS(x=0., y=0., z=0.)
        self.longlCmdType = 1
        self.accel = self.brake = self.steering = self.velocity = self.acceleration = 0.


class StartupTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.params = {"~max_speed_kph": 30., "~initial_speed_kph": 0., "~publish_command": True}
        self.points = [PathPoint(float(x), 0.) for x in range(101)]
        ros = Mock()
        ros.get_param.side_effect = lambda key, default=None: self.params.get(key, default)
        ros.Time.now.side_effect = lambda: NS(to_sec=lambda: self.now)
        ros.Publisher.side_effect = lambda *args, **kwargs: Mock()
        modules = {"rospy": ros,
                   "geometry_msgs.msg": NS(PointStamped=Message, PoseStamped=Message),
                   "morai_msgs.msg": NS(CtrlCmd=Message),
                   "nav_msgs.msg": NS(Odometry=Message, Path=Message),
                   "std_msgs.msg": NS(Bool=Message, Float64=Message)}
        spec = importlib.util.spec_from_file_location("startup_node", PACKAGE / "scripts/curvature_speed_purepursuit_node.py")
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(self.module)
        self.module.time = NS(monotonic=lambda: self.now)
        self.module.load_path_file = lambda _: self.points
        self.node = self.module.CurvatureSpeedPurePursuitNode()

    def tick(self, x=0., y=0., speed=0.):
        self.now += .05
        message = NS(pose=NS(pose=NS(position=NS(x=x, y=y),
                                    orientation=NS(x=0., y=0., z=0., w=1.))),
                     twist=NS(twist=NS(linear=NS(x=speed, y=0.))))
        self.node.odom_callback(message)
        self.node.control_callback(None)
        return self.node.command_pub.publish.call_args.args[0]

    def test_vehicle_at_exact_start_can_request_acceleration_from_rest(self):
        for _ in range(20):
            command = self.tick()
        self.assertEqual(self.node.last_progress_s, 0.)
        self.assertGreater(command.accel, 0.)
        self.assertEqual(command.brake, 0.)
        self.assertGreater(self.node.speed_command_pub.publish.call_args.args[0].data, 0.)

    def test_startup_still_ramps_speed_from_zero_with_time_acceleration_limit(self):
        self.assertEqual(self.node.command_speed_mps, 0.)
        previous = 0.
        for _ in range(20):
            self.tick()
            current = self.node.command_speed_mps
            self.assertGreater(current, previous)
            self.assertLessEqual(current - previous, self.node.max_accel_mps2 * .05 + 1e-9)
            self.assertLessEqual(current * 3.6, 30.)
            previous = current

    def test_curved_start_retains_curvature_speed_ceiling(self):
        self.points = [PathPoint(5. * math.sin(i * .02), 5. * (1. - math.cos(i * .02)))
                       for i in range(151)]
        self.node = self.module.CurvatureSpeedPurePursuitNode()
        for _ in range(120):
            self.tick()
            self.assertLessEqual(self.node.command_speed_mps, self.node.speed_profile[0] + 1e-9)
        self.assertGreater(self.node.command_speed_mps, 0.)
        self.assertLess(self.node.command_speed_mps * 3.6, 30.)

    def test_positive_initial_speed_remains_an_explicit_spatial_start_cap(self):
        self.params["~initial_speed_kph"] = 1.8
        self.node = self.module.CurvatureSpeedPurePursuitNode()
        for _ in range(40):
            self.tick()
        self.assertAlmostEqual(self.node.speed_profile[0], .5)
        self.assertAlmostEqual(self.node.command_speed_mps, .5)

    def test_zero_max_speed_cannot_be_overridden_by_startup(self):
        self.params["~max_speed_kph"] = 0.
        self.node = self.module.CurvatureSpeedPurePursuitNode()
        for _ in range(20):
            command = self.tick()
        self.assertEqual(command.accel, 0.)
        self.assertEqual(self.node.command_speed_mps, 0.)

    def test_goal_still_stops_and_brakes(self):
        self.tick()
        command = self.tick(x=100.)
        self.assertEqual((command.accel, command.brake), (0., 1.))
        self.assertTrue(self.node.goal_pub.publish.call_args.args[0].data)
        self.assertEqual(self.node.speed_profile[-1], 0.)

    def test_missing_or_stale_odometry_does_not_accelerate(self):
        self.node.control_callback(None)
        self.node.command_pub.publish.assert_not_called()
        self.tick()
        self.now += 1.
        self.node.control_callback(None)
        command = self.node.command_pub.publish.call_args.args[0]
        self.assertEqual((command.accel, command.brake), (0., 1.))
        self.assertEqual(self.node.command_speed_mps, 0.)

    def test_invalid_initial_speed_cannot_remove_the_start_cap(self):
        for value in (-1., float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.params["~initial_speed_kph"] = value
                self.module.CurvatureSpeedPurePursuitNode()
