import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

from purepursuit_mgeo.route_geometry import RoutePolyline, travel_time


PACKAGE = Path(__file__).resolve().parents[1]


def load_gate_module():
    rospy = ModuleType("rospy")
    modules = {"rospy": rospy}
    message_types = {
        "nav_msgs.msg": {"Odometry": type("Odometry", (), {})},
        "std_msgs.msg": {
            "Bool": type("Bool", (), {}),
            "Float64": type("Float64", (), {}),
            "String": type("String", (), {}),
        },
        "lidar_perception.msg": {"LidarObstacleArray": type("LidarObstacleArray", (), {})},
    }
    for name, values in message_types.items():
        module = ModuleType(name)
        module.__dict__.update(values)
        modules[name] = module
    source = PACKAGE / "scripts" / "roundabout_merge_gate_node.py"
    spec = importlib.util.spec_from_file_location("roundabout_gate_under_test", source)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {**modules, spec.name: module}):
        spec.loader.exec_module(module)
    return module


GATE = load_gate_module()


def odometry(x, y, speed=1.0):
    return NS(
        pose=NS(pose=NS(position=NS(x=x, y=y))),
        twist=NS(twist=NS(linear=NS(x=speed, y=0.0))),
    )


class RoundaboutGateRouteProgressTests(unittest.TestCase):
    def setUp(self):
        self.gate = GATE.RoundaboutMergeGate.__new__(GATE.RoundaboutMergeGate)
        self.gate.route = RoutePolyline([
            (0.0, 0.0), (10.0, 0.0), (10.0, 10.0),
            (0.0, 10.0), (0.0, 0.0), (10.0, 0.0),
            (20.0, 0.0), (30.0, 0.0),
        ])
        self.gate.region = {
            "request_start_s_m": 42.0, "yield_s_m": 45.0,
            "entry_s_m": 48.0, "conflict_s_m": 57.5,
            "request_end_s_m": 65.0, "max_route_offset_m": 2.0,
        }
        self.gate.progress = 45.0
        self.gate.latest_odom = odometry(5.0, 0.0)
        self.gate.ego_span = (55.0, 60.0)
        self.gate.fast_accel_mps2 = 1.0
        self.gate.slow_accel_mps2 = 0.5
        self.gate.entry_speed_mps = 2.0
        self.gate.launch_delay_min_s = 0.0
        self.gate.launch_delay_max_s = 0.4

    def test_uses_global_route_progress_to_disambiguate_repeated_xy(self):
        # The same XY occurs at route s=5 and s=45. The active mission is the
        # second traversal, so it must use the published route progress.
        self.assertAlmostEqual(self.gate._ego_interval()[0],
                               travel_time(10.0, 1.0, 1.0, 2.0))
        self.gate.progress = 5.0
        self.assertIsNone(self.gate._ego_interval())

    def test_rejects_pose_progress_disagreement_and_out_of_range_progress(self):
        self.gate.latest_odom = odometry(5.0, 3.0)
        self.assertIsNone(self.gate._ego_interval())
        self.gate.latest_odom = odometry(5.0, 0.0)
        self.gate.progress = self.gate.route.length + 1.0
        self.assertIsNone(self.gate._ego_interval())

    def test_predicts_full_vehicle_occupancy_interval(self):
        entry, exit_time = self.gate._ego_interval()
        self.assertAlmostEqual(entry, travel_time(10.0, 1.0, 1.0, 2.0))
        self.assertAlmostEqual(exit_time, 0.4 + travel_time(15.0, 1.0, 0.5, 2.0))
        self.assertGreater(exit_time, entry)


if __name__ == "__main__":
    unittest.main()
