import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import unittest


class SafetyMessage:
    def __init__(self): self.header=NS(stamp=None,frame_id='')


class ManagedSafetyTest(unittest.TestCase):
    def setUp(self):
        source=Path(__file__).resolve().parents[3]/'detection/morai_sensor_fusion/scripts/roi_sensor_safety_adapter.py'
        spec=importlib.util.spec_from_file_location('safety_adapter_test',source)
        self.m=importlib.util.module_from_spec(spec)
        modules={'rospy':Mock(),'lidar_perception.msg':NS(LidarObstacleArray=NS),
                 'morai_perception_msgs.msg':NS(SafetyStop=SafetyMessage),'nav_msgs.msg':NS(Odometry=NS),
                 'std_msgs.msg':NS(Bool=NS)}
        with patch.dict(sys.modules,modules): spec.loader.exec_module(self.m)
        self.m.time=NS(monotonic=lambda:100.0)
        self.n=self.m.RoiSensorSafetyAdapter.__new__(self.m.RoiSensorSafetyAdapter)
        n=self.n
        n.latest_lidar=n.latest_odom=NS()
        n.last_lidar_at=n.last_odom_at=100.0
        n.input_timeout_sec=0.5
        n.require_source_stamps=n.require_fresh_lidar=True
        n.require_fresh_camera_stops=False
        n.source_fresh=Mock(return_value=True)
        n.nearest_forward_obstacle=Mock(return_value=0.0)
        n.camera_stops=n.camera_updated={}
        n.managed_stop_topic='/highway_lane_strategy/stop_required'
        n.managed_at=100.0; n.managed_stop=False
        n.publisher=Mock()

    def output(self):
        self.n.publish_safety(None)
        return self.n.publisher.publish.call_args.args[0]

    def test_approved_bypass_uses_trajectory_guard_instead_of_old_front_rectangle(self):
        self.assertFalse(self.output().stop_required)
        self.n.nearest_forward_obstacle.assert_not_called()

    def test_missing_trajectory_heartbeat_stops(self):
        self.n.managed_at=98.0
        self.assertTrue(self.output().stop_required)
        self.assertEqual(self.output().reason,'managed_trajectory_stale')

    def test_clear_trajectory_never_overrides_lidar_loss(self):
        self.n.last_lidar_at=98.0
        self.assertTrue(self.output().stop_required)
        self.assertEqual(self.output().reason,'roi_lidar_stale')


if __name__=='__main__': unittest.main()
