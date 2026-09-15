"""Exercise real callbacks/timers with reordered ROS topic deliveries."""
import importlib.util
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

from test_highway_safety import NODE, NS, Stamp, RosPath, PoseStamped, path_at
from purepursuit_mgeo.path import PathPoint
from purepursuit_mgeo.plan_transport import trajectory_payload


def load_script(name):
    source = Path(__file__).resolve().parents[1] / 'scripts' / (name + '.py')
    spec = importlib.util.spec_from_file_location(name + '_transport_test', source)
    module = importlib.util.module_from_spec(spec)
    modules = {'rospy': NODE.rospy, 'nav_msgs.msg': NS(Odometry=NS, Path=RosPath),
               'geometry_msgs.msg': NS(PoseStamped=PoseStamped),
               'std_msgs.msg': NS(Bool=NS, String=NS),
               'lidar_perception.msg': NS(LidarObstacleArray=NS), spec.name: module}
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(module)
    return module


GUARD = load_script('bypass_lane_guard_node')
MANAGER = load_script('avoidance_path_manager_node')


def plan(seq, safe=True):
    path = path_at(y=2.0)
    path.header.seq = seq
    status = dict(seq=seq, planner_ready=True, avoidance_required=True,
                  safe_path_available=safe, selected_kind='bypass', selected_side='left',
                  path=dict(frame_id='map', stamp=100.0,
                            points=[[p.pose.position.x, p.pose.position.y, 0.0] for p in path.poses]))
    return path, NS(data=json.dumps(status))


class PlanTransportTest(unittest.TestCase):
    def make(self, module, cls):
        with patch.object(module, 'load_mgeo_path', return_value=[PathPoint(float(x), 0., 0.) for x in range(101)]):
            node = cls()
        node.require_atomic_plan_path = True
        return node

    def test_guard_new_path_without_status_does_not_invalidate_complete_plan(self):
        n = self.make(GUARD, GUARD.BypassLaneGuard)
        p, status = plan(1)
        n._base_path_cb(p); n._base_status_cb(status)
        self.assertTrue(n._base_bundle_fresh(Stamp()))
        n._base_path_cb(plan(2)[0])
        self.assertTrue(n._base_bundle_fresh(Stamp()))
        self.assertEqual(n.base_path_seq, 1)

    def test_manager_status_before_legacy_path_is_still_complete(self):
        n = self.make(MANAGER, MANAGER.AvoidancePathManager)
        n._plan_status_cb(plan(2)[1])
        self.assertTrue(n._selected_path_is_fresh(Stamp()))
        self.assertEqual(n.selected_points[0].y, 2.0)
        # A delayed visualization topic must never replace the approved path.
        n._selected_path_cb(plan(1)[0])
        self.assertEqual(n.selected_path_seq, 2)

    def test_first_commit_publishes_only_the_chosen_path(self):
        n = self.make(MANAGER, MANAGER.AvoidancePathManager)
        n.latest_odom = NS(pose=NS(pose=PoseStamped().pose))
        n._sensors_fresh = Mock(return_value=(True, {}))
        n.avoidance_required = n.safe_path_available = True
        n.selected_kind = 'bypass'
        n._commit_selected = Mock(return_value=True)
        detour = [PathPoint(float(x), 2.0, 0.0) for x in range(51)]
        n._remaining_committed = Mock(return_value=(detour, 0, 0.0, 50.0))
        n.active_path_pub = Mock()
        n._timer_cb(None)
        self.assertEqual(n.active_path_pub.publish.call_count, 1)
        self.assertTrue(all(p.pose.position.y == 2.0 for p in n.active_path_pub.publish.call_args.args[0].poses))

    def test_guard_invalid_bundle_and_source_expiry_revoke_ready(self):
        n = self.make(GUARD, GUARD.BypassLaneGuard)
        n._base_status_cb(plan(1)[1])
        self.assertFalse(n._base_bundle_fresh(Stamp(102.0)))
        n._base_status_cb(NS(data='{}'))
        self.assertFalse(n._base_bundle_fresh(Stamp()))
        self.assertFalse(n.base_status['planner_ready'])

    def test_manager_unsafe_decision_does_not_wait_for_path_topic(self):
        n = self.make(MANAGER, MANAGER.AvoidancePathManager)
        n._plan_status_cb(plan(1)[1])
        n._plan_status_cb(plan(2, safe=False)[1])
        self.assertFalse(n.safe_path_available)
        n._plan_status_cb(NS(data='{}'))
        self.assertFalse(n.planner_ready)

    def test_missing_hazard_flag_is_not_treated_as_clear_road(self):
        n = self.make(MANAGER, MANAGER.AvoidancePathManager)
        n._plan_status_cb(plan(1)[1])
        data = json.loads(plan(2)[1].data)
        del data['avoidance_required']
        n._plan_status_cb(NS(data=json.dumps(data)))
        self.assertFalse(n.planner_ready)

    def test_strategy_never_combines_legacy_clear_with_atomic_stop(self):
        with patch.object(NODE, 'load_mgeo_path', return_value=[]):
            n = NODE.HighwayLaneStrategyNode()
        n.base_trajectory_topic = '/avoidance_path_manager/trajectory'
        n._base_trajectory_cb(NS(data=trajectory_payload(path_at(y=2.0), True, 0., 'STOP_NO_SAFE_PATH')))
        n._base_stop_cb(NS(data=False))
        n._base_path_cb(path_at(y=0.0))
        self.assertTrue(n.base_stop)
        self.assertEqual(n.latest_base_path.poses[0].pose.position.y, 2.0)
        self.assertEqual(n.base_reason, 'STOP_NO_SAFE_PATH')

    def test_strategy_does_not_refresh_old_source_path_in_place(self):
        with patch.object(NODE, 'load_mgeo_path', return_value=[]):
            n = NODE.HighwayLaneStrategyNode()
        n.trajectory_pub = Mock()
        p = path_at()
        n._publish(p, False, 2., False, {'reason': 'base_pass'}, Stamp(100.3), 0.1)
        self.assertEqual(p.header.stamp.to_sec(), 100.0)
        payload = json.loads(n.trajectory_pub.publish.call_args.args[0].data)
        self.assertEqual(payload['path']['stamp'], 100.3)
        self.assertFalse(payload['stop_required'])

    def test_integrated_launch_requires_atomic_transport_at_every_boundary(self):
        import xml.etree.ElementTree as ET
        launch = Path(__file__).resolve().parents[4] / 'src/bringup/morai_bringup/launch/final_ws_highway_bringup.launch'
        root = ET.parse(launch).getroot()
        expected = {'bypass_lane_guard': ('require_atomic_plan_path', 'true'),
                    'avoidance_path_manager': ('require_atomic_plan_path', 'true'),
                    'highway_lane_strategy': ('base_trajectory_topic', '/avoidance_path_manager/trajectory'),
                    'adaptive_curvature_purepursuit': ('trajectory_topic', '/highway_lane_strategy/trajectory')}
        for name, (key, value) in expected.items():
            node = root.find("node[@name='%s']" % name)
            self.assertEqual(node.find("param[@name='%s']" % key).get('value'), value)


if __name__ == '__main__': unittest.main()
