"""ROI planning regressions with real path geometry; no ROS master or vehicle."""
import importlib.util
import json
from pathlib import Path
import re
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[2]
sys.path.insert(0, str(PACKAGE / 'src'))
from purepursuit_mgeo.path import PathPoint
from purepursuit_mgeo.frenet_path import ReferencePath


class Stamp:
    def __init__(self, value=0.):
        self.value = value
    def to_sec(self):
        return self.value
    def __sub__(self, other):
        return Stamp(self.value - other.value)


class Message:
    def __init__(self, data=None, **kwargs):
        self.data = data
        self.header = NS(stamp=Stamp(), frame_id='map', seq=1)
        self.pose = NS(position=NS(x=0., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))
        self.poses = []
        self.obstacles = []
        self.__dict__.update(kwargs)


def load_node(name, ros):
    modules = {'rospy': ros, 'geometry_msgs.msg': NS(Point=Message, PoseStamped=Message, Quaternion=Message),
               'nav_msgs.msg': NS(Odometry=Message, Path=Message),
               'std_msgs.msg': NS(Bool=Message, String=Message, Float64=Message, ColorRGBA=Message),
               'visualization_msgs.msg': NS(Marker=Message, MarkerArray=Message),
               'lidar_perception.msg': NS(LidarObstacleArray=Message)}
    spec = importlib.util.spec_from_file_location('_roi_test_' + name, PACKAGE / 'scripts' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {**modules, spec.name: module}):
        spec.loader.exec_module(module)
    return module


class HighwayTest(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        ros = Mock()
        ros.get_param.side_effect = lambda key, default=None: default
        ros.Time.now.side_effect = lambda: Stamp(self.now)
        ros.Time.from_sec.side_effect = Stamp
        ros.Publisher.side_effect = lambda *a, **k: Mock()
        self.module = load_node('highway_lane_strategy_node', ros)
        self.module.load_mgeo_path = lambda _: [PathPoint(float(x), 0., 0.) for x in range(101)]
        self.node = self.module.HighwayLaneStrategyNode()
        self.refresh()

    def refresh(self):
        node = self.node
        node._odom_cb(NS(pose=NS(pose=NS(position=NS(x=0., y=3.5), orientation=NS(x=0., y=0., z=0., w=1.))),
                         twist=NS(twist=NS(linear=NS(x=2., y=0.)))))
        node._obstacles_cb(Message())
        node._base_path_cb(node._local_to_map([(float(x), 0.) for x in range(46)], Stamp(self.now)))
        node._base_stop_cb(Message(False))

    def lane(self, **values):
        data = {'timestamp': self.now, 'frame_id': 'base_link', 'output_status': 'FRESH',
                'lane_valid': True, 'confidence': .95, 'lane_width_m': 3.5,
                'heading_error_rad': 0., 'lateral_error_m': 0.,
                'centerline_points': [[float(x), 0.] for x in range(5, 26)],
                'left_lane': {'detected': True, 'dashed': True}}
        data.update(values)
        self.node._lane_info_cb(Message(json.dumps(data)))

    def test_center_identity_jump_is_slew_limited_and_frozen_during_change(self):
        self.lane()
        initial = dict(self.node.filtered_center_y)
        for _ in range(5):
            previous = dict(self.node.filtered_center_y)
            self.now += .05
            self.lane(centerline_points=[[float(x), 3.5] for x in range(5, 26)])
            self.assertTrue(all(abs(self.node.filtered_center_y[x] - previous[x]) <= .160001 for x in previous))
        self.assertNotEqual(initial, self.node.filtered_center_y)
        self.node.state = self.node.LANE_CHANGE
        previous = dict(self.node.filtered_center_y)
        self.now += .05
        self.lane(centerline_points=[[float(x), -3.5] for x in range(5, 26)])
        self.assertEqual(previous, self.node.filtered_center_y)

    def test_one_sided_lane_can_be_held_but_cannot_start_a_new_change(self):
        self.lane(lane_width_m=None)
        self.assertTrue(self.node._lane_hold_valid(Stamp(self.now))[0])
        self.assertFalse(self.node._lane_valid(Stamp(self.now))[0])

    def test_old_duplicate_future_and_wrong_frame_observations_do_not_renew_age(self):
        self.lane()
        original = self.node.lane_info_at.to_sec()
        for data in ({'timestamp': 99.}, {'timestamp': 100.}, {'timestamp': 102.},
                     {'timestamp': 0.}, {'timestamp': 100.1, 'frame_id': 'camera_link'}):
            self.now = 100.2
            self.lane(**data)
            self.assertEqual(self.node.lane_info_at.to_sec(), original)
        self.assertFalse(self.node._lane_hold_valid(Stamp(101.))[0])
        self.assertFalse(self.node._lane_hold_valid(Stamp(90.))[0])

    def test_occlusion_recovers_then_stops_after_grace_and_resumes_on_fresh_lane(self):
        self.node.state = self.node.INNER_HOLD
        self.lane()
        self.node._tick(None)
        self.assertFalse(self.node.stop_pub.publish.call_args.args[0].data)
        self.now += .7
        self.refresh()
        self.node._tick(None)
        self.assertFalse(self.node.stop_pub.publish.call_args.args[0].data)
        self.now += 3.1
        self.refresh()
        self.node._tick(None)
        self.assertTrue(self.node.stop_pub.publish.call_args.args[0].data)
        self.now += .1
        self.refresh()
        self.lane()
        self.node._tick(None)
        self.assertFalse(self.node.stop_pub.publish.call_args.args[0].data)

    def test_finite_lane_change_hands_over_before_path_endpoint(self):
        node = self.node
        node.state = node.LANE_CHANGE
        node.committed_path = node.latest_base_path
        node.committed_change_length_m = 28.
        node.change_travel_m = 38.
        self.lane()
        node._tick(None)
        self.now += node.change_complete_confirm_s + .1
        self.refresh()
        node._tick(None)
        self.assertEqual(node.state, node.INNER_HOLD)
        self.assertEqual(node.lane_changes_done, 1)

    def test_path_smoothing_limits_lateral_step(self):
        node = self.node
        node.last_inner_path = node._local_to_map([(float(x), 0.) for x in range(46)], Stamp(self.now))
        path = node._smooth_inner_path([(float(x), 2.) for x in range(1, 46)], Stamp(self.now))
        local = node._path_map_to_local(path)
        self.assertTrue(local)
        self.assertTrue(all(abs(y) <= .100001 for x, y in local))


class BypassTest(unittest.TestCase):
    def test_only_slow_obstacles_trigger_bypass_but_all_obstacle_records_are_kept(self):
        module = load_node('avoidance_frenet_debug_node', Mock())
        node = module.AvoidanceFrenetDebugNode.__new__(module.AvoidanceFrenetDebugNode)
        node.reference = ReferencePath([PathPoint(0., 0., 0.), PathPoint(100., 0., 0.)])
        for name, value in {'vehicle_length_m': 4.635, 'vehicle_width_m': 1.892,
                            'vehicle_center_from_base_m': 1.5, 'trigger_lateral_half_cap_m': 1.25,
                            'trigger_lateral_margin_m': .05, 'trigger_distance_m': 35.,
                            'bypass_static_speed_max_mps': .6}.items():
            setattr(node, name, value)
        obstacles = [NS(id=i, center_x_map=15., center_y_map=0., yaw=0., length=4., width=2.,
                        velocity_x_map=speed, velocity_y_map=0.) for i, speed in enumerate((0., .6, .61, 5.))]
        result = node._obstacle_infos(NS(obstacles=obstacles), node.reference.project(0., 0.))
        self.assertEqual(len(result), 4)
        self.assertEqual([x.threatening for x in result], [True, True, False, False])


class LaunchContractTest(unittest.TestCase):
    def test_includes_resolve_and_forward_only_declared_arguments(self):
        packages = {ET.parse(p).getroot().findtext('name'): p.parent for p in (WORKSPACE / 'src').rglob('package.xml')}
        pending = [WORKSPACE / 'src/bringup/morai_bringup/launch/final_ws_highway_bringup.launch',
                   PACKAGE / 'launch/morai_avoidance_highway_roundabout_final.launch',
                   WORKSPACE / 'src/bringup/morai_bringup/launch/final_ws_native_no_lamps.launch']
        seen = set()
        while pending:
            path = pending.pop()
            if path in seen:
                continue
            seen.add(path)
            root = ET.parse(path).getroot()
            declared = {arg.get('name') for arg in root.findall('arg')}
            for element in root.iter():
                for value in element.attrib.values():
                    for name in re.findall(r'\$\(arg ([^)]+)\)', value):
                        self.assertIn(name, declared, (str(path), name))
            for include in root.iter('include'):
                match = re.fullmatch(r'\$\(find ([^)]+)\)(.*)', include.get('file', ''))
                if not match or match[1] not in packages:
                    continue
                target = packages[match[1]] / match[2].lstrip('/')
                self.assertTrue(target.is_file(), str(target))
                target_args = {arg.get('name') for arg in ET.parse(target).getroot().findall('arg')}
                for arg in include.findall('arg'):
                    self.assertIn(arg.get('name'), target_args, (str(path), str(target), arg.get('name')))
                pending.append(target)

    def test_highway_has_one_nominal_controller_and_no_second_lane_udp_reader(self):
        root = ET.parse(WORKSPACE / 'src/bringup/morai_bringup/launch/final_ws_highway_bringup.launch').getroot()
        types = [node.get('type') for node in root.findall('node')]
        self.assertEqual(types.count('adaptive_curvature_purepursuit_node.py'), 1)
        self.assertNotIn('purepursuit_mgeo_node.py', types)
        self.assertNotIn('lane_info_runner.py', types)
        self.assertNotIn('lane_info_semantic_adapter.py', types)
        perception = root.find('include')
        args = {a.get('name'): a.get('value') for a in perception.findall('arg')}
        self.assertEqual(args['enable_nominal_purepursuit'], 'false')
        self.assertEqual(args['enable_roi_camera'], 'true')
        self.assertEqual(args['enable_stopline_control'], 'true')
        declared = {a.get('name'): a.get('default') for a in root.findall('arg')}
        self.assertIn("arg('max_speed_kph')", declared['target_speed_mps'])
        self.assertEqual(declared['enable_control'], 'false')


if __name__ == '__main__':
    unittest.main()
