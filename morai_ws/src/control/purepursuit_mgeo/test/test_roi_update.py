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

    def payload(self, stamp, frame='base_link'):
        return {
            'timestamp': stamp,
            'observation_time_source': 'camera_receive_wall',
            'observation_wall_timestamp': stamp,
            'frame_id': frame,
            'output_status': 'FRESH',
            'lane_valid': True,
            'confidence': .95,
            'lane_width_m': 3.5,
            'heading_error_rad': 0.,
            'lateral_error_m': 0.,
            'centerline_points': [[float(x), 0.] for x in range(5, 26)],
        }

    def submit(self, payload):
        with patch.object(self.module.time, 'time', return_value=self.now):
            self.node._lane_info_cb(Message(json.dumps(payload)))

    def test_fresh_camera_observation_is_recorded_with_source_time(self):
        self.submit(self.payload(100.0))
        self.assertEqual(self.node.lane_info_at.to_sec(), 100.0)
        self.assertEqual(self.node.lane_observed_wall_at, 100.0)

    def test_republished_held_geometry_does_not_extend_camera_age(self):
        self.submit(self.payload(100.0))
        self.now = 100.25
        self.submit(self.payload(100.0))
        self.assertEqual(self.node.lane_info_at.to_sec(), 100.25)
        self.assertEqual(self.node.lane_observed_wall_at, 100.0)

        self.now = 100.61
        with patch.object(self.module.time, 'time', return_value=self.now):
            valid, reason = self.node._lane_valid(Stamp(self.now))
        self.assertFalse(valid)
        self.assertEqual(reason, 'lane_observation_stale')

    def test_old_future_reordered_and_wrong_frame_observations_are_rejected(self):
        self.submit(self.payload(100.0))
        original_receipt = self.node.lane_info_at.to_sec()
        self.now = 100.2
        for payload in (self.payload(99.0), self.payload(100.4),
                        self.payload(99.9), self.payload(100.2, frame='camera_link')):
            self.submit(payload)
            self.assertEqual(self.node.lane_info_at.to_sec(), original_receipt)
            self.assertEqual(self.node.lane_observed_wall_at, 100.0)


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

