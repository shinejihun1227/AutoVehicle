"""Real node callbacks with scoped ROS substitutes; not a ROS/MORAI test."""

import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE / "src"))
from stopline_control.core import Decision


class Stamp:
    def __init__(self, value=0.):
        self.value = value

    def to_sec(self):
        return self.value


class Message:
    def __init__(self, **values):
        self.header = NS(stamp=Stamp(), frame_id="base_link")
        self.__dict__.update(values)


class Command:
    def __init__(self, **values):
        self.longlCmdType = 1
        self.accel = self.brake = self.steering = 0.
        self.velocity = self.acceleration = 0.
        self.__dict__.update(values)


class StopLineWiringTest(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        self.parameters = {}
        self.ros = Mock()
        self.ros.Publisher.side_effect = lambda *_args, **_kwargs: Mock()
        self.ros.get_param.side_effect = lambda key, default=None: self.parameters.get(key, default)
        self.ros.get_time.side_effect = lambda: self.now
        self.ros.Time.now.side_effect = lambda: Stamp(self.now)
        self.modules = {
            "rospy": self.ros,
            "morai_msgs.msg": NS(CtrlCmd=Command),
            "morai_perception_msgs.msg": NS(StopLineDetection=Message, TrafficLight=Message,
                                            SafetyStop=Message, LaneDetection=Message,
                                            SensorQuality=Message),
            "nav_msgs.msg": NS(Odometry=Message),
            "std_msgs.msg": NS(Bool=Message, String=Message),
            "lidar_perception.msg": NS(LidarObstacleArray=Message),
        }
        self.wrapper = self.load(PACKAGE / "scripts/stopline_controller.py")
        self.node = self.wrapper.StopLineController()

    def load(self, path):
        spec = importlib.util.spec_from_file_location("_wiring_" + path.stem, path)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, self.modules):
            spec.loader.exec_module(module)
        module.time = NS(monotonic=lambda: self.now)
        return module

    def odometry(self, speed=2., stamp=None):
        return Message(
            header=NS(stamp=Stamp(self.now if stamp is None else stamp), frame_id="map"),
            twist=NS(twist=NS(linear=NS(x=speed, y=0.))),
            pose=NS(pose=NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.))),
        )

    def approach(self):
        self.node.nominal_callback(Command(accel=0.6, steering=0.2))
        self.node.odom_callback(self.odometry())
        self.node.signal_callback(Message(state="RED", valid=True, confidence=1.,
                                          header=NS(stamp=Stamp(self.now), frame_id="camera_link")))
        self.node.stopline_callback(Message(distance_m=4., valid=True, confidence=1.,
                                            header=NS(stamp=Stamp(self.now), frame_id="base_link")))
        self.node.publish_command(None)
        return self.node.output_pub.publish.call_args.args[0]

    def safety_chain(self, nominal, pedestrian=False, obstacle=False, traffic_topic=""):
        self.parameters = {
            "~traffic_stop_topic": traffic_topic,
            "~pedestrian_stop_topic": "/perception/pedestrian_crossing/stop_required",
            "~intersection_stop_topic": "/perception/intersection/driving_unavailable",
        }
        adapter_module = self.load(SOURCE / "detection/morai_sensor_fusion/scripts/roi_sensor_safety_adapter.py")
        self.ros.Subscriber.reset_mock()
        adapter = adapter_module.RoiSensorSafetyAdapter()
        topics = [call.args[0] for call in self.ros.Subscriber.call_args_list]
        self.assertEqual("/perception/traffic_light/stop_required" in topics, bool(traffic_topic))
        adapter.odom_callback(self.odometry())
        objects = [NS(center_x_map=3., center_y_map=0., width=2.)] if obstacle else []
        adapter.lidar_callback(Message(obstacles=objects))
        adapter._camera_stop_callback(Message(data=pedestrian), "pedestrian")
        if traffic_topic:
            adapter._camera_stop_callback(Message(data=True), "traffic_light")
        adapter.publish_safety(None)
        safety = adapter.publisher.publish.call_args.args[0]
        mux_module = self.load(SOURCE / "control/control_mux/scripts/control_mux_node.py")
        mux = mux_module.ControlMux()
        mux.nominal_callback(nominal)
        mux.localization_callback(self.odometry())
        mux.safety_callback(safety)
        mux.publish_command(None)
        return mux.command_pub.publish.call_args.args[0]

    def test_partial_stopline_brake_reaches_mux_without_raw_red_override(self):
        command = self.approach()
        self.assertGreater(command.brake, 0.)
        self.assertLess(command.brake, 1.)
        output = self.safety_chain(command)
        self.assertEqual((output.accel, output.brake, output.steering), (0., command.brake, 0.2))

    def test_pedestrian_and_lidar_still_override_partial_brake(self):
        for reason in ("pedestrian", "obstacle"):
            with self.subTest(reason=reason):
                output = self.safety_chain(self.approach(), **{reason: True})
                self.assertEqual((output.accel, output.brake), (0., 1.))

    def test_legacy_traffic_stop_remains_when_wrapper_is_disabled(self):
        self.node.enabled = False
        output = self.safety_chain(self.approach(), traffic_topic="/perception/traffic_light/stop_required")
        self.assertEqual(output.brake, 1.)

    def test_overlay_never_weakens_existing_brake_or_changes_steering(self):
        nominal = Command(accel=0.5, brake=0.9, steering=-0.3)
        output = self.wrapper.limit_command(nominal, Decision("APPROACH", "test", 0.2, 0.4))
        self.assertEqual((output.accel, output.brake, output.steering), (0., 0.9, -0.3))
        self.assertEqual(nominal.accel, 0.5)

    def test_fallback_speed_cap_preserves_stopline_brake(self):
        fallback_module = self.load(SOURCE / "experimental/stability_stack/scripts/camera_localization_fallback_controller.py")
        fallback = fallback_module.CameraLocalizationFallbackController()
        for speed in (1., 3.):
            fallback.odom_callback(self.odometry(speed))
            command = self.approach()
            brake = command.brake
            fallback.apply_fallback_speed_cap(command)
            self.assertGreaterEqual(command.brake, brake)
            self.assertEqual(command.accel, 0.)

    def test_stale_nominal_and_wrong_type_fail_safe(self):
        self.approach()
        self.now += 0.6
        self.node.publish_command(None)
        self.assertEqual(self.node.output_pub.publish.call_args.args[0].brake, 1.)
        self.node.nominal_callback(Command(longlCmdType=2, velocity=30.))
        self.node.publish_command(None)
        self.assertEqual(self.node.output_pub.publish.call_args.args[0].brake, 1.)

    def test_duplicate_odometry_does_not_refresh_stale_speed(self):
        self.approach()
        self.now += 0.6
        self.node.nominal_callback(Command(accel=0.5))
        self.node.odom_callback(self.odometry(stamp=10.))
        self.node.publish_command(None)
        self.assertEqual(self.node.output_pub.publish.call_args.args[0].brake, 1.)
        status = json.loads(self.node.status_pub.publish.call_args.args[0].data)
        self.assertIsNone(status["measured_speed_kph"])

    def test_invalid_stopline_frame_and_stamp_are_rejected(self):
        self.node.stopline_callback(Message(distance_m=3., valid=True, confidence=1.,
                                            header=NS(stamp=Stamp(10.), frame_id="front_camera")))
        self.assertIsNone(self.node.core.line)
        self.node.stopline_callback(Message(distance_m=3., valid=True, confidence=1.))
        self.assertIsNone(self.node.core.line)

    def test_bool_false_cannot_release_a_held_stop(self):
        self.node.core.holding = self.node.core.stop_requested = True
        self.node.legacy_callback(Message(data=False))
        self.assertTrue(self.node.core.holding)
        self.assertTrue(self.node.core.stop_requested)

    def test_launch_signal_ownership_and_evaluator_separation(self):
        path = SOURCE / "bringup/morai_bringup/launch"
        root = ET.parse(path / "perception_control_bringup.launch").getroot()
        node = root.find(".//node[@type='roi_sensor_safety_adapter.py']")
        params = node.findall("param[@name='traffic_stop_topic']")
        self.assertEqual(len(params), 2)
        self.assertEqual(params[0].attrib, {"if": "$(arg enable_stopline_control)", "name": "traffic_stop_topic", "value": ""})
        self.assertEqual(params[1].get("unless"), "$(arg enable_stopline_control)")
        self.assertEqual(params[1].get("value"), "$(arg stopline_signal_topic)")
        camera = root.find(".//include[@file='$(find camera_perception)/launch/camera_perception.launch']")
        self.assertEqual(camera.find("arg[@name='traffic_light_state_topic']").get("value"), "$(arg stopline_signal_state_topic)")
        self.assertEqual(camera.find("arg[@name='traffic_light_stop_topic']").get("value"), "$(arg stopline_signal_topic)")
        final = ET.parse(path / "final_ws_bringup.launch").getroot()
        self.assertEqual(final.find("arg[@name='enable_control']").get("default"), "false")
        for document in (root, final):
            self.assertFalse(any("mission_evaluator" in item.get("file", "") for item in document.iter("include")))

    def test_fusion_camera_watchdog_and_rotated_box_bumper_clearance(self):
        self.parameters = {"~require_fresh_camera_stops": True,
                           "~pedestrian_stop_topic": "/perception/pedestrian_crossing/stop_required",
                           "~front_reference_offset_m": 3.845, "~require_source_stamps": True}
        module = self.load(SOURCE / "detection/morai_sensor_fusion/scripts/roi_sensor_safety_adapter.py")
        adapter = module.RoiSensorSafetyAdapter()
        adapter.odom_callback(self.odometry())
        lidar = Message(header=NS(stamp=Stamp(self.now), frame_id="map"), obstacles=[])
        adapter.lidar_callback(lidar)
        adapter.publish_safety(None)
        self.assertEqual(adapter.publisher.publish.call_args.args[0].reason, "camera_pedestrian_stale")
        adapter._camera_stop_callback(Message(data=False), "pedestrian")
        adapter.publish_safety(None)
        self.assertFalse(adapter.publisher.publish.call_args.args[0].stop_required)
        # Center is outside the old 8m range; the face is 6.155m from bumper.
        lidar.obstacles = [NS(center_x_map=12., center_y_map=0., width=2., length=4., yaw=0.)]
        adapter.lidar_callback(lidar)
        adapter.publish_safety(None)
        self.assertAlmostEqual(adapter.publisher.publish.call_args.args[0].distance_m, 6.155)
        self.assertTrue(adapter.publisher.publish.call_args.args[0].stop_required)
        self.now += 0.6
        adapter._camera_stop_callback(Message(data=False), "pedestrian")
        adapter.publish_safety(None)
        self.assertEqual(adapter.publisher.publish.call_args.args[0].reason, "roi_lidar_stale")


if __name__ == "__main__":
    unittest.main()
