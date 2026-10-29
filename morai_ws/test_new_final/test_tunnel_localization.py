"""Real EKF callbacks with GPS blackout, allowed Competition speed and IMU."""
import math
import unittest
import xml.etree.ElementTree as ET
from types import SimpleNamespace as NS
from unittest.mock import patch, Mock
import test_migration as migration


class TunnelLocalizationTest(unittest.TestCase):
    def setUp(self):
        fixture = migration.LocalizationSourceTests()
        fixture.setUp()
        self.module, self.ros = fixture.module, fixture.ros
        self.now = 100.
        self.ros.get_time.side_effect = lambda: self.now
        self.ros.get_param.side_effect = lambda key, default=None: (
            '/Ego_topic' if key == '~competition_speed_topic' else default)
        self.module.time = NS(monotonic=lambda: self.now)
        deps = {'morai_msgs.msg': NS(EgoVehicleStatus=object),
                'std_msgs.msg': NS(String=object)}
        with patch.dict('sys.modules', deps):
            self.node = self.module.LocalEnuEkf()
        self.node.publish = Mock()

    def speed(self, value=10.):
        self.node.speed_callback(NS(header=NS(stamp=NS(to_sec=lambda: self.now)),
                                    velocity=NS(x=value, y=float('nan'), z=float('nan'))))

    def imu(self, yaw=0., accel=4.):
        self.node.imu_callback(NS(header=NS(stamp=NS(to_sec=lambda: self.now)),
            orientation=NS(x=0., y=0., z=math.sin(yaw/2), w=math.cos(yaw/2)),
            angular_velocity=NS(z=0.), linear_acceleration=NS(x=accel)))

    def gps(self, x=123., y=456.):
        self.node.gps_callback(NS(header=NS(stamp=NS(to_sec=lambda: self.now)),
            pose=NS(pose=NS(position=NS(x=x, y=y, z=0.)), covariance=[1.]*36),
            twist=NS(twist=NS(linear=NS(x=0., y=0.)))))

    def initialize(self, yaw=0.):
        self.speed()
        self.imu(yaw)
        self.gps()

    def test_no_gps_for_ten_seconds_tracks_measured_speed_without_accel_drift(self):
        self.initialize()
        expected = 123.
        for i in range(500):
            self.now += .02
            v = 5. if i < 250 else 10.
            self.speed(v)
            self.imu(accel=4.)
            expected += v*.02
        self.assertAlmostEqual(self.node.state[0, 0], expected, places=6)
        self.assertAlmostEqual(self.node.state[1, 0], 456., places=6)
        self.assertEqual(self.node.state[3, 0], 10.)

    def test_imu_mount_heading_rotates_distance_and_signed_speed(self):
        self.initialize(math.pi/2)
        for _ in range(100):
            self.now += .02
            self.speed(-2.)
            self.imu(math.pi/2)
        self.assertAlmostEqual(self.node.state[0, 0], 123., places=5)
        self.assertAlmostEqual(self.node.state[1, 0], 452., places=5)

    def test_frozen_valid_gps_cannot_pin_vehicle_inside_tunnel(self):
        self.initialize()
        for _ in range(200):
            self.now += .02
            self.speed()
            self.imu()
            self.gps()
        self.assertTrue(self.node.gps_frozen)
        self.assertGreater(self.node.state[0, 0], 150.)
        self.gps(x=163.)
        self.assertFalse(self.node.gps_frozen)
        self.assertAlmostEqual(self.node.last_accepted_gps_stamp, self.now)

    def test_speed_dropout_or_imu_dropout_does_not_freshen_pose(self):
        self.initialize()
        self.node.publish.reset_mock()
        self.now += .6
        self.imu()
        self.now += .02
        self.imu()
        self.node.publish.assert_not_called()
        self.speed()
        self.gps(x=130.)
        self.node.publish.assert_not_called()

    def test_without_initial_fix_speed_and_imu_do_not_invent_map_position(self):
        self.speed()
        self.imu()
        self.assertFalse(self.node.initialized)
        self.node.publish.assert_not_called()

    def test_nonfinite_fix_is_rejected_before_initialization(self):
        self.gps(x=float('nan'))
        self.assertFalse(self.node.initialized)

    def test_profile_two_routes_competition_speed_into_actual_ekf_include(self):
        launch = migration.WS/'src/bringup/morai_bringup/launch'
        profile = ET.parse(launch/'final_ws_curvature_signal.launch').getroot()
        self.assertEqual(profile.find("include/arg[@name='ekf_competition_speed_topic']").get('value'),
                         '$(arg vehicle_speed_topic)')
        base = ET.parse(launch/'morai_udp_ekf_purepursuit.launch').getroot()
        ekf = next(i for i in base.findall('include') if 'ekf_local_enu.launch' in i.get('file'))
        self.assertEqual(ekf.find("arg[@name='competition_speed_topic']").get('value'),
                         '$(arg ekf_competition_speed_topic)')

    def test_gps_blackout_prediction_keeps_existing_main_controller_receiving_fresh_pose(self):
        self.speed(5.);self.imu();self.gps(x=0.,y=0.)
        controller=migration.control_fixture.StartupTest();controller.setUp()
        for i in range(500):
            self.now+=.02
            self.speed(5.);self.imu(accel=4.)
            controller.now=self.now-.05
            command=controller.tick(x=float(self.node.state[0,0]),
                                    y=float(self.node.state[1,0]),speed=5.)
            if i>450:
                self.assertEqual(command.brake,0.)
                self.assertGreater(command.accel,0.)
        self.assertAlmostEqual(self.node.state[0,0],50.,places=5)

    def test_no_tunnel_camera_priority_or_new_camera_fallback_is_added(self):
        profile=ET.parse(migration.WS/'src/bringup/morai_bringup/launch/final_ws_curvature_signal.launch').getroot()
        self.assertIsNone(profile.find("arg[@name='enable_tunnel_lane_priority']"))
        self.assertIsNone(profile.find("node[@type='camera_localization_fallback_controller.py']"))
