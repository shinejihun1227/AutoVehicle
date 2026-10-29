import math
import unittest
from curvature_speed_purepursuit.tunnel_lane import TunnelLaneFollower
import test_node_startup as fixture
from types import SimpleNamespace as NS
import json


class TunnelLaneTest(unittest.TestCase):
    def setUp(self):
        self.follower = TunnelLaneFollower()

    def test_lane_curvature_steers_when_dead_reckoned_route_is_straight(self):
        for i in range(20):
            t = 100+i*.1
            self.follower.estimator(dict(mode='IMU_COMPETITION_DEAD_RECKONING'), t, t)
            self.follower.observe((-.7, .15), t, t)
            result = self.follower.update(0., 5., 3., 1., t, t, .1)
        self.assertTrue(result['lane_valid'])
        self.assertEqual(result['camera_weight'], 1.)
        self.assertGreater(result['steering'], .08)

    def test_straight_offset_converges_in_kinematic_closed_loop(self):
        y, yaw, speed, dt = .6, 0., 5., .05
        for i in range(240):
            t = 100+i*dt
            y7 = -(y+math.sin(yaw)*7)/math.cos(yaw)
            y14 = -(y+math.sin(yaw)*14)/math.cos(yaw)
            self.follower.estimator(dict(mode='IMU_COMPETITION_DEAD_RECKONING'), t, t)
            self.follower.observe((-y7, math.atan2(y14-y7,7)), t, t)
            result = self.follower.update(0., speed, 3., 1., t, t, dt)
            yaw += speed/3.*math.tan(result['steering'])*dt
            y += speed*math.sin(yaw)*dt
        self.assertLess(abs(y), .1)
        self.assertLess(abs(yaw), .02)

    def test_camera_loss_slows_then_stops_and_duplicate_frames_do_not_acquire(self):
        for i in range(30):
            t=100+i*.1
            self.follower.estimator(dict(mode='IMU_COMPETITION_DEAD_RECKONING'),t,t)
            self.follower.observe((0.,0.),100.,100.)
            result=self.follower.update(.1,5.,3.,1.,t,t,.1)
        self.assertTrue(result['stop'])
        self.assertFalse(result['lane_valid'])

    def test_estimator_dropout_cannot_restore_straight_map_priority_in_blackout(self):
        self.follower.estimator(dict(mode='IMU_COMPETITION_DEAD_RECKONING'),100.,100.)
        for i in range(40):
            t=100+i*.1
            self.follower.estimator(None,t,t)
            self.follower.observe((-.7,.1),t,t)
            result=self.follower.update(0.,5.,3.,1.,t,t,.1)
        self.assertTrue(result['blackout'])
        self.assertTrue(result['stop'])
        self.assertEqual(result['reason'],'tunnel_estimator_stale_stop')

    def test_curved_tunnel_follows_observed_lane_with_zero_map_steering(self):
        x,y,yaw,radius,speed,dt=0.,0.,0.,40.,5.,.05
        for i in range(240):
            t=100+i*dt
            cx=-x*math.cos(yaw)+(radius-y)*math.sin(yaw)
            cy=x*math.sin(yaw)+(radius-y)*math.cos(yaw)
            y7=cy-math.sqrt(radius**2-(7-cx)**2)
            y14=cy-math.sqrt(radius**2-(14-cx)**2)
            self.follower.estimator(dict(mode='IMU_COMPETITION_DEAD_RECKONING'),t,t)
            self.follower.observe((-y7,math.atan2(y14-y7,7)),t,t)
            result=self.follower.update(0.,speed,3.,1.,t,t,dt)
            yaw+=speed/3.*math.tan(result['steering'])*dt
            x+=speed*math.cos(yaw)*dt
            y+=speed*math.sin(yaw)*dt
        self.assertLess(abs(math.hypot(x,y-radius)-radius),.5)
        self.assertGreater(yaw,1.)

    def test_recovered_gps_fades_camera_priority_and_retains_speed_of_normal_route(self):
        for i in range(20):
            t=100+i*.1
            self.follower.estimator(dict(mode='IMU_COMPETITION_DEAD_RECKONING'),t,t)
            self.follower.observe((-.7,.1),t,t)
            self.follower.update(0.,5.,3.,1.,t,t,.1)
        for i in range(20):
            t=102+i*.1
            self.follower.estimator(dict(mode='GPS_IMU_COMPETITION'),t,t)
            result=self.follower.update(-.1,5.,3.,1.,t,t,.1)
        self.assertAlmostEqual(result['steering'], -.1)
        self.assertIsNone(result['speed_cap_mps'])
        self.assertFalse(result['stop'])


class MainNodeTunnelTest(unittest.TestCase):
    def setUp(self):
        self.case=fixture.StartupTest();self.case.setUp()
        self.case.params['~enable_tunnel_lane_priority']=True
        self.case.node=self.case.module.CurvatureSpeedPurePursuitNode()

    def test_current_main_controller_uses_camera_turn_without_gps_route_turn(self):
        case=self.case
        for i in range(30):
            t=case.now
            case.node.estimator_callback(NS(data=json.dumps(dict(mode='IMU_COMPETITION_DEAD_RECKONING'))))
            case.node.tunnel_lane.observe((-.7,.15),t,t)
            command=case.tick(speed=5.)
        self.assertGreater(command.steering,.08)
        self.assertLessEqual(case.node.command_speed_mps,25./3.6)

    def test_camera_loss_stop_is_not_a_route_goal(self):
        case=self.case
        for i in range(30):
            case.node.estimator_callback(NS(data=json.dumps(dict(mode='IMU_COMPETITION_DEAD_RECKONING'))))
            command=case.tick(speed=5.)
        self.assertEqual((command.accel,command.brake),(0.,1.))
        self.assertFalse(case.node.goal_pub.publish.call_args.args[0].data)
