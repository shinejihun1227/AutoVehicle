"""Exercise wait-to-planner handoff using real node methods and mocked ROS."""
import json
from types import SimpleNamespace as NS
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
import test_adapter as adapter_fixture
import test_final_dynamic_overlay as final_fixture


class AdapterHandoffTests(unittest.TestCase):
    def setUp(self):
        self.fixture = adapter_fixture.AdapterGateTests()
        self.fixture.setUp()
        self.n = self.fixture.node
        self.n.dynamic_stop_wait = True
        self.n.dynamic_guard_state = None
        self.n.region_gate.regions[1]['name'] = 'dynamic_obstacle'
        self.fixture.pose(35)

    def feedback(self, request=True, timestamp=100., ids=None):
        self.n._dynamic_guard_cb(adapter_fixture.Message(json.dumps(dict(
            dynamic_guard_version='DYNAMIC_WAIT5_TUNNEL_V1', timestamp=timestamp,
            request_static_avoidance=request, waiting_ids=[7] if ids is None else ids))))

    def test_pending_wait_does_not_call_planner(self):
        self.feedback(False)
        self.fixture.tick()
        self.assertEqual(self.n.plan_calls, 0)

    def test_request_calls_real_adapter_parent_planner(self):
        self.feedback()
        self.fixture.tick()
        self.assertEqual(self.n.plan_calls, 1)
        self.assertTrue(self.n._region_status['dynamic_static_requested'])

    def test_request_retained_outside_region(self):
        self.fixture.pose(45)
        self.feedback()
        self.fixture.tick()
        self.assertEqual(self.n.plan_calls, 1)

    def test_request_does_not_override_upstream_stop(self):
        self.feedback()
        self.n.base_stop = True
        self.fixture.tick()
        self.assertEqual(self.n.plan_calls, 0)
        self.assertTrue(self.n._pending_output[1])

    def test_request_does_not_override_mission_owner(self):
        self.feedback()
        self.n.highway = dict(active=True, timestamp=100.)
        self.fixture.tick()
        self.assertEqual(self.n.plan_calls, 0)
        self.assertEqual(self.n._pending_output[3], 'highway')

    def test_stale_request_rejected(self):
        self.feedback(timestamp=99.)
        self.fixture.tick()
        self.assertEqual(self.n.plan_calls, 0)

    def test_empty_wait_ids_cannot_request_planning(self):
        self.feedback(ids=[])
        self.fixture.tick()
        self.assertEqual(self.n.plan_calls, 0)

    def test_tunnel_wait_requests_planner_after_wait(self):
        self.fixture.pose(50)
        self.n.test_blackout = True
        self.feedback(False)
        self.fixture.tick()
        self.assertEqual(self.n.plan_calls, 0)
        self.feedback(timestamp=100.01)
        self.fixture.tick()
        self.assertEqual(self.n.plan_calls, 1)


class FinalHandoffTests(unittest.TestCase):
    def setUp(self):
        self.f = final_fixture.FinalDynamicOverlayTests()
        self.f.setUp()

    def test_real_command_requests_static_after_five_seconds_but_holds_brake(self):
        for _ in range(28):
            output, status = self.f.tick([self.f.obstacle()])
        self.assertTrue(status['dynamic_zone_guard']['request_static_avoidance'])
        self.assertEqual(output.accel, 0.)
        self.assertEqual(output.brake, 1.)
        payload = json.loads(self.f.node.dynamic_guard_pub.publish.call_args.args[0].data)
        self.assertTrue(payload['request_static_avoidance'])
        self.assertEqual(payload['waiting_ids'], [7])

    def test_tunnel_activates_final_guard_outside_dynamic_region(self):
        self.f.node.tunnel_mode_active = lambda *args: True
        obj = self.f.obstacle()
        obj['center_x_map'] = 20.
        output, status = self.f.tick([obj], x=10.)
        self.assertTrue(status['dynamic_zone_guard']['tunnel_dynamic_active'])
        self.assertTrue(status['dynamic_zone_guard']['stop'])
        self.assertEqual(output.brake, 1.)

    def test_executed_detour_must_match_fresh_plan(self):
        n, now = self.f.node, self.f.c.now
        def path(y=0., age=0.):
            return NS(header=NS(frame_id='map', stamp=NS(to_sec=lambda: now-age)),
                      poses=[NS(pose=NS(position=NS(x=x,y=y,z=0.))) for x in (45., 46., 55.)])
        odom = NS(value=NS(pose=NS(pose=NS(position=NS(x=45.,y=0.),
                         orientation=NS(x=0.,y=0.,z=0.,w=1.)))))
        n.event = None
        n.dynamic_handoff_plan_sample = (path(), now)
        n.executed_path_sample = (path(), now)
        self.assertIsNotNone(n.dynamic_detour_path(odom, now, now))
        n.executed_path_sample = (path(y=1.), now)
        self.assertIsNone(n.dynamic_detour_path(odom, now, now))
        n.executed_path_sample = (path(age=2.), now)
        self.assertIsNone(n.dynamic_detour_path(odom, now, now))


class LaunchWiringTests(unittest.TestCase):
    def test_both_nodes_load_region_config(self):
        root = Path(__file__).resolve().parents[2]
        launch = ET.parse(root/'src/bringup/morai_bringup/launch/integrated_obstacle_v61.launch')
        for name in ('curvature_signal_controller','frenet_fsm_controller'):
            node = launch.find("node[@name='%s']" % name)
            self.assertIsNotNone(node.find("rosparam[@file='$(arg avoidance_config_file)']"))
