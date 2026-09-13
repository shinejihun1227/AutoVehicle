import unittest
from unittest.mock import Mock, patch
from test_highway_safety import NODE, Stamp, NS, path_at, obstacle


class HighwayIntegrationTests(unittest.TestCase):
    def setUp(self):
        with patch.object(NODE,'load_mgeo_path',return_value=[]): self.n=NODE.HighwayLaneStrategyNode()
        self.n._odom_pose=Mock(return_value=(0.0,0.0,0.0,2.0))
        self.n.latest_odom=NS()
        self.n.latest_obstacles=NS(obstacles=[])
        self.n.latest_base_path=path_at()

    def test_highway_detection_alone_does_not_change_route(self):
        n=self.n; n.require_mission_request=True; n.highway_environment=True
        self.assertFalse(n._activation_present())
        n._highway_request_cb(NS(data=True)); self.assertTrue(n._activation_present())
        n.request_at=Stamp(98.0); self.assertFalse(n._activation_present())

    def test_repeated_changes_are_bounded(self):
        self.n.lane_changes_done=1
        self.assertEqual(self.n._choose_lane_change(Stamp())[3],'lane_change_count_limit')

    def test_slow_ego_cannot_assume_instant_merge_speed_for_rear_gap(self):
        n=self.n; n._odom_pose.return_value=(0.0,0.0,0.0,0.0)
        rear=NS(oid=2,x=-12.0,length=4.0,vx=4.0)
        n._target_lane_neighbors=Mock(return_value=(None,rear,[rear]))
        safe,reason,_=n._gap_safe_for_speed(4.0,3.5,32.0)
        self.assertFalse(safe)
        self.assertEqual(reason,'rear_gap')

    def test_rear_car_after_commit_remains_in_collision_check(self):
        n=self.n; n.state=n.LANE_CHANGE
        n.latest_obstacles=NS(obstacles=[obstacle(-8.0,3.5,10.0,0.0)])
        p=path_at();
        for ps in p.poses: ps.pose.position.y=min(3.5,ps.pose.position.x*0.5)
        self.assertFalse(n._dynamic_path_safe(p,2.0)[0])

    def test_sparse_vertices_do_not_hide_obstacle(self):
        n=self.n; p=path_at(); p.poses=[p.poses[0],p.poses[25],p.poses[50]]
        n.latest_obstacles=NS(obstacles=[obstacle(12.0)])
        self.assertFalse(n._dynamic_path_safe(p,3.0)[0])

    def test_base_path_guard_distinguishes_obstacle_from_bypass(self):
        n=self.n; n.latest_obstacles=NS(obstacles=[obstacle(15.0)])
        self.assertFalse(n._dynamic_path_safe(path_at(),3.0)[0])
        p=path_at()
        for ps in p.poses: ps.pose.position.y=min(4.0,ps.pose.position.x*0.8)
        self.assertTrue(n._dynamic_path_safe(p,3.0)[0])

    def test_bad_obstacle_frame_and_nan_are_not_clear_space(self):
        n=self.n; n.latest_obstacles=None
        n._obstacles_cb(NS(header=NS(frame_id='base_link',stamp=Stamp()),obstacles=[]))
        self.assertIsNone(n.latest_obstacles)
        n._obstacles_cb(NS(header=NS(frame_id='map',stamp=Stamp()),obstacles=[obstacle(float('nan'))]))
        self.assertIsNone(n.latest_obstacles)


if __name__=='__main__': unittest.main()
