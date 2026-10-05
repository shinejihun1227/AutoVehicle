import unittest
from purepursuit_mgeo.path import PathPoint
from purepursuit_mgeo.frenet_path import ReferencePath
from purepursuit_mgeo.frenet_sampling_planner import generate_frenet_bypass_candidates
from purepursuit_mgeo.trajectory_safety import ObstacleBox, evaluate_candidate


class FrenetIntegrationTests(unittest.TestCase):
    def candidates(self):
        original=[PathPoint(float(x),0.0,0.0) for x in range(101)]
        reference=ReferencePath(original)
        candidates=generate_frenet_bypass_candidates(reference,0.0,0.0,0.0,1,35.0,0.0,
            1.0,0.4,1.892,0.85,4.0,22.0,22.0,[0.0,0.4],3.8,90.0)
        self.assertTrue(all(p.y==0.0 for p in original))
        return candidates

    def assess(self,candidate,boxes):
        return evaluate_candidate(candidate_index=0,candidate=candidate,obstacles=boxes,
            vehicle_length_m=4.635,vehicle_width_m=1.892,vehicle_center_from_base_m=1.5,
            collision_longitudinal_margin_m=0.4,collision_lateral_margin_m=0.45,
            wheelbase_m=3.0,max_steering_rad=0.6981317008,evaluation_speed_mps=2.0,
            max_lateral_accel_mps2=1.0,collision_sample_stride=1,escape_prefix_m=0.0)

    def test_detour_exists_without_modifying_original_route(self):
        candidates=self.candidates()
        self.assertEqual({c.side for c in candidates},{'left','right'})
        self.assertTrue(any(self.assess(c,[ObstacleBox(1,35.0,0.0,0.0,2.0,0.8)]).valid for c in candidates))
        for c in candidates:
            self.assertAlmostEqual(c.path[0].y,0.0)
            self.assertAlmostEqual(c.path[-1].y,0.0)

    def test_blocked_both_sides_does_not_produce_safe_candidate(self):
        boxes=[ObstacleBox(1,35.0,0.0,0.0,8.0,12.0)]
        self.assertFalse(any(self.assess(c,boxes).valid for c in self.candidates()))


if __name__=='__main__': unittest.main()
