import json
import math
from pathlib import Path
import unittest
from purepursuit_mgeo.roundabout_contract import RoundaboutScope, MissionSpeedGate
from purepursuit_mgeo.yujin_roundabout.core import Decision, Ego, Track


WS=Path(__file__).resolve().parents[4]


def scope():
    return RoundaboutScope(WS/'config/roundabout_v61.yaml',
        WS/'data/routes/2026_molit_comp_global_path.txt',
        WS/'data/routes/roundabout_circulating_path.txt')


class RoundaboutGateTest(unittest.TestCase):
    def setUp(self):
        self.scope=scope();self.gate=MissionSpeedGate(self.scope)

    def state(self, **kwargs):
        state=dict(timestamp=100.,target_speed=2.8,active=True,stop_required=False,
                   emergency=False,decision_reason='clear',main_path_sha256=self.scope.path_sha)
        state.update(kwargs)
        return json.dumps(state)

    def test_only_selected_mgeo_roundabout_can_cap_the_course(self):
        for progress in (0.,self.scope.start-.01,self.scope.end+1):
            self.assertEqual(self.gate.limit(10.,progress,100.,100.)[:2],(10.,False))
        self.gate.observe(self.state(),100.,100.)
        self.assertEqual(self.gate.limit(10.,self.scope.start+1,100.,100.)[:2],(2.8,False))

    def test_stale_missing_or_wrong_path_decision_stops_inside_only(self):
        self.assertTrue(self.gate.limit(10.,self.scope.start+1,100.,100.)[1])
        self.gate.observe(self.state(),100.,100.)
        self.assertTrue(self.gate.limit(10.,self.scope.start+1,100.4,100.4)[1])
        self.gate.observe(self.state(main_path_sha256='other'),100.,100.)
        self.assertIsNone(self.gate.sample)

    def test_empty_tracks_allow_entry_after_original_mandatory_stop(self):
        engine=Decision(self.scope.path,self.scope.circulating,self.scope.config)
        i=self.scope.config['entry_decision_stop_index']
        p,q=self.scope.path[i:i+2]
        ego=Ego(p.x,p.y,math.atan2(q.y-p.y,q.x-p.x),0.)
        for j in range(30):
            t=100+j*.05;engine.update_tracks([],t)
            result=engine.step(ego,t)
            if j==0:self.assertTrue(result['stop_required'])
        self.assertFalse(result['stop_required'],result)
        self.assertTrue(result['allowed'])

    def test_actual_vehicle_overlap_blocks_even_after_entry_permission(self):
        engine=Decision(self.scope.path,self.scope.circulating,self.scope.config)
        i=self.scope.config['entry_decision_stop_index'];p,q=self.scope.path[i:i+2]
        yaw=math.atan2(q.y-p.y,q.x-p.x);ego=Ego(p.x,p.y,yaw,0.)
        engine.update_tracks([Track(42,p.x+2*math.cos(yaw),p.y+2*math.sin(yaw),yaw,4.5,1.8,0.,0.)],100.)
        result=engine.step(ego,100.)
        self.assertTrue(result['emergency'])
        self.assertTrue(result['stop_required'])

    def test_lidar_dropout_is_not_an_empty_clear_frame(self):
        engine=Decision(self.scope.path,self.scope.circulating,self.scope.config)
        p=self.scope.path[self.scope.config['mission_indices']['approach']]
        result=engine.step(Ego(p.x,p.y,0.,0.),100.,obstacles_fresh=False)
        self.assertEqual(result['decision_reason'],'obstacles_stale_or_invalid')
