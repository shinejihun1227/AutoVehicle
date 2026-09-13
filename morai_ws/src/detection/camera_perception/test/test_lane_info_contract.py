import unittest
from camera_perception.lane_info_contract import convert, fresh_payload


def sample():
    return dict(timestamp=100.0, frame_id='base_link', sequence=1,
                coordinate_convention={'x':'forward_m','y':'left_m'},
                lane_valid=True, confidence=0.9, lane_width_m=3.5,
                centerline_points=[[x, 0.5+0.1*x] for x in range(20)],
                left_lane=dict(detected=True, type='white_dashed', dashed=True),
                right_lane=dict(detected=True, type='white_solid'),
                stopline_detected=True, stopline_distance_m=12.0,
                stopline=dict(inlier_ratio=0.8, covers_front=True))


class LaneContractTests(unittest.TestCase):
    def test_legacy_lateral_and_heading_references(self):
        _, semantics, lane, line = convert(sample(), 100.1)
        self.assertTrue(lane['valid'])
        self.assertAlmostEqual(lane['lateral_offset_m'], -1.2)
        self.assertAlmostEqual(lane['heading_error_rad'], 0.0996686525)
        self.assertTrue(semantics['dashed'])
        self.assertTrue(line['valid'])

    def test_slow_inference_does_not_refresh_source(self):
        clean, semantics, lane, line = convert(sample(), 101.0)
        self.assertEqual(clean['timestamp'], 100.0)
        self.assertFalse(clean['lane_valid'])
        self.assertFalse(lane['valid'])
        self.assertFalse(line['valid'])
        self.assertFalse(any(semantics.values()))

    def test_guide_coast_or_straddle_does_not_authorize_crossing(self):
        for field in ('from_guide','coasted'):
            payload=sample(); payload['left_lane'][field]=True
            self.assertFalse(convert(payload,100.1)[1]['dashed'])
        payload=sample(); payload['straddling_lane']={'detected':True}
        self.assertFalse(convert(payload,100.1)[2]['valid'])

    def test_extrapolated_stopline_is_not_a_braking_observation(self):
        p=sample(); p['stopline']['covers_front']=False
        self.assertFalse(convert(p,100.1)[3]['valid'])

    def test_stopline_independent_of_lane_and_no_extrapolated_lane(self):
        p=sample(); p['lane_valid']=False; p['confidence']=0.0
        self.assertTrue(convert(p,100.1)[3]['valid'])
        p=sample(); p['centerline_points']=p['centerline_points'][:10]
        self.assertFalse(convert(p,100.1)[2]['valid'])

    def test_frame_clock_and_nonfinite_rejection(self):
        for key,value in (('timestamp',float('nan')),('timestamp',0.0),('timestamp',200.0),('frame_id','map')):
            p=sample(); p[key]=value
            self.assertFalse(fresh_payload(p,100.1))
        p=sample(); p['stopline_distance_m']=float('nan')
        self.assertFalse(convert(p,100.1)[3]['valid'])


if __name__=='__main__': unittest.main()
