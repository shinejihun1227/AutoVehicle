"""Observed short lanes, curve compensation and spatial braking regressions."""
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

WS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(WS/'src/experimental/curvature_speed_purepursuit/src'))
from curvature_speed_purepursuit.lane_centering import (
    LaneCenteringAssist, lane_window, lane_quality, lane_measurement, ideal_lane_reference)
from curvature_speed_purepursuit.planner import (
    PathPoint, cumulative_arc_lengths, curvature_speed_mps, build_speed_profile)


def lane(near,far,offset=0.,width=3.3):
    info=dict(lane_valid=True,frame_id='base_link',center_source='both',
              coordinate_convention=dict(x='forward_m',y='left_m'),lane_width_m=width,
              centerline_points=[[near,offset],[far,offset]])
    for side,sign in (('left',1),('right',-1)):
        info[side+'_lane']=dict(detected=True,from_guide=False,coasted=False,
                               confidence=.9,n_points=50,x_range_m=[near,far])
        info[side+'_boundary_points']=[[near,offset+sign*width/2],[far,offset+sign*width/2]]
    return info


class RealFinalLaneSpeed(unittest.TestCase):
    def test_short_real_boundaries_work_without_extrapolation(self):
        for observed,expected in (((3.,18.),(7.,14.)),((4.,12.),(5.,10.)),((3.,9.),(4.,8.))):
            info=lane(*observed,offset=.25)
            window=lane_window(info)
            self.assertEqual(window,expected)
            self.assertTrue(lane_quality(info,.60,*window)[0])
            self.assertAlmostEqual(lane_measurement(info,*window)[0],-.25)
        info=lane(4.,7.9)
        self.assertFalse(lane_quality(info,.60,*lane_window(info))[0])

    def test_short_span_does_not_accept_guide_coast_or_other_lane(self):
        for key,value in (('from_guide',True),('coasted',True),('confidence',.59)):
            info=lane(4.,10.)
            info['left_lane'][key]=value
            self.assertFalse(lane_quality(info,.60,*lane_window(info))[0])
        for width in (2.2,6.5):
            info=lane(4.,10.,width=width)
            self.assertFalse(lane_quality(info,.60,*lane_window(info))[0])
        info=lane(4.,10.);info['straddling_lane']={'detected':True}
        self.assertFalse(lane_quality(info,.60,*lane_window(info))[0])

    def test_short_curve_compensation_does_not_add_corner_cutting(self):
        points=[PathPoint(20*math.sin(i*.005),20*(1-math.cos(i*.005))) for i in range(301)]
        s=cumulative_arc_lengths(points)
        for near,far in ((7.,14.),(5.,10.),(4.,8.)):
            ref=ideal_lane_reference(points,s,0.,near,far)
            self.assertIsNotNone(ref)
            assist=LaneCenteringAssist(weight=.55,max_correction_rad=.075,min_frames=3)
            heading=math.atan2(ref[1]-ref[0],far-near)
            for i in range(5):
                assist.observe(-ref[0],heading,ref,.9,1.,1+i*.1,1+i*.1,near,far)
            status=assist.update(21/3.6,1.4,1.4,.05)
            self.assertTrue(status['active'],status)
            self.assertAlmostEqual(status['correction_rad'],0.,places=9)

    def test_assist_moves_toward_center_and_fades_on_loss(self):
        assist=LaneCenteringAssist(weight=.55,max_correction_rad=.075,min_frames=3,
                                  filter_tau_sec=.20,correction_rate_rad_s=.14)
        y,yaw,v=.35,0.,21/3.6
        for i in range(400):
            now=1+i*.02
            if i%5==0:
                # A straight observed lane in the moving vehicle's body frame.
                assist.observe(y+5*math.tan(yaw),-yaw,(0.,0.),.95,1.,now,now,5.,10.)
            out=assist.update(v,now,now,.02)
            steering=out['correction_rad']
            self.assertLessEqual(abs(steering),.075)
            yaw+=v/3*math.tan(steering)*.02
            y+=v*math.sin(yaw)*.02
        self.assertLess(abs(y),.15)
        self.assertLess(abs(yaw),.05)
        lost=assist.update(v,now+1,now+1,.02)
        self.assertFalse(lost['active'])

    def test_14_to_21_curve_target_still_brakes_to_zero(self):
        curvature=1/(14/3.6)**2
        self.assertAlmostEqual(curvature_speed_mps(curvature,1.)*3.6,14.)
        self.assertAlmostEqual(curvature_speed_mps(curvature,2.25)*3.6,21.)
        s=list(range(101))
        speeds=build_speed_profile(s,[curvature]*len(s),42/3.6,2.25,1.4,1.2,final_speed_mps=0.)
        self.assertAlmostEqual(speeds[40]*3.6,21.)
        self.assertEqual(speeds[-1],0.)
        for a,b in zip(speeds,speeds[1:]):
            self.assertLessEqual(a*a-b*b,2*1.2+1e-8)

    def test_fusion_and_nominal_share_curve_limit_and_keep_highway(self):
        root=ET.parse(WS/'src/bringup/morai_bringup/launch/final_ws_curvature_signal.launch').getroot()
        nodes={n.get('name'):n for n in root.findall('node')}
        params=[]
        for name in ('curvature_speed_purepursuit','curvature_signal_controller'):
            params.append(next(p.get('value') for p in nodes[name].findall('param') if p.get('name')=='lateral_accel_limit_mps2'))
        self.assertEqual(params,['$(arg curve_lateral_accel_limit_mps2)']*2)
        self.assertIn('highway_command_mux',nodes)
        self.assertIn('roundabout_merge_gate',nodes)

    def test_competition_ip_change_preserves_ports_image_and_clearance(self):
        spec=importlib.util.spec_from_file_location('real_network',WS/'docker/final_ws/configure_real_final_network.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)
            old='IMAGE_NAME=existing:gpu\nUBUNTU_IP=192.168.10.21\nMORAI_IP=192.168.10.14\nCONTAINER_NAME=old\n'
            (p/'highway.env').write_text(old)
            settings,backup=module.configure(p,'competition')
            self.assertEqual(backup.read_text(),old)
            self.assertEqual(settings['UBUNTU_IP'],'192.168.0.10')
            self.assertEqual(settings['MORAI_IP'],'192.168.0.1')
            self.assertIn('IMAGE_NAME=existing:gpu',(p/'highway.env').read_text())
            self.assertFalse((p/'highway-test.env').exists())
