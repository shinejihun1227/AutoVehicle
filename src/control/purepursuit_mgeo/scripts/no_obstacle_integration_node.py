#!/usr/bin/env python3
"""Fixed-route supply and exclusive v6.1/nominal command selection."""
import json
import copy
import math
import threading
import time
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, String
from morai_msgs.msg import CtrlCmd
from purepursuit_mgeo.path import load_mgeo_path
from purepursuit_mgeo.command_continuity import continue_command


def choose(now, nominal, decision, driver, timeout=0.3, decision_timeout=None):
    def fresh(sample, limit=timeout):
        return sample is not None and 0 <= now-sample[1] <= limit
    if not fresh(decision, timeout if decision_timeout is None else decision_timeout):
        return None, 'decision_stale'
    if decision[0]['active']:
        if not fresh(driver) or not driver[0].get('driver_input_fresh') or not driver[0].get('active'):
            return None, 'roundabout_driver_stale'
        return driver[0], 'roundabout_v61'
    if not fresh(nominal):
        return None, 'nominal_stale'
    return nominal[0], 'normal_highway'


class Integration:
    def __init__(self):
        self.lock = threading.RLock()
        self.points = load_mgeo_path(rospy.get_param('~path_file'))
        self.odom = self.nominal = self.decision = self.driver = None
        self.command_grace=float(rospy.get_param('~nominal_gap_grace_sec',.2))
        self.mission_timeout=float(rospy.get_param('~mission_status_timeout_sec',.5))
        if not 0 <= self.command_grace <= .3 or not .3 <= self.mission_timeout <= .5:
            raise ValueError('Invalid bounded command continuation window')
        self.last_path_publish=None
        self.s = [0.0]
        for a,b in zip(self.points,self.points[1:]):
            self.s.append(self.s[-1]+math.hypot(a.x-b.x,a.y-b.y))
        self.path = Path(); self.path.header.frame_id = 'map'
        for p in self.points:
            pose=PoseStamped();pose.header.frame_id='map';pose.pose.position.x=p.x;pose.pose.position.y=p.y;pose.pose.orientation.w=1
            self.path.poses.append(pose)
        self.path_pub=rospy.Publisher('/integration/base_path',Path,queue_size=1)
        self.stop_pub=rospy.Publisher('/integration/base_stop',Bool,queue_size=1)
        self.request_pub=rospy.Publisher('/planning/highway_lane_change_request',Bool,queue_size=1)
        self.cmd_pub=rospy.Publisher('/control/ctrl_cmd',CtrlCmd,queue_size=1)
        self.diag=rospy.Publisher('~status',String,queue_size=1)
        rospy.Subscriber('/localization/odometry',Odometry,self.pose,queue_size=1)
        rospy.Subscriber('/integration/nominal_cmd',CtrlCmd,self.command,queue_size=1)
        rospy.Subscriber('/roundabout_merge_gate/status',String,lambda m:self.status(m,'decision'),queue_size=1)
        rospy.Subscriber('/roundabout_test_driver/status',String,lambda m:self.status(m,'driver'),queue_size=1)
        self.timer=rospy.Timer(rospy.Duration(.05),self.tick)
    def pose(self,m):
        if m.header.frame_id!='map':return
        age=rospy.get_time()-m.header.stamp.to_sec()
        if not 0<=age<=.5:return
        with self.lock:self.odom=(m,time.monotonic())
    def command(self,m):
        if m.longlCmdType!=1 or not all(math.isfinite(v) for v in (m.accel,m.brake,m.steering)):return
        with self.lock:self.nominal=(m,time.monotonic())
    def status(self,m,key):
        try:
            d=json.loads(m.data)
            if type(d['active']) is not bool or not 0<=rospy.get_time()-d['timestamp']<=self.mission_timeout:return
            if key=='driver' and not all(math.isfinite(d[k]) for k in ('accel_cmd','brake_cmd','steering_cmd')):return
            with self.lock:
                old=getattr(self,key)
                if old and d['timestamp']<=old[0]['timestamp']:return
                setattr(self,key,(d,time.monotonic()))
        except (ValueError,KeyError,TypeError):return
    def tick(self,_):
        with self.lock:
            now=time.monotonic(); ros_now=rospy.get_time()
            pose_ok=(self.odom is not None and 0<=now-self.odom[1]<=.5
                     and -.05<=ros_now-self.odom[0].header.stamp.to_sec()<=.5)
            highway=False
            if pose_ok:
                p=self.odom[0].pose.pose.position
                i=min(range(len(self.points)),key=lambda i:(self.points[i].x-p.x)**2+(self.points[i].y-p.y)**2)
                highway=1118.742<=self.s[i]<=1741.721 and math.hypot(self.points[i].x-p.x,self.points[i].y-p.y)<=11
            selected,reason=choose(now,self.nominal,self.decision,self.driver,
                                   decision_timeout=self.mission_timeout)
            continued=False
            if selected is None and reason=='nominal_stale' and pose_ok and self.nominal is not None:
                velocity=self.odom[0].twist.twist.linear
                speed=math.hypot(velocity.x,velocity.y)
                selected=continue_command(self.nominal[0],self.nominal[1],now,.3,
                    self.command_grace,healthy=math.isfinite(speed) and speed<=53.2/3.6)
                continued=selected is not None
                if continued:reason='nominal_short_gap_continue'
            cmd=CtrlCmd();cmd.longlCmdType=1;cmd.brake=1
            if pose_ok and selected is not None:
                if isinstance(selected,dict):
                    cmd.accel=selected['accel_cmd'];cmd.brake=selected['brake_cmd'];cmd.steering=selected['steering_cmd']
                else:cmd=selected
            else:reason='odometry_stale' if not pose_ok else reason
            publish_path=self.last_path_publish is None or now-self.last_path_publish>=.1
            if publish_path:self.last_path_publish=now
            path=copy.copy(self.path)
            path.header=copy.copy(self.path.header)
            path.header.stamp=rospy.Time.now()
        # Send control first and serialize the full route outside the input
        # lock. Thousands of poses must not starve command/odometry callbacks.
        self.cmd_pub.publish(cmd)
        self.stop_pub.publish(Bool(data=not pose_ok))
        self.request_pub.publish(Bool(data=highway))
        self.diag.publish(String(data=json.dumps({'source':reason,'continuation_active':continued,
            'highway_request':highway,'accel':cmd.accel,'brake':cmd.brake,'steering':cmd.steering})))
        if publish_path:self.path_pub.publish(path)

if __name__=='__main__':
    rospy.init_node('no_obstacle_integration');Integration();rospy.spin()
