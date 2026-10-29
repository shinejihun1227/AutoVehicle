#!/usr/bin/env python3
"""V6.1 decision adapter; no CtrlCmd publisher, pedal PID or simulator socket."""
import json
import math
import threading
import time
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from morai_msgs.msg import EgoVehicleStatus
from lidar_perception.msg import LidarObstacleArray
from purepursuit_mgeo.roundabout_contract import RoundaboutScope
from purepursuit_mgeo.yujin_roundabout.core import Decision, Ego, Track


class RoundaboutGateNode:
    def __init__(self):
        rospy.init_node('yujin_roundabout_gate')
        self.scope = RoundaboutScope(rospy.get_param('~config_file'),
            rospy.get_param('~path_file'), rospy.get_param('~circulating_file'))
        for key in ('max_accel', 'max_lateral_accel'):
            limit = float(rospy.get_param('~'+key, self.scope.config[key]))
            if not math.isfinite(limit) or limit <= 0:
                raise ValueError('Invalid main controller limit: '+key)
            self.scope.config[key] = min(self.scope.config[key], limit)
        self.engine = Decision(self.scope.path, self.scope.circulating, self.scope.config)
        self.lock = threading.RLock()
        self.samples = {}
        self.pub = rospy.Publisher('/roundabout_v61/status', String, queue_size=1)
        rospy.Subscriber('/localization/odometry', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param('~vehicle_speed_topic', '/Ego_topic'),
                         EgoVehicleStatus, self.speed_callback, queue_size=1)
        rospy.Subscriber('/perception/lidar/tracked_obstacles_map', LidarObstacleArray,
                         self.tracks_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1./self.scope.config['rate_hz']), self.tick)

    def accept(self, key, message, parser):
        with self.lock:
            try:
                stamp, now = float(message.header.stamp.to_sec()), rospy.get_time()
                previous = self.samples.get(key)
                if (not math.isfinite(stamp) or not -0.05 <= now-stamp <= 0.6
                        or (previous and stamp <= previous[0])):
                    raise ValueError('stale or unordered source')
                value = parser(message)
                self.samples[key] = stamp, time.monotonic(), value
            except (AttributeError, TypeError, ValueError, OverflowError):
                self.samples.pop(key, None)

    def odom_callback(self, message):
        def parse(m):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            if (m.header.frame_id != 'map' or
                    not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w))
                    or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > .01):
                raise ValueError('invalid odometry')
            return p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.accept('odom', message, parse)

    def speed_callback(self, message):
        def parse(m):
            # Competition bridge already converted velocity.x to m/s.
            v = float(m.velocity.x)
            if not math.isfinite(v) or abs(v) > 80.:
                raise ValueError('invalid Competition speed')
            return abs(v)
        self.accept('speed', message, parse)

    def tracks_callback(self, message):
        def parse(m):
            if m.header.frame_id != 'map' or m.obstacle_count != len(m.obstacles):
                raise ValueError('invalid track frame/count')
            tracks, ids = [], set()
            for o in m.obstacles:
                values = (o.center_x_map,o.center_y_map,o.yaw,o.length,o.width,
                          o.velocity_x_map,o.velocity_y_map)
                if (o.id in ids or not all(math.isfinite(v) for v in values)
                        or o.length <= 0 or o.width <= 0):
                    raise ValueError('invalid track')
                ids.add(o.id)
                tracks.append(Track(o.id, *values))
            return tracks
        self.accept('tracks', message, parse)

    def tick(self, _):
        with self.lock:
            now, wall = rospy.get_time(), time.monotonic()
            cfg = self.scope.config
            def value(key, timeout):
                s = self.samples.get(key)
                return s if s and -0.05 <= now-s[0] <= timeout and 0 <= wall-s[1] <= timeout else None
            pose, speed = value('odom', cfg['odom_timeout_s']), value('speed', .5)
            tracks = value('tracks', cfg['obstacle_timeout_s'])
            if tracks:
                self.engine.update_tracks(tracks[2], tracks[0])
            if pose:
                ego = Ego(*pose[2], speed[2] if speed else 0.)
                state = self.engine.step(ego, now, fresh=bool(speed), obstacles_fresh=bool(tracks))
            else:
                state = dict(active=True,target_speed=0.,stop_required=True,emergency=True,
                             decision_reason='roundabout_pose_stale',state='INPUT_FAULT')
            state.update(main_path_sha256=self.scope.path_sha,
                         competition_speed_fresh=bool(speed), obstacle_fresh=bool(tracks), timestamp=now)
            self.pub.publish(String(data=json.dumps(state, allow_nan=False)))


if __name__ == '__main__':
    RoundaboutGateNode()
    rospy.spin()
