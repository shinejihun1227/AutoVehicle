"""Camera-priority local path following during the current EKF GPS blackout."""
import math
from collections import deque
from statistics import median


class TunnelLaneFollower:
    def __init__(self, maximum_speed=25./3.6, timeout=.4):
        if not math.isfinite(maximum_speed) or maximum_speed <= 0:
            raise ValueError('Invalid tunnel speed cap')
        self.maximum_speed, self.timeout = maximum_speed, timeout
        self.frames = deque(maxlen=32)
        self.mode = None
        self.blend, self.last_target, self.loss_since = 0., 0., None

    def estimator(self, state, ros_now, wall_now):
        if (not isinstance(state, dict) or state.get('mode') not in
                ('GPS_IMU_COMPETITION', 'IMU_COMPETITION_DEAD_RECKONING')):
            # Invalid/missing status must not masquerade as GPS recovery.
            return
        self.mode = state['mode'], ros_now, wall_now

    def observe(self, measurement, stamp, received):
        # The caller first requires actual, fresh two-sided observed paint.
        if measurement is None:
            self.frames.clear()
            return
        lateral, heading = measurement
        if (not all(math.isfinite(v) for v in (lateral, heading, stamp, received))
                or abs(lateral) > 5. or abs(heading) > .7):
            self.frames.clear()
            return
        if self.frames and (stamp <= self.frames[-1][0] or received <= self.frames[-1][1]):
            return
        if self.frames and stamp-self.frames[-1][0] > self.timeout:
            self.frames.clear()
        if self.frames and (abs(lateral-self.frames[-1][2]) > .8
                            or abs(heading-self.frames[-1][3]) > .15):
            self.frames.clear()
        self.frames.append((stamp, received, lateral, heading))

    def update(self, nominal, speed, wheelbase, steering_sign, ros_now, wall_now, dt):
        mode = self.mode
        blackout = bool(mode and mode[0] == 'IMU_COMPETITION_DEAD_RECKONING')
        estimator_fresh = bool(mode and 0 <= ros_now-mode[1] <= .5
                               and 0 <= wall_now-mode[2] <= .5)
        while self.frames and (not -.05 <= ros_now-self.frames[0][0] <= .8
                               or not 0 <= wall_now-self.frames[0][1] <= .8):
            self.frames.popleft()
        frames = list(self.frames)
        valid = bool(len(frames) >= 4 and frames[-1][0]-frames[0][0] >= .2-1e-9
                     and -.05 <= ros_now-frames[-1][0] <= self.timeout
                     and 0 <= wall_now-frames[-1][1] <= self.timeout)
        step = min(.1, max(0., dt))
        cap, stop, reason = None, False, 'gps_route_priority'
        if blackout and valid and estimator_fresh:
            y7 = median(-f[2] for f in frames[-3:])
            heading = median(f[3] for f in frames[-3:])
            x = max(7., min(14., 4.+.4*speed))
            y = y7 + (x-7.)*math.tan(heading)
            curvature = 2.*y/(x*x+y*y)
            self.last_target = steering_sign*max(-.25, min(.25, math.atan(wheelbase*curvature)))
            self.blend = min(1., self.blend+2.*step)
            self.loss_since = None
            cap = min(self.maximum_speed, math.sqrt(1.5/max(abs(curvature), 1e-4)))
            reason = 'tunnel_camera_priority'
        elif blackout:
            if self.loss_since is None:
                self.loss_since = wall_now
            cap = min(self.maximum_speed, 2.)
            stop = wall_now-self.loss_since >= 1.
            reason = ('tunnel_estimator_stale_stop' if not estimator_fresh else
                      'tunnel_lane_missing_stop') if stop else 'tunnel_lane_acquiring_slow'
        else:
            self.loss_since = None
            self.blend = max(0., self.blend-step)
        steering = nominal*(1.-self.blend)+self.last_target*self.blend
        return dict(steering=steering, speed_cap_mps=cap, stop=stop, reason=reason,
                    blackout=blackout, lane_valid=valid, camera_weight=self.blend)
