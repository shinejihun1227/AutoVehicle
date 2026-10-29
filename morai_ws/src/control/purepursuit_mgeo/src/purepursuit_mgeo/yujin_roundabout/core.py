"""ROS-independent roundabout decision model. SI units, map coordinates.

Approach speed is independent of gap permission. Gap checking starts at the
yield position; immediate collision and sensor watchdog checks always apply.
"""
import math
from dataclasses import dataclass
import numpy as np
from .prediction import MergePredictor
from .entry_decision import ObservationWindow, ConflictEvaluator


@dataclass
class Ego:
    x: float
    y: float
    yaw: float
    speed: float


@dataclass
class Track:
    id: int
    x: float
    y: float
    yaw: float
    length: float
    width: float
    vx: float
    vy: float


class Polyline:
    def __init__(self, points):
        self.points = list(points)
        self.s = [0.0]
        self.segments = []
        for a, b in zip(self.points, self.points[1:]):
            dx, dy = b.x-a.x, b.y-a.y
            length = math.hypot(dx, dy)
            self.s.append(self.s[-1]+length)
            if length > 1e-6:
                self.segments.append((a.x, a.y, dx/length, dy/length, length, self.s[-2]))
        if not self.segments or not all(math.isfinite(v) for p in points for v in (p.x, p.y)):
            raise ValueError('Path must contain finite nonidentical points')
        self._segments = np.asarray(self.segments)

    def project(self, x, y):
        a = self._segments
        along = np.clip((x-a[:,0])*a[:,2]+(y-a[:,1])*a[:,3],0,a[:,4])
        d2 = (x-a[:,0]-along*a[:,2])**2+(y-a[:,1]-along*a[:,3])**2
        i = int(np.argmin(d2))
        return math.sqrt(float(d2[i])),float(a[i,5]+along[i]),float(a[i,2]),float(a[i,3])


def eta(distance, speed, target, acceleration):
    """Accelerate/decelerate to target then cruise; stationary starts are finite."""
    if distance <= 0:
        return 0.0
    if target <= 0 or acceleration <= 0:
        raise ValueError('ETA target and acceleration must be positive')
    speed = max(0., speed)
    sign = 1. if target >= speed else -1.
    t = abs(target-speed)/acceleration
    d = (speed+target)*t/2
    if distance > d:
        return t+(distance-d)/target
    return 2*distance/(speed+math.sqrt(max(0., speed*speed+2*sign*acceleration*distance)))


def box_axes(yaw):
    return ((math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw)))


def projected_radius(length, width, yaw, axis):
    u, v = box_axes(yaw)
    return .5*(length*abs(u[0]*axis[0]+u[1]*axis[1]) +
               width*abs(v[0]*axis[0]+v[1]*axis[1]))


def swept_boxes(ego, track, cfg):
    """Continuous SAT for two oriented boxes, constant yaw/velocity.

    Returns a signed separating-axis gap (NOT Euclidean clearance) and TTC.
    Side and crossing objects are retained; no class/size based deletion.
    """
    offset = cfg['ego_front_m']-cfg['ego_length']/2
    ex, ey = ego.x+offset*math.cos(ego.yaw), ego.y+offset*math.sin(ego.yaw)
    dx, dy = track.x-ex, track.y-ey
    vx = track.vx-ego.speed*math.cos(ego.yaw)
    vy = track.vy-ego.speed*math.sin(ego.yaw)
    enter, leave, gap = 0., float('inf'), -float('inf')
    for axis in box_axes(ego.yaw)+box_axes(track.yaw):
        r = (projected_radius(cfg['ego_length'], cfg['ego_width'], ego.yaw, axis) +
             projected_radius(track.length, track.width, track.yaw, axis) + cfg['emergency_margin_m'])
        d, v = dx*axis[0]+dy*axis[1], vx*axis[0]+vy*axis[1]
        gap = max(gap, abs(d)-r)
        if abs(v) < 1e-9:
            if abs(d) > r:
                leave = -1.
        else:
            a, b = sorted(((-r-d)/v, (r-d)/v))
            enter, leave = max(enter, a), min(leave, b)
    return gap, enter if enter <= leave and leave >= 0 else None


class Decision:
    def __init__(self, path, circulating, cfg):
        self.path, self.circulating, self.c = Polyline(path), Polyline(circulating), cfg
        if type(cfg.get('lead_relative_following_enabled', True)) is not bool:
            raise ValueError('lead_relative_following_enabled must be boolean')
        for name, default, lower, upper in (
                ('lead_follow_alignment_cos_min', .9, .5, 1.),
                ('lead_follow_lateral_tolerance_m', .9, 0., 2.),
                ('lead_relative_reaction_s', .3, 0., 2.)):
            value = cfg.get(name, default)
            if type(value) not in (int, float) or not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError('invalid '+name)
        names = ['approach', 'stop_line', 'entry', 'conflict_start', 'conflict_end', 'merge_complete', 'exit']
        indexes = [cfg['mission_indices'][n] for n in names]
        if any(type(i) is not int or not 0 <= i < len(path) for i in indexes):
            raise ValueError('Set all mission_indices to valid zero-based path point indices')
        if any(a >= b for a, b in zip(indexes, indexes[1:])):
            raise ValueError('Mission indices must be strictly increasing')
        self.m = dict(zip(names, [self.path.s[i] for i in indexes]))
        ci = cfg['circulating_conflict_indices']
        if len(ci) != 2 or any(type(i) is not int or not 0 <= i < len(circulating) for i in ci) or ci[0] >= ci[1]:
            raise ValueError('Set two ordered circulating_conflict_indices')
        self.cs, self.ce = (self.circulating.s[i] for i in ci)
        for name in ['approach_speed', 'merge_speed', 'roundabout_speed', 'exit_speed', 'max_accel', 'comfortable_decel', 'time_headway', 'minimum_gap', 'safe_confirm_s', 'track_hold_s', 'corridor_m', 'emergency_ttc', 'min_vehicle_speed', 'prediction_horizon', 'ego_length', 'ego_width']:
            if not math.isfinite(cfg[name]) or cfg[name] <= 0:
                raise ValueError(name+' must be finite and positive')
        if self.m['stop_line'] + cfg['ego_front_m'] + cfg['stop_margin_m'] >= self.m['conflict_start']:
            raise ValueError('Stop line must precede conflict by front overhang and margin')
        if len(cfg['roi_map']) != 4 or not all(math.isfinite(v) for v in cfg['roi_map']):
            raise ValueError('Set finite roi_map [xmin,xmax,ymin,ymax]')
        if cfg['roi_map'][0] >= cfg['roi_map'][1] or cfg['roi_map'][2] >= cfg['roi_map'][3]:
            raise ValueError('Invalid ROI bounds')
        if not 0 < cfg['ego_front_m'] < cfg['ego_length']:
            raise ValueError('ego_front_m must be the measured base_link-to-front distance')
        if not 0 < cfg['direction_cos_min'] <= 1:
            raise ValueError('direction_cos_min must be in (0,1]')
        for name in ['rate_hz','odom_timeout_s','obstacle_timeout_s','decision_timeout_s','lead_range_m','max_path_error_m','max_accel_rate','max_brake_rate']:
            if not math.isfinite(cfg[name]) or cfg[name] <= 0:
                raise ValueError(name+' must be positive')
        for name in ['front_time_buffer_s','rear_time_buffer_s','stop_margin_m','stop_tolerance_m','wait_distance_m','commit_distance_m','gap_kp','emergency_margin_m','emergency_gap_m']:
            if not math.isfinite(cfg[name]) or cfg[name] < 0:
                raise ValueError(name+' must be nonnegative')
        self.state = 'APPROACH'
        self.entry_was_authorized = False
        self.clear_since = None
        self.last_time = None
        self.target = 0.
        self.static_hazards = []
        self.frame_serial = 0
        self.frame_stamp = None
        self.safe_frames = 0
        self.safe_serial = -1
        self.yield_hold = False

        # Mandatory one-time stop before roundabout entry.
        # The vehicle must stop once at global-path index 1792,
        # then make the gap decision while stationary.
        self.entry_decision_stop_index = int(
            cfg.get('entry_decision_stop_index', 1792)
        )
        if not 0 <= self.entry_decision_stop_index < len(self.path.s):
            raise ValueError('entry_decision_stop_index out of range')

        self.entry_decision_stop_s = self.path.s[
            self.entry_decision_stop_index
        ]

        self.entry_stop_completed = False
        self.traffic_seen_after_stop = False
        self.empty_confirm_since = None

        self.merge_target = cfg['merge_speed']
        self.observations = ObservationWindow(cfg)
        # Most recent raw source frame.
        # Used by the ID-independent circulating-flow memory so that
        # historical ObservationWindow samples never refresh a vehicle forever.
        self.latest_tracks = tuple()
        self.predictor = MergePredictor(self.path,self.circulating,cfg)
        self.use_prediction = cfg.get('trajectory_prediction_enabled',True)
        self.merge_ceiling = min(cfg.get('maximum_merge_speed',cfg['merge_speed']),
                                 cfg['test_speed_limit'])
        if not math.isfinite(self.merge_ceiling) or self.merge_ceiling < cfg['merge_speed']:
            raise ValueError('merge candidate ceiling must be finite and >= merge_speed')
        if self.use_prediction:
            self.merge_target = self.merge_ceiling
        self.conflicts = ConflictEvaluator(self)

    def update_tracks(self, tracks, now):
        """New source frames only. Raw IDs are diagnostic labels, never keys."""
        if self.frame_stamp is not None and now <= self.frame_stamp:
            return
        self.frame_stamp = now
        self.frame_serial += 1
        self.latest_tracks = tuple(tracks)
        self.observations.update(tracks, now)

    def _tracks(self, now):
        return self.observations.project(now, self.predictor, Track)

    def step(self, ego, now, fresh=True, obstacles_fresh=True):
        c = self.c
        if self.state in ('COMMIT','FOLLOW'):
            self.entry_was_authorized = True
        dt = 0.05 if self.last_time is None else max(0., min(.2, now-self.last_time))
        if self.last_time is not None and now < self.last_time:
            fresh = False
        self.last_time = now
        dist, s, tx, ty = self.path.project(ego.x, ego.y)
        result = dict(timestamp=now, state=self.state, ego_x=ego.x, ego_y=ego.y,
                      ego_speed=ego.speed, ego_yaw_rad=ego.yaw, ego_s=s, active=self.m['approach'] <= s < self.m['exit'],
                      target_speed=0., stop_required=True, allowed=False, emergency=False,
                      blocking_vehicle_id=None, vehicle_eta=None, ego_eta=None, merge_gap=None,
                      lead_vehicle_id=None, lead_x=None, lead_y=None, lead_speed=None,
                      lead_gap=None, desired_gap=None, relative_speed=None, time_gap=None,
                      minimum_ttc=None, minimum_distance=None, decision_reason='',
                      lost_track_ids=[], recovered_track_ids=[], id_handoffs=[],
                      recovery_frames=0, recovery_span_s=0., track_recovery_reason=None,
                      decision_phase='CRUISE', gap_check_active=False, safe_frames=0,
                      safe_confirm_elapsed_s=0., blocking_track_details=[],
                      dangerous_vehicle_count=0, conflict_candidate_count=0,
                      minimum_conflict_eta=None, ego_conflict_eta=None, minimum_time_gap=None,
                      safe_duration=0., commit_latched=self.entry_was_authorized, emergency_active=False,
                      lead_relative_following_active=False,
                      lead_prediction_stop_suppressed=False,
                      lead_close_gap_stop_suppressed=False,
                      lead_prediction_suppressed_ids=[],
                      lead_gap_control_mode='legacy',
                      lead_current_separation_m=None,
                      lead_closing_speed_mps=None,
                      lead_relative_speed_cap_mps=None)
        def finish(reason, target=0., stop=False, emergency=False, launch=False):
            result['decision_reason_detail'] = reason
            # A newly accepted launch must request the speed used in its
            # forecast. Pedal slew is already enforced by the test driver.
            self.target = 0. if stop else (target if launch else min(target, self.target+c['max_accel']*dt))
            # Safety reductions are not delayed by the acceleration limiter.
            result.update(state=self.state, target_speed=max(0., self.target), stop_required=stop,
                          emergency=emergency, decision_reason=reason,
                          allowed=not stop and self.state in ('READY','COMMIT','FOLLOW'))
            result.update(commit_latched=self.entry_was_authorized, emergency_active=emergency,
                          safe_duration=result['safe_confirm_elapsed_s'])
            if emergency:
                result['decision_reason'] = ('emergency_ttc' if reason == 'emergency_collision_risk' else reason)
            elif self.state == 'EXIT':
                result['decision_reason'] = 'exit'
            elif self.state == 'FOLLOW':
                result['decision_reason'] = 'inside_roundabout'
            elif self.state == 'COMMIT':
                result['decision_reason'] = 'safe_gap_confirmed' if launch else 'commit_latched'
            elif self.state == 'READY':
                result['decision_reason'] = 'safe_gap_ready'
            elif result['gap_check_active']:
                result['decision_reason'] = ('waiting_conflict_vehicle' if result['dangerous_vehicle_count'] else 'safe_gap_confirming')
            return result
        if not fresh or dist > c['max_path_error_m']:
            self.clear_since = None
            self.safe_frames = 0
            return finish('stale_or_invalid_input' if not fresh else 'off_path', stop=True, emergency=True)
        # The mission does not own normal driving outside its approach/exit.
        if not result['active']:
            self.state = 'CRUISE' if s < self.m['approach'] else 'EXIT'
            self.entry_was_authorized = False
            self.yield_hold = False
            self.safe_frames = 0

            self.entry_stop_completed = False
            self.traffic_seen_after_stop = False
            self.empty_confirm_since = None
            self.merge_target = self.merge_ceiling if self.use_prediction else c['merge_speed']
            self.clear_since = None
            self.static_hazards.clear()
            return finish('csv_cruise' if self.state == 'CRUISE' else 'mission_complete',
                          c.get('cruise_speed', c['exit_speed']))
        if not obstacles_fresh:
            self.clear_since = None
            self.safe_frames = 0
            return finish('obstacles_stale_or_invalid', stop=True, emergency=True)
        tracks = self._tracks(now)
        # Do not persist low-speed LiDAR fragments for the whole mission.
        #
        # ObservationWindow already keeps recent observations for the bounded
        # occlusion hold time, while circulating FLOW memory handles real
        # vehicles hidden by the roundabout occluder.
        #
        # Therefore a low-speed fragment may remain conservative briefly via
        # `tracks`, but it cannot become a permanent ghost blocker.
        self.static_hazards.clear()
        static_cap = float('inf')
        emergency = False
        blockers = []
        front, rear = c['ego_front_m'], c['ego_length']-c['ego_front_m']
        # Distance to the mandatory entry-decision stop point.
        available_distance = max(
            0.0,
            self.entry_decision_stop_s - s
        )
        near_yield = available_distance <= c['wait_distance_m']
        gap_check = near_yield or self.state in ('COMMIT','FOLLOW')
        result.update(stop_line_distance_m=available_distance,
                      yield_base_s=self.entry_decision_stop_s,
                      gap_check_active=gap_check,
                      decision_phase='MERGE' if self.state in ('COMMIT','FOLLOW') else ('YIELD' if near_yield else 'APPROACH'))
        ego_in = eta(self.m['conflict_start']-s-front, ego.speed, c['merge_speed'], c['max_accel'])
        ego_out = eta(self.m['conflict_end']-s+rear, ego.speed, c['merge_speed'], c['max_accel'])
        result['ego_eta'] = ego_in
        lead = None
        ttc_risks = []
        for t in tracks:
            # Project first so that, after COMMIT, we can distinguish a
            # clearly trailing vehicle from a front/side collision threat.
            lateral, os, ox, oy = self.path.project(t.x, t.y)

            sep, ttc = swept_boxes(ego, t, c)
            result['minimum_distance'] = sep if result['minimum_distance'] is None else min(sep, result['minimum_distance'])

            # Once entry has been committed, a vehicle clearly behind the ego
            # must not make the ego brake in the merge. The trailing vehicle
            # is expected to yield/decelerate. Front and side threats remain
            # protected by the normal TTC emergency logic.
            trailing_after_commit = (
                self.entry_was_authorized
                and self.state in ('COMMIT', 'FOLLOW')
                and os < s
            )

            if ttc is not None:
                result['minimum_ttc'] = ttc if result['minimum_ttc'] is None else min(ttc, result['minimum_ttc'])

                # Decide the front-only exception after the closest lead is
                # selected. Raw IDs are labels, never exemption keys.
                ttc_risks.append((t, ttc, trailing_after_commit))

            # Roundabout-only lead following: only moving circulating/lead
            # vehicles may reduce the commanded merge speed. Stationary LiDAR
            # objects are excluded from lead selection.
            speed = math.hypot(t.vx,t.vy)
            projected = t.vx*ox+t.vy*oy
            aligned = speed >= c['min_vehicle_speed'] and projected/max(speed,1e-9) >= c['direction_cos_min']
            along_radius = projected_radius(t.length, t.width, t.yaw, (ox, oy))
            side_radius = projected_radius(t.length, t.width, t.yaw, (-oy, ox))
            gap = os-s-front-along_radius
            if (speed < c['min_vehicle_speed'] and os > s and
                    lateral <= c['ego_width']/2+side_radius+c['emergency_margin_m']):
                static_cap = min(static_cap, math.sqrt(max(0.,2*c['comfortable_decel']*(gap-c.get('static_minimum_gap_m', c['minimum_gap'])))))
                if gap <= c['emergency_gap_m']:
                    emergency = True
            if (speed >= c['min_vehicle_speed'] and
                    lateral <= c['ego_width']/2 + side_radius + c['emergency_margin_m'] and
                    os > s and gap < c['lead_range_m'] and aligned):
                if lead is None or gap < lead[0]:
                    lead = (gap, max(0.,projected), t)
        relative_follow = False
        if lead and c.get('lead_relative_following_enabled', True):
            _, _, lead_track = lead
            lead_lateral, _, ltx, lty = self.path.project(lead_track.x, lead_track.y)
            lead_magnitude = math.hypot(lead_track.vx, lead_track.vy)
            alignment = c.get('lead_follow_alignment_cos_min', .9)
            lead_separation, _ = swept_boxes(ego, lead_track, c)
            result['lead_current_separation_m'] = lead_separation
            relative_follow = (
                self.entry_was_authorized
                and self.state in ('COMMIT', 'FOLLOW')
                and lead_separation > 0.
                and dist <= c.get('lead_follow_lateral_tolerance_m', .9)
                and lead_lateral <= c.get('lead_follow_lateral_tolerance_m', .9)
                and math.cos(ego.yaw)*tx+math.sin(ego.yaw)*ty >= alignment
                and (lead_track.vx*ltx+lead_track.vy*lty)/max(lead_magnitude, 1e-9) >= alignment)
        result['lead_relative_following_active'] = relative_follow
        for risk_track, risk_ttc, trailing_after_commit in ttc_risks:
            if risk_ttc <= c['emergency_ttc'] and not trailing_after_commit:
                if relative_follow and risk_track is lead[2]:
                    # Continuous relative-speed following replaces this lead's
                    # future constant-velocity collision stop. Current box
                    # overlap, other cars and rear handling stay protected.
                    result['lead_prediction_stop_suppressed'] = True
                    result['lead_prediction_suppressed_ids'].append(risk_track.id)
                else:
                    emergency = True
                    result.setdefault('emergency_track_ids', []).append(risk_track.id)
        if lead:
            gap, speed, t = lead
            desired = c['minimum_gap'] + c['time_headway']*ego.speed
            result.update(lead_vehicle_id=t.id, lead_x=t.x, lead_y=t.y, lead_speed=speed,
                          lead_gap=gap, desired_gap=desired, relative_speed=ego.speed-speed,
                          time_gap=gap/max(ego.speed,.1))
            if relative_follow and gap <= c['emergency_gap_m']:
                # A nearby moving lead does not request a full stop simply
                # because of distance; match its speed and limit closing.
                result['lead_close_gap_stop_suppressed'] = True
            elif gap <= c['emergency_gap_m']:
                emergency = True
                result.setdefault('emergency_track_ids', []).append(t.id)
        if gap_check:
            assessment = self.conflicts.evaluate(ego, tracks, s)
            result.update(assessment)
            blockers = assessment.pop('_blockers', [])
            result.pop('_blockers', None)
            if blockers:
                result['vehicle_eta'], result['blocking_vehicle_id'], result['merge_gap'] = min(blockers)
            if (
                not self.entry_was_authorized
                and assessment.get('predicted_collision_time') is not None
                and assessment['predicted_collision_time'] <= c['emergency_ttc']
            ):
                emergency = True
            if not assessment['prediction_complete'] and not self.entry_was_authorized:
                self.clear_since = None
                self.safe_frames = 0
                return finish('prediction_horizon_insufficient', stop=True, emergency=True)
        if emergency:
            self.clear_since = None
            self.safe_frames = 0
            return finish('emergency_collision_risk', stop=True, emergency=True)

        # ----------------------------------------------------
        # Mandatory stop at entry-decision point.
        # Do not authorize COMMIT before one complete stop.
        # ----------------------------------------------------
        if not self.entry_stop_completed:
            stop_tol = c.get('entry_decision_stop_tolerance_m', 0.35)
            stopped_speed = c.get('entry_decision_stopped_speed', 0.15)

            if available_distance <= stop_tol:
                self.state = 'WAIT'
                self.clear_since = None
                self.safe_frames = 0

                if ego.speed <= stopped_speed:
                    self.entry_stop_completed = True

                    flow_now = (
                        result.get('flow_vehicle_hypothesis_count', 0)
                        or 0
                    )

                    if flow_now > 0:
                        # A real circulating vehicle has already been seen.
                        # Never go back to the "empty road" mode.
                        self.traffic_seen_after_stop = True
                        self.empty_confirm_since = None
                    else:
                        # Initially empty: start the one-time 0.5 s check.
                        self.empty_confirm_since = now

                return finish(
                    'mandatory_entry_decision_stop',
                    stop=True
                )

            # Normal approach until reaching index 1792.
            self.state = 'APPROACH'
            approach_cap = math.sqrt(
                max(
                    0.0,
                    2.0 * c['comfortable_decel']
                    * available_distance
                )
            )

            target = min(c['approach_speed'], approach_cap)

            if lead:
                gap, speed, _ = lead
                target = min(
                    target,
                    max(
                        0.0,
                        speed
                        + c['gap_kp']
                        * (gap-result['desired_gap'])
                    ),
                    math.sqrt(
                        max(
                            0.0,
                            speed*speed
                            + 2*c['comfortable_decel']
                            * (gap-c['minimum_gap'])
                        )
                    )
                )

            target = min(target, static_cap)

            return finish(
                'approach_entry_decision_stop',
                target
            )
        if not gap_check:
            self.state = 'APPROACH'
            self.clear_since = None
            self.safe_frames = 0
            target = min(c['approach_speed'], math.sqrt(2*c['comfortable_decel']*available_distance))
            if lead:
                gap, speed, _ = lead
                target = min(target, max(0.,speed+c['gap_kp']*(gap-result['desired_gap'])),
                             math.sqrt(max(0.,speed*speed+2*c['comfortable_decel']*(gap-c['minimum_gap']))))
            target = min(target, static_cap)
            return finish('approach_to_yield' if not lead else 'lead_following_approach',target)
        # Once a safe gap has been confirmed and COMMIT starts, do not
        # bounce back to WAIT only because a new non-emergency gap blocker appears.
        # Immediate collision/emergency checks above remain active. This avoids
        # stop-go hesitation while the ego vehicle is accelerating into the merge.
        launch = False

        if s >= self.m['exit']:
            self.state = 'EXIT'
            target = c['exit_speed']
        elif self.state in ('COMMIT','FOLLOW'):
            if s >= self.m['merge_complete']:
                self.state = 'FOLLOW'

            # Keep strong acceleration while actually merging.
            # After clearing the conflict zone, hold the lower
            # roundabout cruising speed regardless of prediction mode.
            if s < self.m['conflict_end']:
                base_target = (
                    self.merge_target
                    if self.use_prediction
                    else c['merge_speed']
                )
            else:
                base_target = c['roundabout_speed']

            # Start reducing speed before the physical roundabout exit.
            # Global-path index 1860 is near the measured exit-start point
            # (-81.81, 339.33), so decelerate over roughly the preceding 10 m.
            decel_start_s = self.path.s[1840]
            decel_end_s = self.path.s[1860]

            if s >= decel_start_s:
                span = max(0.1, decel_end_s - decel_start_s)
                progress = max(
                    0.0,
                    min(1.0, (s - decel_start_s) / span)
                )

                # Smoothstep for gradual acceleration release / deceleration.
                smooth = progress * progress * (3.0 - 2.0 * progress)

                decel_target = (
                    self.merge_target
                    + (c['exit_speed'] - self.merge_target) * smooth
                )

                target = min(base_target, decel_target)

                result['exit_decel_progress'] = progress
                result['exit_speed_target'] = decel_target
            else:
                target = base_target
        elif s >= self.m['entry'] and not self.entry_was_authorized:
            # Starting/teleporting inside mission is NOT evidence of a committed merge.
            return finish('uncommitted_entry_restart_before_stop_line', stop=True, emergency=True)
        else:
            if blockers:
                self.clear_since = None
                self.safe_frames = 0
            elif self.frame_stamp is not None and self.frame_serial != self.safe_serial:
                self.safe_serial = self.frame_serial
                if self.clear_since is None:
                    self.clear_since = self.frame_stamp
                self.safe_frames += 1
            span = 0. if self.clear_since is None else self.frame_stamp-self.clear_since

            # Only the ID-independent circulating FLOW is used here.
            # Highway/non-circulating objects are not counted.
            flow_present = (
                result.get('flow_vehicle_hypothesis_count', 0)
                or 0
            ) > 0

            if flow_present:
                self.traffic_seen_after_stop = True
                self.empty_confirm_since = None

            elif (
                self.entry_stop_completed
                and not self.traffic_seen_after_stop
                and self.empty_confirm_since is None
            ):
                self.empty_confirm_since = now

            empty_elapsed = (
                0.0
                if self.empty_confirm_since is None
                else max(0.0, now-self.empty_confirm_since)
            )

            result.update(
                safe_frames=self.safe_frames,
                safe_confirm_elapsed_s=span,
                traffic_seen_after_stop=self.traffic_seen_after_stop,
                initial_empty_elapsed_s=empty_elapsed,
                entry_stop_completed=self.entry_stop_completed
            )

            safe = (
                self.safe_frames >= 2
                and span+1e-9 >= c['safe_confirm_s']
            )

            # Special 0.5 s delay is used ONLY when no circulating
            # vehicle has been seen since the mandatory stop.
            if (
                safe
                and self.entry_stop_completed
                and not self.traffic_seen_after_stop
            ):
                safe = (
                    empty_elapsed
                    >= c.get('initial_empty_confirm_s', 0.5)
                )

            approach_cap = math.sqrt(2*c['comfortable_decel']*available_distance)
            if safe:
                if c.get('evaluation_mode', False):
                    # Stationary gap-evaluation mode:
                    # report that entry is currently safe, but do not latch
                    # COMMIT because the ego vehicle is intentionally not moving.
                    self.state = 'READY'
                    self.entry_was_authorized = False
                    self.yield_hold = False
                    launch = False
                    target = 0.0
                else:
                    # Normal driving mode: preserve the existing behavior.
                    self.state = 'COMMIT'
                    self.entry_was_authorized = True
                    self.yield_hold = False
                    launch = True
                    target = self.merge_target
            else:
                self.state = 'WAIT' if available_distance <= c['wait_distance_m'] else 'APPROACH'
                target = min(c['approach_speed'], approach_cap)
                self.yield_hold = self.yield_hold or available_distance <= c['stop_tolerance_m'] or (near_yield and ego.speed < .15)
                if self.yield_hold:
                    return finish('waiting_for_safe_gap', stop=True)
        if lead:
            gap, speed, _ = lead
            # Continuous headway feedback plus a stopping-distance speed ceiling.
            if relative_follow:
                # Desired gap is an allowance to close a larger gap, rather
                # than a reason to stop behind a lead that is pulling away.
                following = speed+c['gap_kp']*max(0., gap-result['desired_gap'])
                available = max(0., gap-c['minimum_gap'])
                decel = c['comfortable_decel']
                response = c.get('lead_relative_reaction_s', .3)
                # Bound dv*response + dv^2/(2*decel) by the available gap.
                closing_cap = max(0., math.sqrt((decel*response)**2+2*decel*available)-decel*response)
                braking_cap = speed+closing_cap
                result.update(lead_gap_control_mode='relative_speed',
                              lead_closing_speed_mps=max(0., ego.speed-speed),
                              lead_relative_speed_cap_mps=braking_cap)
            else:
                following = max(0., speed+c['gap_kp']*(gap-result['desired_gap']))
                braking_cap = math.sqrt(max(0., speed*speed+2*c['comfortable_decel']*(gap-c['minimum_gap'])))
            target = min(target, following, braking_cap)
        target = min(target, static_cap)
        reason = 'lead_relative_following' if relative_follow else ('lead_following' if lead else self.state.lower())
        return finish(reason, target, launch=launch)
