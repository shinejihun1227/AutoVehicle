"""ID-independent observations -> occupancy intervals -> physical collision checks.

Historical observations are independent motion hypotheses, not recovered tracks.
Their lifetime is bounded from the source stamp and cannot be refreshed by ticks.
"""
import math
from collections import deque
from dataclasses import replace


class ObservationWindow:
    def __init__(self, cfg):
        self.hold = cfg.get('entry_occlusion_hold_s', 0.8)
        if not math.isfinite(self.hold) or not 0.4 <= self.hold <= 3.0:
            raise ValueError('entry_occlusion_hold_s must be 0.4..3 seconds')
        self.frames = deque()
        self.cfg = cfg

    def update(self, tracks, stamp):
        self.frames.append((stamp, tuple(tracks)))
        self._expire(stamp)

    def _expire(self, now):
        while self.frames and now-self.frames[0][0] > self.hold:
            self.frames.popleft()

    def project(self, now, predictor, track_type):
        self._expire(now)
        result = []
        # Latest geometry covers older observations in the same small spatial
        # cell only when their motion agrees within the predictor's uncertainty.
        # This is scene compaction, not raw-ID matching or identity recovery.
        for stamp, tracks in reversed(self.frames):
            for t in tracks:
                x,y,yaw,vx,vy = predictor.propagate(t, max(0., now-stamp))
                candidate = track_type(t.id,x,y,yaw,t.length,t.width,vx,vy)
                # Collapse virtually identical spatial hypotheses, never by ID.
                # Distinct/split fragments stay conservative independent hazards.
                if any(math.hypot(x-o.x,y-o.y)<0.15 and
                       math.hypot(vx-o.vx,vy-o.vy)<0.15 and
                       abs(t.length-o.length)<0.15 and abs(t.width-o.width)<0.15
                       for o in result):
                    continue
                result.append(candidate)
        return result


class ConflictEvaluator:
    def __init__(self, decision):
        self.d = decision

        # ID-independent physical vehicles moving through the circulating lane.
        #
        # A hypothesis is created only from a credible moving circulating
        # observation.  Once created it is propagated along the roundabout
        # path while the vehicle is hidden by the occluder.
        #
        # Raw LiDAR track IDs are diagnostic labels only, never identity keys.
        self.flow_hypotheses = []

        # Recently released physical vehicles.
        #
        # When a vehicle has just cleared the conflict zone, LiDAR may catch
        # fragments of that SAME vehicle again and make it look like a new
        # upstream vehicle.  Keep a short ID-independent "released vehicle"
        # memory so those fragments cannot create a new flow hypothesis.
        self.released_flow = []

        self._last_flow_frame_stamp = None

        for name, default in (('entry_vehicle_length_m',4.5),('entry_vehicle_width_m',1.8)):
            if not math.isfinite(decision.c.get(name,default)) or decision.c.get(name,default)<=0:
                raise ValueError(name+' must be finite and positive')

    def evaluate(self, ego, tracks, s):
        from .core import eta, projected_radius
        d, c = self.d, self.d.c
        target = d.merge_target
        ego_in = eta(d.m['conflict_start']-s-c['ego_front_m'], ego.speed,target,c['max_accel'])
        ego_out = eta(d.m['conflict_end']-s+c['ego_length']-c['ego_front_m'],ego.speed,target,c['max_accel'])
        candidates, blockers, intervals, gaps = [], [], [], []

        # ---------------------------------------------------------
        # Stage 1: collect plausible moving circulating hypotheses.
        #
        # ObservationWindow intentionally keeps recent history so that
        # an occluded vehicle does not immediately disappear.
        # Therefore several historical/split observations may represent
        # the same physical vehicle here.
        # ---------------------------------------------------------
        raw_moving = []

        for t in tracks:
            speed = math.hypot(t.vx,t.vy)
            xmin,xmax,ymin,ymax = c['roi_map']

            if speed < c['min_vehicle_speed']:
                # Do not discard low-speed fragments globally.
                # A real vehicle may temporarily receive a bad velocity
                # estimate while partially occluded.
                lat,os,tx,ty = d.path.project(t.x,t.y)
                radius = projected_radius(t.length,t.width,t.yaw,(tx,ty))
                side = projected_radius(t.length,t.width,t.yaw,(-ty,tx))

                if (s-c['ego_length']+c['ego_front_m'] <= os+radius and
                    os-radius <= d.m['conflict_end'] and
                    lat <= c['ego_width']/2+side+c['emergency_margin_m']):
                    blockers.append((0.,t.id,-ego_out))
                continue

            lat,vs,tx,ty = d.circulating.project(t.x,t.y)
            projected = t.vx*tx+t.vy*ty
            side = projected_radius(t.length,t.width,t.yaw,(-ty,tx))

            circulating = (
                xmin <= t.x <= xmax and
                ymin <= t.y <= ymax and
                lat <= c['corridor_m']+side and
                projected/speed >= c['direction_cos_min']
            )

            if not circulating:
                continue

            # Use a complete vehicle-sized footprint for entry prediction.
            t = replace(
                t,
                length=max(t.length,c.get('entry_vehicle_length_m',4.5)),
                width=max(t.width,c.get('entry_vehicle_width_m',1.8))
            )

            radius = projected_radius(t.length,t.width,t.yaw,(tx,ty))

            # Conservative arrival key: smaller means this hypothesis reaches
            # the conflict region earlier.
            arrival = max(0., (d.cs-vs-radius)/max(projected,1e-9))

            raw_moving.append(dict(
                track=t,
                vs=vs,
                lat=lat,
                projected=projected,
                tx=tx,
                ty=ty,
                arrival=arrival,
            ))

        # ---------------------------------------------------------
        # Stage 2: physical-hypothesis compaction.
        #
        # Do NOT use raw track ID.
        # If recent historical/split observations occupy essentially the
        # same vehicle-length region of the circulating path, treat them
        # as one physical vehicle.
        #
        # Distinct vehicles should have centres separated by roughly one
        # vehicle length or more, so 60% of nominal vehicle length is used
        # as the conservative merge radius.
        # ---------------------------------------------------------
        merge_m = c.get(
            'entry_same_vehicle_merge_m',
            0.60*c.get('entry_vehicle_length_m',4.5)
        )

        physical = []

        for h in raw_moving:
            matched = None

            for i,p_h in enumerate(physical):
                path_close = abs(h['vs'] - p_h['vs']) <= merge_m
                xy_close = math.hypot(
                    h['track'].x - p_h['track'].x,
                    h['track'].y - p_h['track'].y
                ) <= merge_m

                if path_close and xy_close:
                    matched = i
                    break

            if matched is None:
                physical.append(h)
            else:
                # Keep the more conservative hypothesis: the one predicted
                # to reach the conflict region first.
                if h['arrival'] < physical[matched]['arrival']:
                    physical[matched] = h

        moving = [h['track'] for h in physical]

        # Diagnostics: these let us verify that one real vehicle no longer
        # expands into many moving hazard hypotheses.
        raw_count = len(raw_moving)
        physical_count = len(physical)

        # ---------------------------------------------------------
        # Stage 3: persistent ID-independent circulating-flow memory.
        #
        # ObservationWindow protects against short sensor dropouts, but its
        # individual historical samples are still observations.  Here we
        # maintain physical vehicle hypotheses in 1-D roundabout path
        # coordinates.
        #
        # Important behaviour:
        #   - A vehicle does NOT disappear merely because it enters the
        #     occluder's blind zone.
        #   - A reappearing vehicle may have a completely different raw ID.
        #   - A front vehicle is released as soon as its rear clears the
        #     conflict zone.
        #   - A following vehicle remains represented through the blind zone,
        #     so the usable A-B gap is preserved.
        # ---------------------------------------------------------
        now = d.last_time if d.last_time is not None else 0.0

        # ---------------------------------------------------------
        # Recently-cleared vehicle memory.
        #
        # This is NOT an extra waiting time for ego.
        # It only prevents the same physical vehicle from being registered
        # again immediately after it has already cleared the conflict zone.
        #
        # The association radius is intentionally smaller than one vehicle
        # length. Two different normal vehicles cannot physically have their
        # centers this close without overlapping, so a real following vehicle
        # remains independent.
        # ---------------------------------------------------------
        release_memory_s = c.get(
            'entry_release_reacquire_s',
            1.0
        )

        release_match_m = c.get(
            'entry_release_reacquire_m',
            0.55*c.get('entry_vehicle_length_m',4.5)
        )

        self.released_flow = [
            r for r in self.released_flow
            if 0.0 <= now-r['released_at'] <= release_memory_s
        ]

        release_suppressed_count = 0

        # First propagate previously known vehicles along the circulating path.
        for h in self.flow_hypotheses:
            dt_flow = max(0.0, min(0.5, now-h['updated_at']))
            h['vs'] += max(0.0, h['speed']) * dt_flow
            h['updated_at'] = now

        # Only a genuinely new LiDAR source frame may refresh/create a
        # physical vehicle.  Historical ObservationWindow projections must
        # never keep a ghost alive indefinitely.
        frame_stamp = d.frame_stamp

        if (frame_stamp is not None and
                frame_stamp != self._last_flow_frame_stamp):

            self._last_flow_frame_stamp = frame_stamp

            fresh = []

            # Use the latest RAW source frame here, not ObservationWindow
            # history.  Low-speed fragments are intentionally NOT allowed to
            # overwrite a previously credible moving vehicle.
            for t in getattr(d, 'latest_tracks', ()):
                speed = math.hypot(t.vx, t.vy)

                if speed < c['min_vehicle_speed']:
                    continue

                xmin,xmax,ymin,ymax = c['roi_map']

                lat,vs,tx,ty = d.circulating.project(t.x,t.y)
                projected = t.vx*tx + t.vy*ty
                side = projected_radius(t.length,t.width,t.yaw,(-ty,tx))

                circulating = (
                    xmin <= t.x <= xmax and
                    ymin <= t.y <= ymax and
                    lat <= c['corridor_m'] + side and
                    projected/speed >= c['direction_cos_min']
                )

                if not circulating or projected <= 0.0:
                    continue

                t_vehicle = replace(
                    t,
                    length=max(
                        t.length,
                        c.get('entry_vehicle_length_m',4.5)
                    ),
                    width=max(
                        t.width,
                        c.get('entry_vehicle_width_m',1.8)
                    )
                )

                radius = projected_radius(
                    t_vehicle.length,
                    t_vehicle.width,
                    t_vehicle.yaw,
                    (tx,ty)
                )

                arrival = max(
                    0.0,
                    (d.cs-vs-radius)/max(projected,1e-9)
                )

                fresh.append(dict(
                    track=t_vehicle,
                    vs=vs,
                    projected=projected,
                    arrival=arrival,
                ))

            # One source frame may contain several fragments belonging to the
            # same physical vehicle.  Compact them before touching persistent
            # flow memory.
            fresh_physical = []

            for h in fresh:
                matched = None

                for i,p_h in enumerate(fresh_physical):
                    path_close = (
                        abs(h['vs']-p_h['vs']) <= merge_m
                    )

                    xy_close = (
                        math.hypot(
                            h['track'].x-p_h['track'].x,
                            h['track'].y-p_h['track'].y
                        ) <= merge_m
                    )

                    if path_close and xy_close:
                        matched = i
                        break

                if matched is None:
                    fresh_physical.append(h)
                elif h['arrival'] < fresh_physical[matched]['arrival']:
                    # Keep the more conservative observation.
                    fresh_physical[matched] = h

            assoc_m = c.get(
                'entry_flow_assoc_m',
                0.75*c.get('entry_vehicle_length_m',4.5)
            )

            # Existing hypotheses are first propagated to NOW, so nearest
            # along-path association works even if the raw track ID changed
            # while hidden by the occluder.
            unmatched = set(range(len(self.flow_hypotheses)))

            for obs in fresh_physical:
                best = None

                for i in unmatched:
                    h = self.flow_hypotheses[i]
                    ds = abs(obs['vs'] - h['vs'])

                    if ds <= assoc_m:
                        if best is None or ds < best[0]:
                            best = (ds,i)

                if best is None:

                    # Before creating a new vehicle, check whether this is
                    # simply the vehicle that JUST cleared the conflict zone
                    # being reacquired as a LiDAR fragment/new raw ID.
                    #
                    # Identity is based on expected path position, NOT raw ID.
                    released_match = False

                    for r in self.released_flow:
                        dt_release = max(
                            0.0,
                            now-r['released_at']
                        )

                        expected_vs = (
                            r['vs'] +
                            max(0.0,r['speed']) * dt_release
                        )

                        if abs(obs['vs']-expected_vs) <= release_match_m:
                            released_match = True
                            break

                    if released_match:
                        release_suppressed_count += 1
                        continue

                    # Genuine new physical circulating vehicle.
                    self.flow_hypotheses.append(dict(
                        id=obs['track'].id,
                        vs=obs['vs'],
                        speed=max(
                            c['min_vehicle_speed'],
                            obs['projected']
                        ),
                        length=c.get('entry_vehicle_length_m',4.5),
                        last_seen=frame_stamp,
                        updated_at=now,
                    ))
                else:
                    _,i = best
                    unmatched.remove(i)
                    h = self.flow_hypotheses[i]

                    # Position comes directly from the fresh observation.
                    h['vs'] = obs['vs']

                    # Smooth speed so an occlusion-edge fragment cannot make
                    # the predicted rear vehicle suddenly stop or jump.
                    measured = max(
                        c['min_vehicle_speed'],
                        obs['projected']
                    )

                    max_jump = c.get(
                        'entry_flow_speed_jump_mps',
                        1.5
                    )

                    measured = max(
                        h['speed']-max_jump,
                        min(h['speed']+max_jump, measured)
                    )

                    h['speed'] = (
                        0.65*h['speed'] +
                        0.35*measured
                    )

                    # Raw ID is updated for diagnostics only.
                    h['id'] = obs['track'].id
                    h['last_seen'] = frame_stamp
                    h['updated_at'] = now

        # Once a credible circulating vehicle has been created, do NOT
        # delete it simply because LiDAR has not seen it for a fixed number
        # of seconds.  The roundabout occluder can hide the vehicle for long
        # enough that such a timeout creates a false empty gap.
        #
        # The hypothesis already has a credible forward speed and is
        # propagated along circulating-path s every cycle.  Therefore its
        # normal lifetime ends geometrically: when the REAR of the vehicle
        # clears the conflict zone.
        retained = []
        vehicle_length = c.get('entry_vehicle_length_m',4.5)

        # A vehicle that has already crossed in front of ego does not need
        # to remain an entry blocker until the END of the whole conflict zone.
        #
        # Release it once its REAR has cleared a short distance beyond the
        # circulating conflict START.  Following vehicles remain independent
        # flow hypotheses and are still checked normally.
        front_release_s = min(
            d.ce,
            d.cs + c.get('entry_front_release_offset_m', 0.5)
        )

        for h in self.flow_hypotheses:

            # Direction of travel is increasing circulating-path s.
            rear_s = h['vs'] - 0.5*h.get('length',vehicle_length)

            if rear_s > front_release_s:

                # Front vehicle has cleared the merge-entry release point.
                # Remove it immediately from blocking, but retain a short
                # downstream tombstone so fragments of the SAME vehicle
                # cannot be registered again as a new approaching vehicle.
                self.released_flow.append(dict(
                    vs=h['vs'],
                    speed=max(0.0,h['speed']),
                    released_at=now,
                ))

                continue

            retained.append(h)

        self.flow_hypotheses = retained

        flow_blocking_ids = []

        # ---------------------------------------------------------
        # Stage 4: calculate the actual merge time window from the
        # persistent physical vehicle flow.
        #
        # A following vehicle remains here while hidden.  Therefore RAW=0
        # does NOT automatically mean the gap is empty.
        # ---------------------------------------------------------
        for h in self.flow_hypotheses:
            projected = max(h['speed'], 1e-6)
            radius = 0.5*h.get('length',vehicle_length)
            vs = h['vs']

            din  = d.cs-vs-radius
            dout = d.ce-vs+radius

            # Vehicle already fully beyond the conflict interval.
            if dout < 0.0:
                continue

            start = max(0.0, din/projected)
            end   = max(0.0, dout/projected)

            if start > max(
                    c['prediction_horizon'],
                    ego_out+c['rear_time_buffer_s']):
                continue

            candidates.append(h)
            intervals.append(start)

            gap = max(
                start-ego_out,
                ego_in-end
            )

            gaps.append(gap)

            # Safe only if ego occupancy and vehicle occupancy are separated
            # by the configured front/rear temporal buffers.
            if not (
                ego_out+c['rear_time_buffer_s'] < start or
                end+c['front_time_buffer_s'] < ego_in
            ):
                blockers.append((start,h['id'],gap))
                flow_blocking_ids.append(h['id'])

        flow_count = len(self.flow_hypotheses)

        # A hypothesis is considered occluded when it has not received a
        # fresh raw measurement recently, but is still being physically
        # propagated through the blind zone.
        occluded_count = sum(
            1 for h in self.flow_hypotheses
            if now-h['last_seen'] > 0.15
        )

        # Existing geometric predictor remains an independent safety check.
        #
        # However, a FRONT vehicle whose rear has already cleared the
        # merge-entry release point must not be reintroduced as an entry
        # blocker by this second predictor.  Rear/following vehicles remain.
        prediction_moving = []

        for t in moving:
            lat_p, vs_p, tx_p, ty_p = d.circulating.project(t.x, t.y)

            radius_p = projected_radius(
                t.length,
                t.width,
                t.yaw,
                (tx_p, ty_p)
            )

            rear_p = vs_p - radius_p

            if rear_p > front_release_s:
                continue

            prediction_moving.append(t)

        prediction = d.predictor.assess(ego,prediction_moving,s,
            (d.m['exit'] if d.entry_was_authorized else max(s,d.m['conflict_end']+c['ego_length']-c['ego_front_m'])),target) if d.use_prediction else dict(
                prediction_complete=True,predicted_blocking_ids=[],predicted_collisions=[],predicted_collision_time=None)
        for hit in prediction['predicted_collisions']:
            blockers.append((hit['time_s'],hit['id'],0.))
        # Counts are spatial hypotheses, not uniquely identified physical cars.
        return dict(prediction, _blockers=blockers,
                    dangerous_vehicle_count=len(blockers),
                    conflict_candidate_count=len(candidates),
                    raw_circulating_hypothesis_count=raw_count,
                    physical_vehicle_hypothesis_count=physical_count,
                    flow_vehicle_hypothesis_count=flow_count,
                    occluded_vehicle_hypothesis_count=occluded_count,
                    flow_blocking_ids=flow_blocking_ids,
                    released_flow_memory_count=len(self.released_flow),
                    release_suppressed_count=release_suppressed_count,
                    minimum_conflict_eta=min(intervals) if intervals else None,
                    ego_conflict_eta=ego_in, ego_eta=ego_in,
                    minimum_time_gap=min(gaps) if gaps else None,
                    selected_merge_speed_mps=target)
