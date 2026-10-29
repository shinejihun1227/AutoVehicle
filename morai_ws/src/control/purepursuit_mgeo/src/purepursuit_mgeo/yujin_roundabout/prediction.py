"""Path-conditioned, sampled space/time footprint checks for simulator testing.

Three ego response profiles and three object speed profiles are evaluated.
This finite hypothesis set is not a guarantee against arbitrary acceleration,
unseen traffic or an incorrectly calibrated vehicle footprint.
"""
import math
import numpy as np


class MotionPath:
    def __init__(self, polyline, closed=False):
        a=polyline._segments
        self.s=np.r_[a[:,5],a[-1,5]+a[-1,4]]
        self.x=np.r_[a[:,0],a[-1,0]+a[-1,2]*a[-1,4]]
        self.y=np.r_[a[:,1],a[-1,1]+a[-1,3]*a[-1,4]]
        self.heading=np.unwrap(np.arctan2(a[:,3],a[:,2]))
        self.heading=np.r_[self.heading,self.heading[-1]]
        self.curvature=np.abs(np.gradient(self.heading,self.s))
        self.length=float(self.s[-1])
        self.closed=closed

    def sample(self,s):
        q=np.asarray(s,dtype=float)
        if self.closed:q=q%self.length
        else:q=np.clip(q,0,self.length)
        return np.interp(q,self.s,self.x),np.interp(q,self.s,self.y),np.interp(q,self.s,self.heading)


class MergePredictor:
    def __init__(self, path, circulating, cfg):
        self.path,self.circulating,self.c=path,circulating,cfg
        p=circulating.points
        closed=math.hypot(p[-1].x-p[0].x,p[-1].y-p[0].y)<.5
        self.ego_path=MotionPath(path)
        self.circ_path=MotionPath(circulating,closed)
        self.dt=cfg.get('prediction_step_s',.2)
        self.horizon=cfg.get('trajectory_horizon_s',60.)
        if not .02<=self.dt<=.25 or not 5<=self.horizon<=60:
            raise ValueError('prediction step must be .02..25 s and horizon 5..60 s')

    def track_motion(self,track):
        lat,s,tx,ty=self.circulating.project(track.x,track.y)
        speed=math.hypot(track.vx,track.vy)
        along=track.vx*tx+track.vy*ty
        if speed>=self.c['min_vehicle_speed'] and lat<=self.c['corridor_m']+track.width/2 and along/max(speed,1e-9)>=self.c['direction_cos_min']:
            x,y,yaw=self.circ_path.sample(s)
            # Retain measured lateral/heading offsets; never snap the object
            # onto a potentially different lane centre.
            lateral=-(track.x-float(x))*ty+(track.y-float(y))*tx
            angle=math.atan2(math.sin(track.yaw-float(yaw)),math.cos(track.yaw-float(yaw)))
            return s,speed,lateral,angle
        return None

    def propagate(self,track,age):
        motion=self.track_motion(track)
        if motion is None:
            return track.x+track.vx*age,track.y+track.vy*age,track.yaw,track.vx,track.vy
        s,v,lat,offset=motion
        x,y,h=self.circ_path.sample(s+v*age)
        return float(x-lat*np.sin(h)),float(y+lat*np.cos(h)),float(h+offset),v*float(np.cos(h)),v*float(np.sin(h))

    def ego_trajectory(self,ego,s,end_s,target):
        c=self.c;dt=self.dt
        times=np.arange(0,self.horizon+dt*.5,dt)
        # Slow profile covers lower pedal response and a delayed initial launch.
        accelerations=np.array([c.get('prediction_min_accel',.4),c['max_accel'],c['max_accel']])
        fractions=np.array([c.get('prediction_min_speed_fraction',.65),1.,1.])
        delays=np.array([c.get('prediction_start_delay_s',.75),0.,0.]) if ego.speed<.15 else np.zeros(3)
        speed=np.full(3,max(0.,ego.speed));progress=np.full(3,s)
        result_s=[];result_v=[]
        for t in times:
            result_s.append(progress.copy());result_v.append(speed.copy())
            curvature=np.interp(progress,self.ego_path.s,self.ego_path.curvature)
            cap=np.minimum(target,np.sqrt(c.get('max_lateral_accel',1.5)/np.maximum(curvature,1e-6)))*fractions
            dv=np.clip(cap-speed,-c['comfortable_decel']*dt,accelerations*dt)
            next_v=np.maximum(0.,speed+dv)
            next_v=np.where(t<delays,0.,next_v)
            progress=progress+.5*(speed+next_v)*dt
            speed=next_v
        ss=np.asarray(result_s).T;vv=np.asarray(result_v).T
        complete=bool(np.all(ss[:,-1]>=end_s))
        # Check through route exit. Samples after each profile's exit are unused.
        valid=ss<=end_s+target*dt
        x,y,yaw=self.ego_path.sample(ss)
        x0,y0,h0=self.ego_path.sample(s)
        blend=np.clip(1-(ss-s)/4.,0,1)
        x=x+blend*(ego.x-x0);y=y+blend*(ego.y-y0)
        delta=math.atan2(math.sin(ego.yaw-float(h0)),math.cos(ego.yaw-float(h0)))
        yaw=yaw+blend*delta
        offset=c['ego_front_m']-c['ego_length']/2
        return times,x+offset*np.cos(yaw),y+offset*np.sin(yaw),yaw,vv,valid,complete

    def assess(self,ego,tracks,s,end_s,target):
        c=self.c
        times,ex,ey,eh,ev,valid,complete=self.ego_trajectory(ego,s,end_s,target)
        out=dict(prediction_complete=complete,predicted_blocking_ids=[],predicted_collision_time=None,predicted_collisions=[],
                 prediction_model='path_footprints_3x3',prediction_horizon_s=float(times[-1]))
        if not tracks:return out
        ex,ey,eh,ev,valid=[a[:,None,:] for a in (ex,ey,eh,ev,valid)]
        for tr in tracks:
            speed=math.hypot(tr.vx,tr.vy)
            # Roundabout-only merge logic: stationary objects are not
            # circulating traffic. Ignore them in merge-gap trajectory prediction.
            # This prevents roadside/parked LiDAR tracks with tiny Kalman residual
            # velocity from permanently blocking roundabout entry.
            if speed < c['min_vehicle_speed']:
                continue
            uncertainty=max(c.get('prediction_speed_error_mps',.3),speed*c.get('prediction_speed_error_fraction',.15)) if speed>=c['min_vehicle_speed'] else 0.
            speeds=np.maximum(0.,speed+np.array([-uncertainty,0.,uncertainty]))
            motion=self.track_motion(tr)
            if motion is not None:
                os,_,lat,offset=motion
                ox,oy,oh=self.circ_path.sample(os+speeds[:,None]*times)
                ox=ox-lat*np.sin(oh);oy=oy+lat*np.cos(oh);oh=oh+offset
            else:
                direction=math.atan2(tr.vy,tr.vx) if speed>0 else tr.yaw
                ox=tr.x+speeds[:,None]*np.cos(direction)*times
                oy=tr.y+speeds[:,None]*np.sin(direction)*times
                oh=np.full_like(ox,tr.yaw)
            ox,oy,oh=[a[None,:,:] for a in (ox,oy,oh)]
            ov=speeds[None,:,None]
            # Base geometric margin plus travel between sample instants. Rotation
            # adds circumradius*angular travel to reduce temporal aliasing.
            ew=np.abs(np.gradient(eh,self.dt,axis=-1))
            ow=np.abs(np.gradient(np.unwrap(oh,axis=-1),self.dt,axis=-1))
            margin=(c.get('prediction_margin_m',.45)+(ev+ov)*self.dt/2+
                    (ew*math.hypot(c['ego_length'],c['ego_width'])/2+
                     ow*math.hypot(tr.length,tr.width)/2)*self.dt/2)
            dx,dy=ox-ex,oy-ey
            overlap=np.broadcast_to(valid,(3,3,len(times))).copy()
            for angle in (eh,eh+math.pi/2,oh,oh+math.pi/2):
                ax,ay=np.cos(angle),np.sin(angle)
                radius=(c['ego_length']/2*np.abs(np.cos(eh-angle))+
                        c['ego_width']/2*np.abs(np.sin(eh-angle))+
                        tr.length/2*np.abs(np.cos(oh-angle))+
                        tr.width/2*np.abs(np.sin(oh-angle))+margin)
                overlap &= np.abs(dx*ax+dy*ay)<=radius
            hits=np.flatnonzero(np.any(overlap,axis=(0,1)))
            if len(hits):
                out['predicted_blocking_ids'].append(tr.id)
                t=float(times[hits[0]])
                out['predicted_collisions'].append(dict(id=tr.id,time_s=t))
                if out['predicted_collision_time'] is None or t<out['predicted_collision_time']:
                    out['predicted_collision_time']=t
        return out
