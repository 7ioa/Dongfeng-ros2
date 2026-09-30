"""Deterministic driving decisions; no ROS or simulator truth access."""
from dataclasses import dataclass
import math
import numpy as np
from .route import wrap


@dataclass
class Observation:
    fresh: bool = True
    clearance: float = math.inf
    lane_valid: bool = True
    lane_error: float = 0.
    light: str = 'unknown'
    light_id: str = ''
    frame: int = 0
    pitch: float = 0.
    stop_distance: float | None = None
    localization_age: float = 0.
    measured_speed: float = 0.


class CommandMux:
    def __init__(self, timeout=.4):
        self.enabled=False
        self.timeout=timeout
        self.manual_velocity=(0.,0.)
        self.manual_time=-math.inf

    def enable(self, enabled):
        self.enabled=bool(enabled)
        self.manual_time=-math.inf

    def manual(self, command, now):
        self.enabled=False
        self.manual_velocity=command
        self.manual_time=now

    def sample(self, now, automatic, auto_time):
        command,stamp=(automatic,auto_time) if self.enabled else (self.manual_velocity,self.manual_time)
        if not 0 <= now-stamp < self.timeout or not all(math.isfinite(v) for v in command):
            return (0.,0.)
        limit=.20 if self.enabled else .25
        return (max(-limit,min(limit,command[0])),max(-1.2,min(1.2,command[1])))


def obstacle_distance(ranges, angles, curvature=0.):
    """Travel until any part of the body sweeps into a laser endpoint."""
    ranges=np.asarray(ranges);angles=np.asarray(angles)
    if (ranges.ndim!=1 or ranges.shape!=angles.shape or not len(ranges)
            or not np.isfinite(angles).all() or not math.isfinite(curvature)):
        return math.nan
    usable=(np.isfinite(ranges)&(ranges>=.03))|np.isposinf(ranges)
    if np.mean(usable)<.8:return math.nan
    angles=(angles+math.pi)%(2*math.pi)-math.pi
    distance=np.linspace(0.,.65,131)
    heading=curvature*distance
    c,s=np.cos(heading),np.sin(heading)
    if abs(curvature)<1e-6:cx,cy=distance,np.zeros_like(distance)
    else:cx,cy=s/curvature,(1-c)/curvature
    # Use all body edges, including rear-side swing on curves.
    ex=np.linspace(-.105,.105,22);ey=np.linspace(-.087,.087,19)
    boundary=np.vstack((np.c_[ex,np.full_like(ex,-.087)],np.c_[ex,np.full_like(ex,.087)],
                        np.c_[np.full_like(ey,-.105),ey],np.c_[np.full_like(ey,.105),ey]))
    bx=c[:,None]*boundary[:,0]-s[:,None]*boundary[:,1]+cx[:,None]
    by=s[:,None]*boundary[:,0]+c[:,None]*boundary[:,1]+cy[:,None]
    new_area=(abs(bx)>.1051)|(abs(by)>.0871)
    bearings=np.arctan2(by[new_area],bx[new_area]-.075)
    required=np.unique(np.clip(((bearings+math.pi)/(2*math.pi)*72).astype(int),0,71))
    bins=np.clip(((angles+math.pi)/(2*math.pi)*72).astype(int),0,71)
    counts=np.bincount(bins,minlength=72);good=np.bincount(bins,weights=usable,minlength=72)
    if np.any(counts[required]==0) or np.any(good[required]<.8*counts[required]):return math.nan
    valid=np.isfinite(ranges)&(ranges>=.03)
    x=ranges[valid]*np.cos(angles[valid])+.075;y=ranges[valid]*np.sin(angles[valid])
    # The scanner can see the robot's own supports. Mask only its current body.
    outside=(abs(x)>.1032)|(abs(y)>.0773)
    x=x[outside];y=y[outside]
    dx=x[None,:]-cx[:,None];dy=y[None,:]-cy[:,None]
    body_x=c[:,None]*dx+s[:,None]*dy;body_y=-s[:,None]*dx+c[:,None]*dy
    hit=np.any((abs(body_x)<=.105)&(abs(body_y)<=.087),axis=1)
    return max(0.,float(distance[np.flatnonzero(hit)[0]])-.005) if hit.any() else math.inf


class Driver:
    def __init__(self, route, signals, speed=.20):
        if not 0<speed<=.20:raise ValueError('speed must be in (0, 0.20]')
        self.route=route;self.signals=sorted(signals,key=lambda s:s['stop_s'])
        self.speed=speed;self.progress=0.;self.state='WAIT_SENSORS'
        self.signal_index=0;self.green_frames=0;self.last_frame=None
        self.last_lane_time=None;self.last_time=None;self.reason=''
        self.committed=False
        self.entry_authorized=False
        self.green_cycle_ready=False
        self.last_red_observation=-math.inf
        self.clock_fault=False
        self.recovery_pending=False
        self.fault_reason=''

    @property
    def segment(self):return getattr(self.route,'active',None)

    def tracking_curvature(self,pose,progress=None):
        s=self.progress if progress is None else progress
        target=self.route.target(s+(self.segment.lookahead if self.segment else .15))
        alpha=wrap(math.atan2(target[1]-pose[1],target[0]-pose[0])-pose[2])
        return 2*math.sin(alpha)/max(.05,math.dist(target,pose[:2]))

    def resume(self,pose):
        if self.clock_fault:return False
        if not all(math.isfinite(v) for v in pose):return False
        try:
            if hasattr(self.route,'resume'):s=self.route.resume(pose)
            else:
                s,e=self.route.project(*pose[:2])
                s+=round((self.progress-s)/self.route.length)*self.route.length
                if abs(e)>.065 or abs(wrap(pose[2]-self.route.heading(s % self.route.length)))>.8:return False
            self.progress=max(0.,s);self.fault_reason=''
            self.signal_index=next((i for i,sig in enumerate(self.signals) if sig.get('exit_s',sig['stop_s']+.38)>s),len(self.signals))
            self.committed=bool(self.signal_index<len(self.signals) and s+.1032>=self.signals[self.signal_index].get('line_s',self.signals[self.signal_index]['stop_s']+.1382))
            self.green_frames=0;self.entry_authorized=False;self.green_cycle_ready=False;self.last_lane_time=None;self.recovery_pending=False
            return True
        except ValueError:return False

    def stop(self,state,reason):
        self.state=state;self.reason=reason
        if state=='FAULT_STOP':self.fault_reason=reason
        return 0.,0.

    def step(self, pose, obs, now, enabled=True):
        if self.state=='COMPLETE':return 0.,0.
        if self.clock_fault:return self.stop('FAULT_STOP','clock reset; restart mission')
        if not all(math.isfinite(v) for v in (*pose,now)):
            return self.stop('FAULT_STOP','invalid pose')
        if self.last_time is not None and now<self.last_time:
            self.green_frames=0
            self.clock_fault=True
            return self.stop('FAULT_STOP','clock reset; restart mission')
        self.last_time=now
        if not enabled:
            self.recovery_pending=True
            self.entry_authorized=False
            self.green_frames=0;self.last_frame=obs.frame
            if self.signal_index<len(self.signals):
                physical_s,_=self.route.project(pose[0],pose[1])
                sig=self.signals[self.signal_index]
                if physical_s<sig.get('line_s',sig['stop_s']+.1382)-.1032:
                    self.committed=False
            return self.stop('MANUAL','automatic input disabled')
        if not obs.fresh:
            self.green_frames=0
            if now-self.last_red_observation>2.:self.green_cycle_ready=False
            return self.stop('WAIT_SENSORS','sensor missing or stale')
        if self.recovery_pending and not self.resume(pose):
            return self.stop('FAULT_STOP','manual recovery needs current/adjacent route and forward heading')
        if self.fault_reason:return self.stop('FAULT_STOP',self.fault_reason)
        if not math.isfinite(obs.clearance) and obs.clearance!=math.inf:
            return self.stop('FAULT_STOP','invalid clearance')
        if obs.localization_age>1.5:return self.stop('WAIT_SENSORS','localization unavailable beyond 1.5 seconds')
        x,y,yaw=pose
        physical_s,physical_error=self.route.project(x,y,self.progress)
        s,error=physical_s,physical_error
        self.progress=max(self.progress,s)
        seg=self.segment
        if abs(error)>(seg.corridor if seg else .065):return self.stop('FAULT_STOP','outside route corridor')
        if hasattr(self.route,'advance') and self.route.advance(pose,self.progress):
            seg=self.segment
            s,error=self.route.project(x,y,self.progress)
            self.progress=max(self.progress,s)
        if obs.lane_valid:self.last_lane_time=now
        if self.last_lane_time is None:self.last_lane_time=now
        lane_required=seg.lane_required if seg else abs(obs.pitch)<.08
        if not lane_required:self.last_lane_time=now
        if lane_required and now-self.last_lane_time>2.:return self.stop('FAULT_STOP','lane lost on lane-required segment')
        if self.progress>self.route.length-(seg.completion_distance if seg else .045):
            if np.linalg.norm(np.array([x,y])-self.route.target(0))<.08 and abs(wrap(yaw))<math.radians(15):
                return self.stop('COMPLETE','MISSION COMPLETE')
        curvature=self.tracking_curvature(pose,s)
        speed=min(self.speed,seg.speed if seg else self.speed)
        # Curvature limits lateral acceleration; pitch alone never forces crawl.
        speed=min(speed,math.sqrt(.045/max(.01,abs(curvature))))
        if seg is None and abs(curvature)>.6:speed=min(speed,.15)
        if seg and seg.kind=='slope':speed=min(speed,.15 if obs.pitch<0 else .13)
        if lane_required and not obs.lane_valid:speed=min(speed,.09)
        if obs.localization_age>.5:speed=min(speed,.09)
        if seg and self.route.index==len(self.route.segments)-1:
            speed=min(speed,max(.035,(self.route.length-s)*.8))
        self.state={'roundabout':'ROUNDABOUT','parking':'PARKING_AREA','slope':'SLOPE'}.get(seg.kind if seg else '', 'DRIVE');self.reason=''
        if self.signal_index<len(self.signals):
            sig=self.signals[self.signal_index];distance=sig['stop_s']-s
            if obs.stop_distance is not None and distance>0:
                distance=min(distance,max(0.,obs.stop_distance-.035))
            if s>sig.get('exit_s',sig['stop_s']+.38):
                self.signal_index+=1;self.committed=False;self.entry_authorized=False;self.green_cycle_ready=False;self.green_frames=0
            elif distance<.85 and (not seg or self.route.index==sig['segment_index']):
                self.state='APPROACH_SIGNAL'
                if obs.frame!=self.last_frame:
                    self.last_frame=obs.frame
                    self.green_frames=self.green_frames+1 if obs.light_id==sig['id'] and obs.light=='green' else 0
                    if (distance<=.04 and abs(obs.measured_speed)<.01 and obs.light_id==sig['id']
                            and obs.light in ('red','yellow')):
                        self.green_cycle_ready=True;self.last_red_observation=now
                # A demo intersection deliberately waits for an image-observed
                # new green cycle at rest. An already-green approach may be at
                # the very end of green; camera latency cannot reveal time left.
                # Keep the observed transition through a short sensor stop,
                # but never reuse it at the end of a green phase. A stopped
                # vehicle must still receive three new green frames to resume.
                if now-self.last_red_observation>2.:self.green_cycle_ready=False
                green=self.green_frames>=3 and (not sig.get('require_new_green',False) or self.green_cycle_ready)
                line_s=sig.get('line_s',sig['stop_s']+.1382)
                # A fresh pose can arrive after the last green-authorized
                # command has carried the bumper across the line. Preserve
                # that authorization across a sensor stop, but revoke it if a
                # non-green observation arrives while still before the line.
                if s+.1032>=line_s and (green or self.entry_authorized):self.committed=True
                elif s+.1032<line_s:self.entry_authorized=green
                if self.committed:self.state='CROSSING'
                elif green:
                    # stop_s is the center stopping target, not the line itself.
                    if s+.1032>=line_s:
                        self.committed=True;self.state='CROSSING'
                else:
                    speed=min(speed,max(0.,distance*.6))
                    if distance<=.04:
                        reason=(f'waiting for observed red/green cycle at {sig["id"]}' if sig.get('require_new_green',False) and not self.green_cycle_ready and obs.light=='green'
                                else f'confirming green {self.green_frames}/3' if obs.light_id==sig['id'] and obs.light=='green'
                                else f'waiting at {sig["id"]}: {obs.light if obs.light_id==sig["id"] else "unknown"}')
                        return self.stop('WAIT_SIGNAL',reason)
        braking_speed=max(speed,abs(obs.measured_speed))
        stopping=braking_speed*braking_speed/(2*.4)+braking_speed*.35+.035
        if obs.clearance<stopping:return self.stop('OBSTACLE_STOP','obstacle inside braking distance')
        angular=speed*curvature
        # Image-derived lateral error in metres; bounded correction.
        if lane_required and obs.lane_valid and abs(curvature)<.6 and abs(obs.pitch)<.05:
            angular+=max(-.025,min(.025,-.5*obs.lane_error))
        return speed,max(-.8,min(.8,angular))


class YawController:
    """IMU yaw-rate PI compensates four-wheel skid-steering losses."""
    def __init__(self):self.integral=0.

    def step(self, desired, measured, dt, stopped=False):
        if stopped or not all(math.isfinite(v) for v in (desired,measured,dt)):
            self.integral=0.;return 0.
        error=desired-measured
        self.integral=max(-.3,min(.3,self.integral+error*min(.1,max(0.,dt))))
        return max(-1.2,min(1.2,2.*desired+.6*error+1.2*self.integral))
