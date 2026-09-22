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
        limit=.15 if self.enabled else .25
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
    def __init__(self, route, signals, speed=.15):
        if not 0<speed<=.15:raise ValueError('speed must be in (0, 0.15]')
        self.route=route;self.signals=sorted(signals,key=lambda s:s['stop_s'])
        self.speed=speed;self.progress=0.;self.state='WAIT_SENSORS'
        self.signal_index=0;self.green_frames=0;self.last_frame=None
        self.last_lane_time=None;self.last_time=None;self.reason=''
        self.committed=False
        self.clock_fault=False

    def stop(self,state,reason):
        self.state=state;self.reason=reason
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
        physical_s,physical_error=self.route.project(pose[0],pose[1])
        physical_s+=round((self.progress-physical_s)/self.route.length)*self.route.length
        if not enabled:
            self.green_frames=0;self.last_frame=obs.frame
            if self.signal_index<len(self.signals):
                sig=self.signals[self.signal_index]
                if physical_s<sig.get('line_s',sig['stop_s']+.1382)-.1032:
                    self.committed=False
            return self.stop('MANUAL','automatic input disabled')
        if not obs.fresh:
            self.green_frames=0
            return self.stop('WAIT_SENSORS','sensor missing or stale')
        x,y,yaw=pose
        if not self.progress-.15<=physical_s<=self.progress+.6:
            return self.stop('FAULT_STOP','position changed outside tracking window; restart mission')
        s,error=physical_s,physical_error
        self.progress=max(self.progress,s)
        if abs(error)>.065:return self.stop('FAULT_STOP','outside lane corridor')
        if obs.lane_valid:self.last_lane_time=now
        if self.last_lane_time is None:self.last_lane_time=now
        if now-self.last_lane_time>1.:return self.stop('FAULT_STOP','lane lost')
        if self.progress>self.route.length-.045:
            if np.linalg.norm(np.array([x,y])-self.route.target(0))<.08 and abs(wrap(yaw))<math.radians(15):
                return self.stop('COMPLETE','one lap finished')
        target=self.route.target(s+.15)
        alpha=wrap(math.atan2(target[1]-y,target[0]-x)-yaw)
        curvature=2*math.sin(alpha)/max(.05,math.dist(target,(x,y)))
        speed=min(self.speed,.09 if abs(curvature)>.6 or abs(obs.pitch)>.08 else self.speed)
        if not obs.lane_valid:speed=min(speed,.05)
        self.state='DRIVE';self.reason=''
        if self.signal_index<len(self.signals):
            sig=self.signals[self.signal_index];distance=sig['stop_s']-s
            if obs.stop_distance is not None and distance>0:
                distance=min(distance,max(0.,obs.stop_distance-.035))
            if distance<-.38:
                self.signal_index+=1;self.committed=False;self.green_frames=0
            elif distance<.85:
                self.state='APPROACH_SIGNAL'
                if obs.frame!=self.last_frame:
                    self.last_frame=obs.frame
                    self.green_frames=self.green_frames+1 if obs.light_id==sig['id'] and obs.light=='green' else 0
                green=self.green_frames>=3
                if self.committed:self.state='CROSSING'
                elif green:
                    # stop_s is the center stopping target, not the line itself.
                    line_s=sig.get('line_s',sig['stop_s']+.1382)
                    if s+.1032>=line_s:
                        self.committed=True;self.state='CROSSING'
                else:
                    speed=min(speed,max(0.,distance*.6))
                    if distance<=.04:
                        reason=(f'confirming green {self.green_frames}/3' if obs.light_id==sig['id'] and obs.light=='green'
                                else f'waiting at {sig["id"]}: {obs.light if obs.light_id==sig["id"] else "unknown"}')
                        return self.stop('WAIT_SIGNAL',reason)
        stopping=speed*speed/(2*.4)+speed*.35+.035
        if not math.isfinite(obs.clearance) and obs.clearance!=math.inf:
            return self.stop('FAULT_STOP','invalid clearance')
        if obs.clearance<stopping:return self.stop('OBSTACLE_STOP','obstacle inside braking distance')
        angular=speed*curvature
        # Image-derived lateral error in metres; bounded correction.
        if obs.lane_valid and abs(curvature)<.6 and abs(obs.pitch)<.05:
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
