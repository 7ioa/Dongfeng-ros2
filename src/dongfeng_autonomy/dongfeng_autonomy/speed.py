"""Path preview and braking envelopes, using only route and sensor state."""
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import numpy as np
from .route import wrap


def braking_limit(distance, terminal_speed, deceleration, delay):
    """Largest speed satisfying v*delay + (v²-v_end²)/(2*a) <= d."""
    if distance <= 0.:return terminal_speed
    return max(terminal_speed, math.sqrt((deceleration*delay)**2+
        terminal_speed**2+2*deceleration*distance)-deceleration*delay)


@dataclass(frozen=True)
class SpeedProfile:
    max_speed: float = .4
    acceleration: float = .25
    deceleration: float = .25
    lateral_acceleration: float = .045
    planning_delay: float = .25
    obstacle_delay: float = .60
    margin: float = .035
    sample_distance: float = .015
    preview_distance: float = .85
    lookahead_time: float = .35
    max_lookahead: float = .22
    region_limits: dict = field(default_factory=lambda:dict(
        road=.4, intersection=.16, roundabout=.12, parking=.14, slope=.18))
    segment_limits: dict = field(default_factory=dict)

    def __post_init__(self):
        for key in ('max_speed','acceleration','deceleration','lateral_acceleration',
                    'planning_delay','obstacle_delay','margin','sample_distance',
                    'preview_distance','lookahead_time','max_lookahead'):
            value=getattr(self,key)
            if not math.isfinite(value) or value<=0:raise ValueError('Invalid speed profile: '+key)
        if (self.max_speed>.4 or self.acceleration>.6 or self.deceleration>.4
                or self.lateral_acceleration>.045 or self.obstacle_delay<.60
                or self.planning_delay<.25 or self.margin<.035 or self.sample_distance<.005
                or self.max_lookahead>.25 or self.sample_distance>.03
                or self.preview_distance<.65):raise ValueError('Unsafe speed profile')
        for values in (self.region_limits,self.segment_limits):
            if any(not math.isfinite(v) or not 0<v<=.4 for v in values.values()):
                raise ValueError('Invalid regional speed limit')

    @classmethod
    def fast(cls, **kwargs):return cls(**kwargs)

    @classmethod
    def load(cls, path, max_speed=None):
        values=json.loads(Path(path).read_text())
        if max_speed is not None:values['max_speed']=max_speed
        return cls(**values)

    def region_limit(self, segment):
        if segment is None:return self.max_speed
        limit=self.segment_limits.get(segment.name,self.region_limits.get(segment.kind,segment.speed))
        if segment.kind=='parking' and segment.corridor<.05:limit=min(limit,.095)
        return min(self.max_speed,limit)


class SpeedPlanner:
    """A sampled, static path envelope plus live stopping/acceleration limits."""
    def __init__(self, route, profile):
        self.route=route;self.profile=profile
        self.s=np.linspace(0.,route.length,max(2,int(math.ceil(route.length/profile.sample_distance))+1))
        if hasattr(route,'segments'):self.s=np.unique(np.r_[self.s,route.offsets])
        self.curvature=np.array([self.path_curvature(s) for s in self.s])
        self.limits=np.full(len(self.s),profile.max_speed)
        if hasattr(route,'segments'):
            indices=np.clip(np.searchsorted(route.offsets,self.s,side='right')-1,0,len(route.segments)-1)
            self.limits=np.array([profile.region_limit(route.segments[i]) for i in indices])
        self.limits=np.minimum(self.limits,np.sqrt(profile.lateral_acceleration/np.maximum(.01,abs(self.curvature))))
        # Propagate a comfortable braking envelope backwards. Reaction distance
        # is applied at lookup; segment boundaries include a geometric margin.
        self.envelope=self.limits.copy()
        for i in range(len(self.s)-2,-1,-1):
            self.envelope[i]=min(self.envelope[i],math.sqrt(self.envelope[i+1]**2+
                2*profile.deceleration*(self.s[i+1]-self.s[i])))
        if route.closed:
            for i in range(len(self.s)-1,-1,-1):
                j=(i+1)%len(self.s)
                ds=self.s[j]-self.s[i] if j>i else profile.sample_distance
                self.envelope[i]=min(self.envelope[i],math.sqrt(self.envelope[j]**2+2*profile.deceleration*ds))

    def path_curvature(self,s):
        window=.035
        def point(at):
            return self.route.target(at if self.route.closed else min(self.route.length,max(0.,at)))
        a,b,c=point(s-window),point(s),point(s+window)
        u,v=b-a,c-b
        if min(np.linalg.norm(u),np.linalg.norm(v))<1e-6:return 0.
        angle=wrap(math.atan2(v[1],v[0])-math.atan2(u[1],u[0]))
        return angle/max(.001,(np.linalg.norm(u)+np.linalg.norm(v))*.5)

    def limit(self,progress,measured_speed=0.,pose_age=0.):
        p=self.profile
        # Solve reaction-distance dependence conservatively, also looking past
        # the active segment. No mutation of the Mission projection cursor.
        candidates=self.s-progress
        if self.route.closed:candidates %= self.route.length
        mask=(candidates>=0)&(candidates<=p.preview_distance)
        current=progress%self.route.length if self.route.closed else np.clip(progress,0,self.route.length)
        speed=float(np.interp(current,self.s,self.envelope))
        delay_distance=max(speed,abs(measured_speed))*(p.planning_delay+max(0.,pose_age))+p.margin
        if np.any(mask):
            d=np.maximum(0.,candidates[mask]-delay_distance)
            speed=min(speed,float(np.min(np.sqrt(self.limits[mask]**2+2*p.deceleration*d))))
        return speed

    def stop_limit(self,distance,measured_speed=0.,pose_age=0.):
        p=self.profile
        delay=p.planning_delay+max(0.,pose_age)
        distance=max(0.,distance-p.margin)
        measured=abs(measured_speed)
        if measured*delay+measured*measured/(2*.4)>distance:return 0.
        # A stale pose and wheel inertia consume distance before the next command.
        available=max(0.,distance-measured*delay)
        return min(braking_limit(distance,0.,p.deceleration,delay),
                   math.sqrt(2*p.deceleration*available))
