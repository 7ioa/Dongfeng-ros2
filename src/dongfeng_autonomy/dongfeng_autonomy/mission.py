"""Ordered open route segments. Only the active segment owns projection."""
from dataclasses import dataclass
import json
import math
from pathlib import Path
import numpy as np
from .route import Route, wrap


def geometry(primitives):
    points=[]
    for item in primitives:
        kind=item['type']
        if kind=='line':
            a,b=np.asarray(item['from']),np.asarray(item['to'])
            p=np.linspace(a,b,max(2,int(np.linalg.norm(b-a)/.01)+1))
        elif kind=='arc':
            angles=np.radians(np.linspace(*item['degrees'],max(3,int(abs(item['degrees'][1]-item['degrees'][0])*1.5))))
            radii=np.broadcast_to(item['radius'],(2,))
            p=np.asarray(item['center'])+np.c_[np.cos(angles),np.sin(angles)]*radii
        elif kind=='bezier':
            a,b,c,d=np.asarray(item['points']);t=np.linspace(0,1,160)[:,None]
            p=(1-t)**3*a+3*(1-t)**2*t*b+3*(1-t)*t*t*c+t**3*d
        else:raise ValueError('Unknown geometry: '+kind)
        if points and np.linalg.norm(points[-1]-p[0])>.002:raise ValueError('Disconnected geometry')
        points.extend(p if not points else p[1:])
    return points


@dataclass
class RouteSegment:
    name: str
    route: Route
    speed: float = .16
    lane_required: bool = False
    kind: str = 'road'
    corridor: float = .065
    lookahead: float = .15
    completion_distance: float = .045
    next_segment: str | None = None
    signal: dict | None = None
    surface: str = 'flat'


class Mission:
    def __init__(self, name, segments, parking_bounds=None):
        if not segments:raise ValueError('Empty mission')
        self.name=name;self.segments=segments;self.index=0
        self.offsets=np.r_[0.,np.cumsum([s.route.length for s in segments])]
        self.length=float(self.offsets[-1]);self.closed=False
        self.parking_bounds=None
        if parking_bounds is not None:
            bounds=np.asarray(parking_bounds,dtype=float)
            if (bounds.shape!=(4,) or not np.isfinite(bounds).all()
                    or bounds[0]>=bounds[2] or bounds[1]>=bounds[3]):
                raise ValueError('Invalid parking goal bounds')
            self.parking_bounds=tuple(bounds.tolist())
        self.signals=[]
        names=[s.name for s in segments]
        if len(set(names))!=len(names):raise ValueError('Duplicate segment name')
        for i,seg in enumerate(segments):
            if not 0<seg.speed<=.20 or not .02<=seg.corridor<=.15:raise ValueError('Unsafe segment limits')
            if not .05<=seg.lookahead<=.25 or not 0<seg.completion_distance<=.08:raise ValueError('Invalid tracking/completion distance')
            if seg.kind not in ('road','intersection','roundabout','parking','slope') or seg.surface not in ('flat','yard','perimeter'):
                raise ValueError('Unknown segment kind or surface')
            expected=names[i+1] if i+1<len(names) else None
            if seg.next_segment!=expected:raise ValueError('Mission must be an ordered, finite sequence')
            if i and math.dist(segments[i-1].route.target(segments[i-1].route.length),seg.route.target(0))>.002:
                raise ValueError('Disconnected segments: '+seg.name)
            if seg.signal:
                sig=dict(seg.signal)
                local,error=seg.route.project(*sig.pop('line'))
                if abs(error)>.025:raise ValueError('Stop line outside segment')
                sig.update(line_s=float(self.offsets[i]+local),stop_s=float(self.offsets[i]+local-.1382),segment_index=i)
                sig['exit_s']=float(self.offsets[i]+sig.pop('exit_progress',seg.route.length))
                if not self.offsets[i]<=sig['stop_s']<sig['line_s']<sig['exit_s']<=self.offsets[i+1]:
                    raise ValueError('Signal stop/line/exit must be ordered within its segment')
                self.signals.append(sig)

    @classmethod
    def load(cls, path, signals):
        cfg=json.loads(Path(path).read_text());heads={s['id']:s for s in signals}
        segments=[]
        for item in cfg['segments']:
            item=dict(item);primitives=item.pop('geometry')
            if item.get('signal'):
                sig=item['signal'];item['signal']={**heads[sig['id']],**sig}
            segments.append(RouteSegment(route=Route(geometry(primitives),closed=False),**item))
        return cls(cfg['name'],segments,parking_bounds=cfg.get('parking_bounds'))

    @property
    def active(self):return self.segments[self.index]

    @property
    def initial_pose(self):
        first=self.segments[0].route
        return np.r_[first.target(0),first.heading(0)]

    def project(self,x,y,previous=None):
        offset=self.offsets[self.index]
        s,e=self.active.route.project(x,y,None if previous is None else previous-offset)
        return float(offset+s),e

    def target(self,s):
        i=min(len(self.segments)-1,max(0,int(np.searchsorted(self.offsets,s,side='right')-1)))
        return self.segments[i].route.target(s-self.offsets[i])

    def advance(self,pose,progress):
        seg=self.active;end=self.offsets[self.index+1]
        if (self.index<len(self.segments)-1 and progress>=end-seg.completion_distance
                and math.dist(pose[:2],seg.route.target(seg.route.length))<min(.035,self.segments[self.index+1].corridor*.7)
                and abs(wrap(pose[2]-seg.route.heading(seg.route.length)))<.7):
            self.index+=1
            return True
        return False

    def resume(self,pose):
        # Prefer the current visit to a road. Only its immediate neighbours may
        # be considered; never search the full mission at crossing/repeat roads.
        for i in (self.index,self.index+1,self.index-1):
            if not 0<=i<len(self.segments):continue
            seg=self.segments[i];s,e=seg.route.project(*pose[:2])
            if abs(e)<=seg.corridor and abs(wrap(pose[2]-seg.route.heading(s)))<.8:
                self.index=i
                return float(self.offsets[i]+s)
        raise ValueError('manual recovery outside current/adjacent segment or wrong heading')

    def status(self,progress):
        return dict(mission=self.name,segment=self.active.name,segment_index=self.index,
                    segment_progress=round(max(0.,progress-self.offsets[self.index]),3),
                    segment_length=round(self.active.route.length,3),
                    mission_progress=round(min(1.,max(0.,progress/self.length)),4))
