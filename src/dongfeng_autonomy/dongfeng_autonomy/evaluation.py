"""Independent truth-only lap geometry; never imported by the driving node."""
import math
import numpy as np
from .route import wrap


def evaluate_lap(truth, complete, route):
    result = dict(complete=bool(complete), lap_pass=False, checkpoints=[],
                  trajectory_valid=False, road_containment_pass=False,
                  truth_progress=0., stationary_seconds=0., return_distance=None,
                  return_heading_error=None, max_center_error=None,
                  max_footprint_lateral_extent=None)
    if len(truth)<2:
        return result
    data=np.array([[p[k] for k in ('t','x','y','yaw')] for p in truth],dtype=float)
    if not np.isfinite(data).all():return result
    ts,xy,yaws=data[:,0],data[:,1:3],data[:,3]
    dt=np.diff(ts);ds=np.linalg.norm(np.diff(xy,axis=0),axis=1)
    valid=bool(np.all((dt>0)&(dt<=.5)) and np.all(ds<=.30*dt+.005))
    # Reject missing starts, jumps, reverse laps and off-road shortcuts.
    valid=valid and np.linalg.norm(xy[0]-route.target(0))<.08
    progress=[];errors=[];footprint=0.
    # Sample the boundary at <= 2 cm: a side midpoint can cut an inner bend
    # even while all four corners remain within the road.
    edge_x=np.linspace(-.1032,.1032,12);edge_y=np.linspace(-.0773,.0773,9)
    boundary=[(x,y) for x in edge_x for y in (-.0773,.0773)]
    boundary += [(x,y) for x in (-.1032,.1032) for y in edge_y]
    for p in truth:
        s,e=route.project(p['x'],p['y'])
        progress.append(s);errors.append(abs(e))
        c,sn=math.cos(p['yaw']),math.sin(p['yaw'])
        for dx,dy in boundary:
            _,err=route.project(p['x']+c*dx-sn*dy,p['y']+sn*dx+c*dy)
            footprint=max(footprint,abs(err))
    unwrapped=np.unwrap(np.asarray(progress)*2*math.pi/route.length)*route.length/(2*math.pi)
    valid=valid and bool(np.all(abs(np.diff(unwrapped))<=.30*dt+.01))
    # Midpoints of the four actual perimeter bends, in required order.
    r=.65/math.sqrt(2)
    gates=[('B',(2.46+r,.84-r)),('C',(2.46+r,4.56+r)),
           ('D',(.84-r,4.56+r)),('A',(.84-r,.84-r))]
    checkpoints=[];cursor=0
    for p,s in zip(xy,unwrapped):
        if cursor<len(gates):
            name,point=gates[cursor]
            gate_s=route.project(*point)[0]
            if abs(s-gate_s)<.07 and np.linalg.norm(p-point)<.08:
                checkpoints.append(name);cursor+=1
    # The entire final window must stay within 5 mm / 3 degrees of its end pose.
    stopped=0.
    for i in range(len(ts)-2,-1,-1):
        if np.linalg.norm(xy[i]-xy[-1])>.005 or abs(wrap(yaws[i]-yaws[-1]))>math.radians(3):break
        if ts[i+1]-ts[i]>.5 or ts[i+1]<=ts[i]:break
        stopped=float(ts[-1]-ts[i])
    result.update(trajectory_valid=bool(valid),checkpoints=checkpoints,
                  truth_progress=float(unwrapped[-1]-unwrapped[0]),
                  stationary_seconds=stopped,return_distance=float(np.linalg.norm(xy[-1]-route.target(0))),
                  return_heading_error=abs(wrap(float(yaws[-1]))),
                  max_center_error=max(errors),max_footprint_lateral_extent=footprint,
                  road_containment_pass=footprint<=.15)
    result['lap_pass']=bool(complete and valid and checkpoints==['B','C','D','A']
        and route.length-.08<=result['truth_progress']<=route.length+.10
        and result['road_containment_pass'] and result['return_distance']<.08
        and result['return_heading_error']<math.radians(15) and stopped>=2.)
    return result


def evaluate_signals(truth, lamps, signals, route):
    """Check physical bumper crossings against acknowledged lamp updates."""
    events=[];red_stops=[]
    if not truth or not lamps:
        return dict(traffic_pass=False,signal_crossings=[],red_stops=[])
    times=np.array([m['t'] for m in lamps])
    positions=np.array([[p['x'],p['y']] for p in truth])
    progress=np.array([route.project(*p)[0] for p in positions])
    for sig in signals:
        colors=[]
        for p in truth:
            index=int(np.searchsorted(times,p['t'],side='right'))-1
            colors.append(lamps[index]['colors'].get(sig['id'],'unknown') if index>=0 and 0<=p['t']-times[index]<=.6 else 'unknown')
        gap=sig['line_s']-progress-.1032
        for i in range(1,len(truth)):
            if gap[i-1]>0>=gap[i] and abs(gap[i-1]-gap[i])<.1:
                events.append(dict(signal=sig['id'],t=truth[i]['t'],color=colors[i],pass_green=colors[i]=='green'))
        anchor=None;longest=0.;stop_end=None
        for i,p in enumerate(truth):
            stopped=colors[i]=='red' and 0<=gap[i]<.12
            if stopped:
                if anchor is None or np.linalg.norm(positions[i]-positions[anchor])>.005:anchor=i
                duration=p['t']-truth[anchor]['t']
                if duration>longest:longest=duration;stop_end=i
            else:anchor=None
        if longest>=1.:
            red_stops.append(dict(signal=sig['id'],seconds=longest,front_gap=float(gap[stop_end])))
    passed=all(any(e['signal']==s['id'] and e['pass_green'] for e in events) for s in signals)
    passed=bool(passed and events and all(e['pass_green'] for e in events) and red_stops)
    return dict(traffic_pass=passed,signal_crossings=events,red_stops=red_stops)


def gui_signal_sample(sample, received_wall):
    """Keep GUI source simulation time and reject delayed transport snapshots."""
    stamp=sample['sim_time'];wall=sample['wall_time']
    if not all(math.isfinite(v) for v in (stamp,wall,received_wall)) or not 0<=received_wall-wall<=.6:
        return None
    bulbs=sample['bulbs'];colors={}
    for i in range(8):
        name=f'signal_{i}'
        lit=[c for c in ('red','yellow','green') if max(bulbs.get(name+'_'+c,[0]))>.5]
        colors[name]=lit[0] if len(lit)==1 else 'unknown'
    return dict(t=stamp,colors=colors,sample_wall=wall,transport_delay=received_wall-wall)
