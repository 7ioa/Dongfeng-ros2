"""Independent truth-only lap geometry; never imported by the driving node."""
import math
import numpy as np
from .route import wrap


def evaluate_mission(truth,complete,mission,max_speed=.30):
    """Truth progress is produced by a separate, ordered Mission tracker."""
    if len(truth)<2:return dict(lap_pass=False,trajectory_valid=False)
    data=np.array([[p[k] for k in ('t','x','y','yaw','progress')] for p in truth])
    if not np.isfinite(data).all():
        return dict(lap_pass=False,trajectory_valid=False,parking_goal_pass=False,parking_footprint=[])
    dt=np.diff(data[:,0]);ds=np.linalg.norm(np.diff(data[:,1:3],axis=0),axis=1)
    valid=bool(np.all((dt>0)&(dt<.6)) and np.all(ds<=max_speed*dt+.005))
    indices=[p['segment_index'] for p in truth]
    visits=list(dict.fromkeys(indices))
    corridor=all(abs(p['error'])<=mission.segments[p['segment_index']].corridor for p in truth)
    distance=math.dist(data[-1,1:3],mission.target(0));heading=abs(wrap(data[-1,3]))
    parking_footprint=[];parking_goal_pass=True
    if mission.parking_bounds is not None:
        x,y,yaw=data[-1,1:4];c,s=math.cos(yaw),math.sin(yaw)
        parking_footprint=np.array([[x+c*dx-s*dy,y+s*dx+c*dy]
            for dx in (-.1032,.1032) for dy in (-.0773,.0773)])
        xmin,ymin,xmax,ymax=mission.parking_bounds
        # The wheels and chassis must fit inside the paint with 5 mm clearance.
        parking_goal_pass=bool(np.all(parking_footprint>=[xmin+.005,ymin+.005])
                              and np.all(parking_footprint<=[xmax-.005,ymax-.005]))
        parking_footprint=parking_footprint.tolist()
    stationary=0.
    for i in range(len(truth)-2,-1,-1):
        if np.linalg.norm(data[i,1:3]-data[-1,1:3])>.005 or abs(wrap(data[i,3]-data[-1,3]))>math.radians(3):break
        stationary=data[-1,0]-data[i,0]
    passed=bool(complete and valid and visits==list(range(len(mission.segments))) and corridor
                and distance<.08 and heading<math.radians(15) and stationary>=2.
                and data[0,4]<.08 and data[-1,4]>mission.length-.08 and parking_goal_pass)
    return dict(lap_pass=passed,trajectory_valid=valid,route_corridor_pass=corridor,
                return_distance=distance,return_heading_error=heading,stationary_seconds=stationary,
                truth_progress=float(data[-1,4]),visited_segments=[mission.segments[i].name for i in visits],
                parking_goal_pass=parking_goal_pass,parking_footprint=parking_footprint)


def evaluate_lap(truth, complete, route, max_speed=.30):
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
    valid=bool(np.all((dt>0)&(dt<=.5)) and np.all(ds<=max_speed*dt+.005))
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
    valid=valid and bool(np.all(abs(np.diff(unwrapped))<=max_speed*dt+.01))
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
    progress=np.array([p['progress'] for p in truth]) if hasattr(route,'segments') else np.array([route.project(*p)[0] for p in positions])
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
    required={s['id'] for s in signals if s.get('require_new_green',False)}
    stopped={s['signal'] for s in red_stops}
    passed=bool(passed and events and all(e['pass_green'] for e in events) and required<=stopped)
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


def evaluate_performance(truth, states):
    """Attribute completed mission time; wall and simulation clocks stay separate."""
    if len(truth)<2 or len(states)<2:return {}
    begin=next((i for i,s in enumerate(states) if s['command'][0]>.01),None)
    end=next((i for i,s in enumerate(states) if s['state']=='COMPLETE'),len(states)-1)
    if begin is None or end<=begin:return {}
    samples=states[begin:end+1]
    phases={k:dict(sim_seconds=0.,wall_seconds=0.) for k in ('driving','signal_wait','sensor_wait','other_stop')}
    segments={}
    for a,b in zip(samples,samples[1:]):
        sim=max(0.,b['sim_time']-a['sim_time']);wall=max(0.,b['wall_time']-a['wall_time'])
        phase='signal_wait' if a['state']=='WAIT_SIGNAL' else ('sensor_wait' if a['state']=='WAIT_SENSORS' else ('driving' if a['command'][0]>0. else 'other_stop'))
        phases[phase]['sim_seconds']+=sim;phases[phase]['wall_seconds']+=wall
        segment=segments.setdefault(a.get('segment','perimeter'),dict(sim_seconds=0.,wall_seconds=0.,max_path_error=0.,max_localization_error=0.))
        segment['sim_seconds']+=sim;segment['wall_seconds']+=wall
    t=np.array([p['t'] for p in truth]);xy=np.array([[p['x'],p['y']] for p in truth])
    dt=np.diff(t);ds=np.linalg.norm(np.diff(xy,axis=0),axis=1)
    speed=ds/np.maximum(dt,1e-9)
    mask=(t[1:]>=samples[0]['sim_time'])&(t[1:]<=samples[-1]['sim_time'])&(dt>0.)
    localization=[]
    for sample in samples:
        i=int(np.searchsorted(t,sample['sim_time']));i=min(len(t)-1,max(0,i))
        if i and abs(t[i-1]-sample['sim_time'])<abs(t[i]-sample['sim_time']):i-=1
        if abs(t[i]-sample['sim_time'])>.15:continue
        error=math.dist(sample['pose'][:2],xy[i]);localization.append(error)
        segment=segments.get(sample.get('segment','perimeter'))
        if segment:
            segment['max_localization_error']=max(segment['max_localization_error'],error)
            segment['max_path_error']=max(segment['max_path_error'],abs(truth[i].get('error',0.)))
    sim=samples[-1]['sim_time']-samples[0]['sim_time'];wall=samples[-1]['wall_time']-samples[0]['wall_time']
    moving_time=float(np.sum(dt[mask&(speed>.01)]));distance=float(np.sum(ds[mask]))
    # Estimate physical acceleration over >=0.25 s, avoiding differentiation of
    # sub-millimetre contact jitter at individual physics steps.
    acceleration=[]
    indices=np.flatnonzero(mask)
    for i in indices:
        j=int(np.searchsorted(t,t[i]+.25))
        k=int(np.searchsorted(t,t[i]+.5))
        if k>=len(t) or t[k]>samples[-1]['sim_time']:continue
        v1=math.dist(xy[i],xy[j])/(t[j]-t[i]);v2=math.dist(xy[j],xy[k])/(t[k]-t[j])
        acceleration.append((v2-v1)/((t[k]-t[i])*.5))
    return dict(mission_sim_seconds=sim,mission_wall_seconds=wall,real_time_factor=sim/wall if wall else None,
        phase_times=phases,distance_metres=distance,moving_sim_seconds=moving_time,
        average_speed=distance/sim if sim else None,moving_average_speed=distance/moving_time if moving_time else None,
        max_actual_speed=float(np.max(speed[mask])) if mask.any() else None,
        p95_actual_speed=float(np.percentile(speed[mask],95)) if mask.any() else None,
        max_acceleration=max(acceleration,default=None),max_deceleration=-min(acceleration,default=0.),
        max_localization_error=max(localization,default=None),segment_performance=segments)
