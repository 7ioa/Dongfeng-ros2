#!/usr/bin/env python3
"""Compare independently validated routes and export a reproducible figure."""
import argparse
import json
from pathlib import Path
import numpy as np
from dongfeng_autonomy.evaluation import evaluate_performance


def read_run(path):
    root=Path(path)
    summary=json.loads((root/'summary.json').read_text())
    truth=json.loads((root/'truth.json').read_text());states=json.loads((root/'status.json').read_text())
    return {**summary,**evaluate_performance(truth,states)},truth


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--baseline',required=True);parser.add_argument('--fast',required=True)
    parser.add_argument('--output',default='reports/autonomy/speed_comparison')
    args=parser.parse_args();root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    baseline,bt=read_run(args.baseline);fast,ft=read_run(args.fast)
    reductions={key:100.*(1.-fast[key]/baseline[key]) for key in
        ('wall_seconds','mission_wall_seconds','mission_sim_seconds')}
    reductions['driving_sim_seconds']=100.*(1.-fast['phase_times']['driving']['sim_seconds']/baseline['phase_times']['driving']['sim_seconds'])
    result=dict(baseline=baseline,fast=fast,reduction_percent=reductions,
                both_pass=bool(baseline['lap_pass'] and fast['lap_pass']))
    (root/'comparison.json').write_text(json.dumps(result,indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,(speed,times)=plt.subplots(2,1,figsize=(11,7),layout='constrained')
    for label,truth,color in (('Normal',bt,'#536c8c'),('Fast',ft,'#e5792e')):
        t=np.array([s['t'] for s in truth]);xy=np.array([[s['x'],s['y']] for s in truth]);progress=np.array([s['progress'] for s in truth])
        v=np.linalg.norm(np.diff(xy,axis=0),axis=1)/np.diff(t)
        speed.plot(progress[1:],v,label=label,color=color,linewidth=1.,alpha=.85)
    speed.set(xlabel='Route progress (m)',ylabel='Actual speed (m/s)',ylim=(0.,.45))
    speed.grid(alpha=.2);speed.legend()
    colors=['#419b78','#e6ad48','#a897bd','#aeb6c0'];bottom=np.zeros(2)
    for phase,color in zip(('driving','signal_wait','sensor_wait','other_stop'),colors):
        values=[r['phase_times'][phase]['wall_seconds'] for r in (baseline,fast)]
        times.barh(['Normal','Fast'],values,left=bottom,label=phase.replace('_',' '),color=color)
        bottom+=values
    times.set(xlabel='Mission wall time (s), from first movement to COMPLETE')
    times.legend(ncol=4,loc='upper center',bbox_to_anchor=(.5,-.16));times.grid(axis='x',alpha=.2)
    fig.savefig(root/'speed-comparison.png',dpi=160)
    print(json.dumps(dict(both_pass=result['both_pass'],reduction_percent=reductions)),flush=True)
    return 0 if result['both_pass'] else 1

if __name__=='__main__':raise SystemExit(main())
