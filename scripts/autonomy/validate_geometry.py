#!/usr/bin/env python3
"""Check the complete swept body against the actual SDF collision meshes.

Offline validator only. No simulator truth or mesh collision data is imported by
Autonomy. Allows terrain, road deck and yard contact; rejects scenery overlap.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/dongfeng_autonomy'))
from dongfeng_autonomy.mission import Mission


class CollisionScene:
    def __init__(self,root=ROOT):
        triangles=[];names=[]
        sdf=ET.parse(root/'models/dongfeng_sandbox/model.sdf')
        for collision in sdf.findall('.//collision'):
            name=collision.get('name')
            if name in ('terrain','road_deck','yard'):continue
            uri=collision.findtext('geometry/mesh/uri')
            if not uri:raise ValueError('Unvalidated collision geometry: '+name)
            path=root/'models'/uri.removeprefix('model://')
            vertices=[];faces=[]
            for line in path.read_text().splitlines():
                f=line.split()
                if not f:continue
                if f[0]=='v':vertices.append(list(map(float,f[1:4])))
                elif f[0]=='f':faces.append([int(x.split('/')[0])-1 for x in f[1:4]])
            triangles.extend(np.asarray(vertices)[faces]);names.extend([name]*len(faces))
        self.triangles=np.asarray(triangles);self.names=np.asarray(names)
        self.low=self.triangles.min(axis=1);self.high=self.triangles.max(axis=1)

    def collisions(self,x,y,yaw,ground,margin=.005):
        # Bounds include all four wheels, chassis and lidar housing. Raised
        # gate arms retain their true Z; their XY projection is not a wall.
        half=np.array([.1032+margin,.0773+margin,.0535])
        center=np.array([x,y,ground+.0565]);c,s=math.cos(yaw),math.sin(yaw)
        radius=np.array([abs(c)*half[0]+abs(s)*half[1],abs(s)*half[0]+abs(c)*half[1],half[2]])
        mask=((self.low<=center+radius)&(self.high>=center-radius)).all(axis=1)
        tri=(self.triangles[mask]-center)@np.array([[c,-s,0],[s,c,0],[0,0,1]])
        if not len(tri):return []
        overlap=((tri.min(axis=1)<=half)&(tri.max(axis=1)>=-half)).all(axis=1)
        edges=np.roll(tri,-1,axis=1)-tri
        axes=[np.cross(edges[:,0],edges[:,1])]
        for k in range(3):
            for unit in np.eye(3):axes.append(np.cross(edges[:,k],unit))
        for normal in axes:
            projection=(tri*normal[:,None,:]).sum(axis=2);extent=(abs(normal)*half).sum(axis=1)
            overlap &= (projection.min(axis=1)<=extent)&(projection.max(axis=1)>=-extent)
        return np.unique(self.names[mask][overlap]).tolist()


def check_mission(mission,scene,road):
    result=[]
    for seg in mission.segments:
        hits=[]
        for progress in np.arange(0,seg.route.length,.01):
            xy=seg.route.target(progress);yaw=seg.route.heading(progress)
            z=float(road[np.argmin(np.linalg.norm(road[:,:2]-xy,axis=1)),2]) if seg.surface=='perimeter' else (.002 if seg.surface=='yard' else 0.)
            names=scene.collisions(*xy,yaw,z)
            if names:hits.append(dict(progress=round(float(progress),3),xy=xy.tolist(),collisions=names))
        result.append(dict(segment=seg.name,passed=not hits,hits=hits))
    return dict(passed=all(s['passed'] for s in result),margin=.005,step=.01,segments=result)


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',default='reports/autonomy/geometry.json');p.add_argument('--truth');a=p.parse_args()
    config=ROOT/'src/dongfeng_autonomy/config'
    mission=Mission.load(config/'full_demo.json',json.loads((config/'signals.json').read_text()))
    scene=CollisionScene();road=np.load(config/'landmarks.npz')['road']
    result=check_mission(mission,scene,road)
    if a.truth:
        hits=[]
        for sample in json.loads(Path(a.truth).read_text())[::2]:
            collisions=scene.collisions(sample['x'],sample['y'],sample['yaw'],sample['z']-.028,margin=0.)
            if collisions:hits.append(dict(t=sample['t'],collisions=collisions))
        result['truth_collision_free']=not hits;result['truth_collisions']=hits
        result['passed'] &= not hits
    path=Path(a.output);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='segments'}))
    for item in result['segments']:print(item['segment'], 'PASS' if item['passed'] else str(len(item['hits']))+' collisions')
    return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
