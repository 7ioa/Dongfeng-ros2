#!/usr/bin/env python3
"""Sample static map vertical surfaces for sensor-only localization."""
from pathlib import Path
import importlib.util
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[2]

def main():
    output=ROOT/'src/dongfeng_autonomy/config/landmarks.npz'
    inputs=list((ROOT/'models/dongfeng_sandbox/meshes').glob('*.obj'))+list((ROOT/'config/scene').glob('*.json'))+[Path(__file__),ROOT/'scripts/scene/generate_scene.py']
    if '--if-needed' in sys.argv and output.exists() and all(p.stat().st_mtime<=output.stat().st_mtime for p in inputs):
        return
    points=[];normals=[]
    for path in sorted((ROOT/'models/dongfeng_sandbox/meshes').glob('*.obj')):
        if path.name.startswith('collision_'):continue
        vertices=[];faces=[]
        for line in path.read_text().splitlines():
            fields=line.split()
            if not fields:continue
            if fields[0]=='v':vertices.append(list(map(float,fields[1:4])))
            elif fields[0]=='f':faces.append([int(v.split('/')[0])-1 for v in fields[1:4]])
        v=np.array(vertices)
        for face in faces:
            tri=v[face];normal=np.cross(tri[1]-tri[0],tri[2]-tri[0]);norm=np.linalg.norm(normal)
            if norm<1e-10:continue
            normal/=norm
            if abs(normal[2])>.7 or tri[:,2].max()<0 or tri[:,2].min()>.45:continue
            steps=max(1,int(np.ceil(max(np.linalg.norm(tri[i]-tri[j]) for i,j in [(0,1),(0,2),(1,2)])/.025)))
            u,w=np.triu_indices(steps+1);a=u/steps;b=(w-u)/steps
            p=tri[0]+a[:,None]*(tri[1]-tri[0])+b[:,None]*(tri[2]-tri[0])
            p=p[(p[:,2]>=0)&(p[:,2]<=.45)]
            points.extend(p);normals.extend([normal]*len(p))
    points=np.asarray(points,dtype=np.float32);normals=np.asarray(normals,dtype=np.float32)
    # Keep distinct face normals at corners but avoid redundant mesh samples.
    key=np.c_[np.round(points/.008),np.round(normals*3)].astype(np.int32)
    _,idx=np.unique(key,axis=0,return_index=True)
    spec=importlib.util.spec_from_file_location('scene_generator',ROOT/'scripts/scene/generate_scene.py')
    scene=importlib.util.module_from_spec(spec);spec.loader.exec_module(scene)
    road=np.array([[p[0],p[1],p[4]] for p in scene.ROAD],dtype=np.float32)
    output=ROOT/'src/dongfeng_autonomy/config/landmarks.npz'
    np.savez_compressed(output,points=points[idx],normals=normals[idx],road=road)
    print('Wrote',output,'landmarks',len(idx))

if __name__=='__main__':main()
