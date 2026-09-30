"""Small online lidar submap: scan-to-map translation + occupancy ray updates.

Uses only estimated pose, IMU and real scans. The existing static map provides
global anchors; this local SLAM submap records newly observed parking objects.
"""
import math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree


def outside_robot(local_points):
    """Keep returns outside the same body footprint used by obstacle sensing."""
    p=np.asarray(local_points)
    return (abs(p[:,0])>.1032)|(abs(p[:,1])>.0773)


class ParkingMapper:
    def __init__(self,resolution=.025):
        self.resolution=resolution;self.origin=np.array([0.,3.5])
        self.logodds=np.zeros((72,132),np.float32)
        self.seen=np.zeros_like(self.logodds,dtype=bool)
        self.landmarks=np.empty((0,2));self.scans=0;self.matches=0

    def correction(self,endpoints):
        if len(self.landmarks)<40 or len(endpoints)<25:return np.zeros(2),False
        p=np.array(endpoints,dtype=float,copy=True)[:,:2];total=np.zeros(2);tree=cKDTree(self.landmarks)
        for _ in range(4):
            d,i=tree.query(p);good=d<.045
            if good.sum()<25 or good.mean()<.35:return np.zeros(2),False
            delta=np.median(self.landmarks[i[good]]-p[good],axis=0)
            p+=delta;total+=delta
        good=bool(np.linalg.norm(total)<.025)
        if good:self.matches+=1
        return total,good

    def update(self,origin,endpoints):
        p=np.asarray(endpoints)[:,:2]
        if len(p)<10 or not np.isfinite(p).all():return
        self.scans+=1
        for endpoint in p[::2]:
            n=max(2,int(np.linalg.norm(endpoint-origin)/self.resolution)+1)
            cells=np.floor((np.linspace(origin,endpoint,n)-self.origin)/self.resolution).astype(int)
            good=(cells[:,0]>=0)&(cells[:,0]<132)&(cells[:,1]>=0)&(cells[:,1]<72)
            free=cells[:-1][good[:-1]]
            np.add.at(self.logodds,(free[:,1],free[:,0]),-.3)
            self.seen[free[:,1],free[:,0]]=True
            if good[-1]:
                x,y=cells[-1];self.logodds[y,x]+=.9;self.seen[y,x]=True
        np.clip(self.logodds,-4,4,out=self.logodds)
        merged=np.vstack((self.landmarks,p));_,indices=np.unique(np.round(merged/.02).astype(int),axis=0,return_index=True)
        self.landmarks=merged[indices][-10000:]

    def occupancy(self):
        grid=np.round(100/(1+np.exp(-self.logodds))).astype(np.int8)
        grid[~self.seen]=-1
        return grid

    def save(self,directory):
        root=Path(directory);root.mkdir(parents=True,exist_ok=True)
        g=self.occupancy();pixels=np.where(g<0,205,np.rint(2.54*(100-g))).astype(np.uint8)[::-1]
        # Launch teardown can deliver a second SIGINT during export. Replace
        # completed files atomically so interruption cannot truncate a good map.
        pgm=root/'parking.pgm.tmp'
        pgm.write_bytes(f'P5\n132 72\n255\n'.encode()+pixels.tobytes());pgm.replace(root/'parking.pgm')
        yaml=root/'parking.yaml.tmp'
        yaml.write_text(f'image: parking.pgm\nresolution: {self.resolution}\norigin: [0.0, 3.5, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n');yaml.replace(root/'parking.yaml')
        archive=root/'parking_slam.npz.tmp'
        with archive.open('wb') as stream:
            np.savez_compressed(stream,logodds=self.logodds,seen=self.seen,landmarks=self.landmarks,scans=self.scans,matches=self.matches)
        archive.replace(root/'parking_slam.npz')
