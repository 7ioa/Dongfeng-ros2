"""Known-map scan matching using static surface landmarks; no simulator truth."""
import numpy as np
from scipy.spatial import cKDTree


class LandmarkMap:
    def __init__(self, points, normals):
        self.points=np.asarray(points,dtype=float)
        self.normals=np.asarray(normals,dtype=float)
        self.tree=cKDTree(self.points)

    def correction(self, endpoints):
        p=np.asarray(endpoints,dtype=float)
        if len(p)<15 or not np.isfinite(p).all():return np.zeros(2),False
        # Seed ICP with a bounded translation search. Curbs and repeated small
        # surfaces otherwise create a local minimum on the last downhill bend.
        offsets=np.array([(x,y,0.) for x in np.linspace(-.06,.06,7)
                          for y in np.linspace(-.06,.06,7)])
        distance,_=self.tree.query((p[None,:,:]+offsets[:,None,:]).reshape(-1,3))
        scores=np.mean(np.minimum(distance.reshape(len(offsets),len(p)),.05)**2,axis=1)
        seed=offsets[int(np.argmin(scores))]
        p=p+seed;total=seed[:2].copy()
        for _ in range(5):
            distance,index=self.tree.query(p,k=1)
            valid=distance<.12
            if valid.sum()<15:return np.zeros(2),False
            normals=self.normals[index[valid]]
            residual=np.sum(normals*(p[valid]-self.points[index[valid]]),axis=1)
            a=normals[:,:2]
            weights=1./(1.+(residual/.005)**2)
            matrix=a.T@(weights[:,None]*a)
            if np.linalg.eigvalsh(matrix)[0]<.1:return np.zeros(2),False
            delta=np.linalg.solve(matrix+np.eye(2)*.01,-a.T@(weights*residual))
            delta=np.clip(delta,-.04,.04)
            p=p.copy();p[:,:2]+=delta;total+=delta
            if np.linalg.norm(delta)<.0003:break
        return total,True


def rotation(roll,pitch,yaw):
    cr,sr=np.cos(roll),np.sin(roll);cp,sp=np.cos(pitch),np.sin(pitch);cy,sy=np.cos(yaw),np.sin(yaw)
    return np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                     [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
