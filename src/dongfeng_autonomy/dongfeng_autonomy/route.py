"""Arc-length route tracking in the map frame, independent of ROS."""
import math
import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Route:
    def __init__(self, points):
        self.points = np.asarray(points, dtype=float)[:, :2]
        if self.points.ndim != 2 or len(self.points) < 4 or not np.isfinite(self.points).all():
            raise ValueError('Route needs finite 2D points')
        self.delta = np.roll(self.points, -1, axis=0) - self.points
        self.ds = np.linalg.norm(self.delta, axis=1)
        if np.any(self.ds < 1e-8):
            raise ValueError('Duplicate route points')
        self.s = np.r_[0., np.cumsum(self.ds)]
        self.length = self.s[-1]

    @classmethod
    def perimeter(cls, width, length, inset, radius, lane_width):
        r = radius - lane_width / 2
        xl, xr = inset + radius, width - inset - radius
        yb, yt = inset + radius, length - inset - radius
        pts = []
        def line(a, b):
            pts.extend(np.linspace(a,b,max(2,int(math.dist(a,b)/.015)),endpoint=False))
        def arc(cx, cy, begin):
            for a in np.linspace(begin,begin+math.pi/2,100,endpoint=False):
                pts.append([cx+r*math.cos(a),cy+r*math.sin(a)])
        start=(width/2,yb-r)
        line(start,(xr,yb-r));arc(xr,yb,-math.pi/2)
        line((xr+r,yb),(xr+r,yt));arc(xr,yt,0)
        line((xr,yt+r),(xl,yt+r));arc(xl,yt,math.pi/2)
        line((xl-r,yt),(xl-r,yb));arc(xl,yb,math.pi)
        line((xl,yb-r),start)
        return cls(pts)

    def target(self, progress):
        s = progress % self.length
        i = min(np.searchsorted(self.s,s,side='right')-1,len(self.points)-1)
        return self.points[i]+self.delta[i]*(s-self.s[i])/self.ds[i]

    def project(self, x, y, previous=None):
        p=np.array([x,y])
        t=np.clip(np.sum((p-self.points)*self.delta,axis=1)/self.ds**2,0,1)
        closest=self.points+t[:,None]*self.delta
        dist=np.linalg.norm(closest-p,axis=1)
        progress=self.s[:-1]+t*self.ds
        if previous is not None:
            progress += np.round((previous-progress)/self.length)*self.length
            dist=np.where((progress>=previous-.15)&(progress<=previous+.6),dist,np.inf)
        i=int(np.argmin(dist))
        signed=np.cross(self.delta[i]/self.ds[i],p-closest[i]).item()
        return float(progress[i]),float(signed)
