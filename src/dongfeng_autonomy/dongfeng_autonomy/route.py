"""Arc-length route tracking in the map frame, independent of ROS."""
import math
import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Route:
    def __init__(self, points, closed=True):
        self.points = np.asarray(points, dtype=float)
        if self.points.ndim != 2 or self.points.shape[1] != 2 or len(self.points) < 2 or not np.isfinite(self.points).all():
            raise ValueError('Route needs finite 2D points')
        self.closed = closed
        self.starts = self.points if closed else self.points[:-1]
        self.delta = (np.roll(self.points, -1, axis=0) if closed else self.points[1:]) - self.starts
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
        s = progress % self.length if self.closed else np.clip(progress, 0., self.length)
        i = min(np.searchsorted(self.s,s,side='right')-1,len(self.delta)-1)
        return self.starts[i]+self.delta[i]*(s-self.s[i])/self.ds[i]

    def heading(self, progress):
        a, b = self.target(max(0., progress-.005)), self.target(min(self.length-.000001, progress+.005))
        return math.atan2(b[1]-a[1], b[0]-a[0])

    def project(self, x, y, previous=None):
        p=np.array([x,y])
        t=np.clip(np.sum((p-self.starts)*self.delta,axis=1)/self.ds**2,0,1)
        closest=self.starts+t[:,None]*self.delta
        dist=np.linalg.norm(closest-p,axis=1)
        progress=self.s[:-1]+t*self.ds
        if previous is not None:
            if self.closed:progress += np.round((previous-progress)/self.length)*self.length
            dist=np.where((progress>=previous-.15)&(progress<=previous+.6),dist,np.inf)
        if not np.isfinite(dist).any():return float(previous or 0.), math.inf
        i=int(np.argmin(dist))
        cross=self.delta[i,0]*(p-closest[i])[1]-self.delta[i,1]*(p-closest[i])[0]
        # Euclidean distance also detects overshooting an open endpoint.
        signed=math.copysign(float(dist[i]), cross)
        return float(progress[i]),float(signed)
