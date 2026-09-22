"""Source-stamp freshness, independently testable without ROS."""
import math


class Freshness:
    def __init__(self, required, timeout=.5):
        self.required=tuple(required);self.timeout=timeout;self.received={};self.stamps={}

    def update(self,key,stamp,wall):
        if not math.isfinite(stamp) or not math.isfinite(wall) or self.stamps.get(key)==stamp:return False
        self.stamps[key]=stamp;self.received[key]=wall;return True

    def ready(self,now,wall):
        return all(0<=wall-self.received.get(k,-math.inf)<self.timeout and -.1<=now-self.stamps.get(k,-math.inf)<self.timeout for k in self.required)


def orientation_rpy(quaternion, available=True):
    """Reject missing/corrupt orientations; normalize harmless roundoff."""
    if not available or len(quaternion)!=4 or not all(math.isfinite(v) for v in quaternion):return None
    norm=math.sqrt(sum(v*v for v in quaternion))
    if not .95<=norm<=1.05:return None
    x,y,z,w=(v/norm for v in quaternion)
    return (math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
            math.asin(max(-1.,min(1.,2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))
