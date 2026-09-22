"""Image-only lane and signal extraction, with explicit invalid results."""
import cv2
import numpy as np


def detect_light(image, roi=None):
    h,w=image.shape[:2]
    if roi is None: roi=(0,0,w,int(h*.7))
    x,y,rw,rh=map(int,roi)
    x0,y0=max(0,x),max(0,y);x1,y1=min(w,x+rw),min(h,y+rh)
    if x1<=x0 or y1<=y0:return 'unknown',0.
    patch=image[y0:y1,x0:x1]
    hsv=cv2.cvtColor(patch,cv2.COLOR_BGR2HSV)
    candidates=[]
    for color,bounds in [('red',[(0,10),(170,179)]),('yellow',[(16,38)]),('green',[(40,90)])]:
        mask=np.zeros(hsv.shape[:2],np.uint8)
        for lo,hi in bounds:mask|=cv2.inRange(hsv,(lo,120,125),(hi,255,255))
        for c in cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[0]:
            a=cv2.contourArea(c);bx,by,bw,bh=cv2.boundingRect(c)
            if not (4<=a<=700 and .5<=bw/max(1,bh)<=2 and bw<60 and bh<60):continue
            # A bright round bulb must sit against a dark lamp housing.
            pad=max(3,int(max(bw,bh)*.6))
            neighborhood=hsv[max(0,by-pad):min(len(hsv),by+bh+pad),max(0,bx-pad):min(hsv.shape[1],bx+bw+pad),2]
            dark=float(np.mean(neighborhood<100))
            if dark<.25:continue
            fill=a/(bw*bh)
            if fill<.35:continue
            candidates.append((a*dark,color))
    if not candidates:return 'unknown',0.
    candidates.sort(reverse=True)
    # Red wins over simultaneous plausible green: ambiguous observation cannot release.
    colors={c for _,c in candidates}
    color='red' if 'red' in colors else ('yellow' if 'yellow' in colors else 'green')
    return color,min(1.,max(a for a,c in candidates if c==color)/20)


def detect_lane(image):
    """Paired white edge rows. Error is a bounded lateral estimate in metres."""
    h,w=image.shape[:2];hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,(0,0,145),(179,85,255))
    centers=[]
    for row in np.linspace(int(h*.59),int(h*.88),15,dtype=int):
        xs=np.flatnonzero(mask[row])
        if len(xs)==0 or len(xs)>w*.40:continue
        groups=np.split(xs,np.flatnonzero(np.diff(xs)>2)+1)
        edges=[float(np.mean(g)) for g in groups if 1<=len(g)<=18]
        left=[x for x in edges if x<w*.48];right=[x for x in edges if x>w*.52]
        if left and right:
            l=max(left);r=min(right)
            if r-l>w*.14:centers.append(((l+r)/2-w/2)*.30/(r-l))
    if len(centers)<3:
        # The sandbox has unpainted straights. Its asphalt is a low-saturation
        # blue-grey; require a connected road interval containing image centre.
        road=cv2.inRange(hsv,(85,12,55),(115,60,155))
        road=cv2.morphologyEx(road,cv2.MORPH_CLOSE,np.ones((3,5),np.uint8))
        for row in np.linspace(int(h*.52),int(h*.68),20,dtype=int):
            if not road[row,w//2]:continue
            xs=np.flatnonzero(road[row])
            for group in np.split(xs,np.flatnonzero(np.diff(xs)>1)+1):
                if group[0]<=w//2<=group[-1]:
                    l,r=group[0],group[-1]
                    if l>w*.02 and r<w*.98 and r-l>w*.12:
                        centers.append(((l+r)/2-w/2)*.30/(r-l))
        if len(centers)<3:
            # At a crest or tight turn only one boundary is inside the low
            # camera's view. Confirm visible road, but do not invent an error.
            bottom=road[int(h*.68):,int(w*.25):int(w*.75)]
            top=road[:int(h*.45)]
            visible=np.mean(bottom>0)>.70 and np.mean(top>0)<.90 and float(image.std())>20
            return 0.,bool(visible)
    return float(np.clip(np.median(centers),-.08,.08)),True


def signal_roi(pose, signal, camera_k, pitch=0., height=.036):
    """Project a known head position; never consumes its configured color."""
    import math
    x,y,yaw=pose;dx=signal['head'][0]-x;dy=signal['head'][1]-y
    ahead=math.cos(yaw)*dx+math.sin(yaw)*dy-.109
    left=-math.sin(yaw)*dx+math.cos(yaw)*dy
    if ahead<=.08 or ahead>1.5:return None
    up=signal['head'][2]-height
    z=math.cos(pitch)*ahead-math.sin(pitch)*up
    vertical=math.sin(pitch)*ahead+math.cos(pitch)*up
    fx,fy,cx,cy=camera_k
    u=cx-fx*left/max(.05,z);v=cy-fy*vertical/max(.05,z)
    rw=max(24,fx*.24/ahead);rh=max(20,fy*.09/ahead)
    return int(u-rw/2),int(v-rh/2),int(rw),int(rh)


def detect_stop_line(image, camera_k, pitch, expected_distance):
    """Front-bumper distance to a mapped horizontal white stop line on flat road."""
    import math
    if not 0<expected_distance<.85 or abs(pitch)>.08:return None
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,(0,0,170),(179,65,255))
    h,w=mask.shape;mask[:int(h*.50)]=0
    lines=cv2.HoughLinesP(mask,1,np.pi/180,35,minLineLength=60,maxLineGap=12)
    if lines is None:return None
    fx,fy,cx,cy=camera_k;horizon=cy-fy*math.tan(pitch);candidates=[]
    for x1,y1,x2,y2 in lines[:,0]:
        if abs(y2-y1)>max(3,abs(x2-x1)*.08):continue
        if min(x1,x2)>cx or max(x1,x2)<cx:continue
        row=(y1+y2)/2
        if row-horizon<5:continue
        distance=.036*fy/(row-horizon)+.004
        if abs(distance-expected_distance)<.10:candidates.append(distance)
    return min(candidates,key=lambda d:abs(d-expected_distance)) if candidates else None
