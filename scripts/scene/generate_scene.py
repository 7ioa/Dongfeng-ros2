"""Parametric 1:1 sandbox mesh generator. Requires Python 3.10+ and numpy.

Produces a Gazebo Harmonic static model, OBJ/MTL, binary glTF and an offline viewer.
All geometry comes from the same material/layer buckets. No scanned data is used.
"""
from pathlib import Path
import json, math, random, struct, re, xml.etree.ElementTree as ET
from collections import defaultdict
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = ROOT / 'config/scene'
EXPORT_DIR = ROOT / 'exports/scene'
PREVIEW_DIR = ROOT / 'previews/scene'
REPORT_DIR = ROOT / 'reports/scene'
CFG = json.loads((CONFIG_DIR / 'scene_config.json').read_text(encoding='utf-8'))
FRONT_MARKINGS = json.loads((CONFIG_DIR / 'front_markings.json').read_text(encoding='utf-8'))
B_BUILDINGS = json.loads((CONFIG_DIR / 'b_buildings.json').read_text(encoding='utf-8'))
B_LAKE = json.loads((CONFIG_DIR / 'b_lake.json').read_text(encoding='utf-8'))
PARKING_LEFT = json.loads((CONFIG_DIR / 'parking_left.json').read_text(encoding='utf-8'))
FACTORY_BUILDING = json.loads((CONFIG_DIR / 'factory_building.json').read_text(encoding='utf-8'))
RIGHT_D = json.loads((CONFIG_DIR / 'right_d.json').read_text(encoding='utf-8'))
W, L = CFG['interior_width'], CFG['interior_length']
PARKING_Z = .002  # One horizontal yard surface, independent of the perimeter slopes.
random.seed(CFG['seed'])
TAU = math.tau
PALETTE = {
    'asphalt': '#30373b', 'deck_side': '#606870', 'base': '#d5dcd9',
    'blue_board': '#397fa1', 'acrylic': '#90cbd5', 'white': '#f3f1df',
    'kerb': '#cdc6a8', 'grass': '#7b9942', 'grass_dark': '#627f37',
    'grey_yard': '#999497', 'water': '#168bca', 'paving': '#c5c2ad',
    'glass': '#466471', 'glass_light': '#71929d', 'frame': '#bbcbd0',
    'roof': '#bdc8cb', 'roof_dark': '#475d67', 'red': '#c44837',
    'steel': '#9ba8a8', 'black': '#253237', 'yellow': '#e5b32f',
    'light_green': '#80c48e', 'bark': '#705942', 'leaf0': '#344c2a',
    'leaf1': '#496333', 'leaf2': '#61783c', 'leaf3': '#3e5b32'
}

class Mesh:
    def __init__(self):
        self.v, self.f = [], []
    def add(self, verts, faces):
        k=len(self.v)
        self.v.extend([tuple(float(a) for a in p) for p in verts])
        self.f.extend([tuple(k+int(a) for a in f) for f in faces])
    def quad(self, p):
        self.add(p, [(0,1,2),(0,2,3)])
    def arrays(self):
        v=np.asarray(self.v,dtype=np.float32).reshape(-1,3)
        f=np.asarray(self.f,dtype=np.uint32).reshape(-1,3)
        n=np.zeros_like(v)
        cross=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
        for i in range(3): np.add.at(n,f[:,i],cross)
        length=np.linalg.norm(n,axis=1)
        n/=np.maximum(length,1e-12)[:,None]
        return v,f,n

BUCKETS=defaultdict(Mesh)
COLLISION=defaultdict(Mesh)
def bucket(layer,mat): return BUCKETS[(layer,mat)]

def signed_area(p):
    return sum(p[i][0]*p[(i+1)%len(p)][1]-p[(i+1)%len(p)][0]*p[i][1] for i in range(len(p)))/2

def triangulate(points):
    p=list(points)
    if signed_area(p)<0: raise ValueError('Polygon must be CCW')
    ids=list(range(len(p))); out=[]
    def cross(a,b,c): return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    while len(ids)>3:
        found=False
        for q in range(len(ids)):
            a,b,c=ids[q-1],ids[q],ids[(q+1)%len(ids)]
            if cross(p[a],p[b],p[c])<1e-11: continue
            inside=False
            for j in ids:
                if j in (a,b,c): continue
                if min(cross(p[a],p[b],p[j]),cross(p[b],p[c],p[j]),cross(p[c],p[a],p[j])) >= -1e-10:
                    inside=True; break
            if inside: continue
            out.append((a,b,c)); ids.pop(q); found=True; break
        if not found:
            # Remove a numerically collinear vertex, never fan-fill a concavity.
            for q in range(len(ids)):
                if abs(cross(p[ids[q-1]],p[ids[q]],p[ids[(q+1)%len(ids)]]))<1e-9:
                    ids.pop(q); found=True; break
            if not found: raise ValueError('Cannot triangulate polygon')
    if len(ids)==3: out.append(tuple(ids))
    return out

def poly(points,z,layer,mat,collision=None,thickness=0):
    pts=list(points)
    if signed_area(pts)<0: pts.reverse()
    h=(lambda x,y:z) if not callable(z) else z
    verts=[(x,y,h(x,y)) for x,y in pts]
    faces=triangulate(pts)
    m=bucket(layer,mat); m.add(verts,faces)
    if collision is not None: COLLISION[collision].add(verts,faces)
    if thickness:
        bottom=[(x,y,h(x,y)-thickness) for x,y in pts]
        m.add(bottom,[(c,b,a) for a,b,c in faces])
        if collision is not None: COLLISION[collision].add(bottom,[(c,b,a) for a,b,c in faces])
        for i in range(len(pts)):
            j=(i+1)%len(pts)
            q=[bottom[i],bottom[j],verts[j],verts[i]]
            m.quad(q)
            if collision is not None: COLLISION[collision].quad(q)

def rounded_rect(x0,y0,x1,y1,r,n=12):
    radii=[r]*4 if isinstance(r,(float,int)) else r
    # bottom right, top right, top left, bottom left; CCW
    pts=[]
    for (cx,cy,a),rr in zip([(x1-radii[0],y0+radii[0],-math.pi/2),
                             (x1-radii[1],y1-radii[1],0),
                             (x0+radii[2],y1-radii[2],math.pi/2),
                             (x0+radii[3],y0+radii[3],math.pi)],radii):
        for t in np.linspace(a,a+math.pi/2,n+1): pts.append((cx+rr*math.cos(t),cy+rr*math.sin(t)))
    return pts

def ellipse(cx,cy,rx,ry,n=64,wiggle=0):
    return [(cx+rx*(1+wiggle*math.sin(3*t+.8)+wiggle*.5*math.cos(5*t))*math.cos(t),
             cy+ry*(1+wiggle*math.sin(3*t+.8)+wiggle*.5*math.cos(5*t))*math.sin(t)) for t in np.linspace(0,TAU,n,endpoint=False)]

def offset(p,d):
    # Positive distance offsets a CCW polygon inward.
    p=np.asarray(p,float); result=[]
    for i in range(len(p)):
        a=p[i]-p[i-1]; b=p[(i+1)%len(p)]-p[i]
        a/=np.linalg.norm(a); b/=np.linalg.norm(b)
        na=np.array([-a[1],a[0]]); nb=np.array([-b[1],b[0]])
        result.append(tuple(p[i]+(na+nb)*d/max(1+np.dot(na,nb),.05)))
    return result

def ribbon(p,width,z,layer,mat,closed=False):
    if callable(z):
        # A long marking must follow every terrain grade, not bridge its endpoints.
        sampled=[]
        for i in range(len(p) if closed else len(p)-1):
            a=np.asarray(p[i],float);b=np.asarray(p[(i+1)%len(p)],float)
            count=max(1,math.ceil(float(np.linalg.norm(b-a))/.02))
            sampled.extend([tuple(a+(b-a)*t/count) for t in range(count)])
        if not closed:sampled.append(tuple(p[-1]))
        p=sampled
    p=np.asarray(p,float); out=[]; inside=[]
    for i in range(len(p)):
        a=p[i-1] if i or closed else p[0]-(p[1]-p[0])
        b=p[(i+1)%len(p)] if i<len(p)-1 or closed else p[-1]+(p[-1]-p[-2])
        tangent=b-a; tangent/=np.linalg.norm(tangent)
        n=np.array([-tangent[1],tangent[0]])
        out.append(p[i]+n*width/2); inside.append(p[i]-n*width/2)
    h=(lambda x,y:z) if not callable(z) else z
    m=bucket(layer,mat)
    for i in range(len(p) if closed else len(p)-1):
        j=(i+1)%len(p)
        m.quad([(inside[i][0],inside[i][1],h(*inside[i])),
                (inside[j][0],inside[j][1],h(*inside[j])),
                (out[j][0],out[j][1],h(*out[j])),
                (out[i][0],out[i][1],h(*out[i]))])

def box(cx,cy,zbase,sx,sy,sz,layer,mat,collision=None,yaw=0):
    c,s=math.cos(yaw),math.sin(yaw)
    v=[]
    for x,y,z in [(-1,-1,0),(1,-1,0),(1,1,0),(-1,1,0),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]:
        xx=x*sx/2; yy=y*sy/2
        v.append((cx+c*xx-s*yy,cy+s*xx+c*yy,zbase+z*sz))
    for ids in [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]:
        pts=[v[i] for i in ids]; bucket(layer,mat).quad(pts)
        if collision: COLLISION[collision].quad(pts)

def beam(p0,p1,r,layer,mat,collision=None,n=8,r2=None):
    a,b=np.asarray(p0,float),np.asarray(p1,float)
    delta=b-a; length=np.linalg.norm(delta)
    if length<1e-8:return
    axis=delta/length; ref=np.array([0,0,1]) if abs(axis[2])<.9 else np.array([1,0,0])
    u=np.cross(axis,ref);u/=np.linalg.norm(u);v=np.cross(axis,u)
    r2=r if r2 is None else r2
    vertices=[]
    for p,rad in [(a,r),(b,r2)]:
        vertices.extend([tuple(p+rad*(u*math.cos(t)+v*math.sin(t))) for t in np.linspace(0,TAU,n,endpoint=False)])
    faces=[]
    for i in range(n):
        j=(i+1)%n; faces.extend([(i,j,n+j),(i,n+j,n+i)])
    faces.extend([(0,i+1,i) for i in range(1,n-1)])
    faces.extend([(n,n+i,n+i+1) for i in range(1,n-1)])
    bucket(layer,mat).add(vertices,faces)
    if collision: COLLISION[collision].add(vertices,faces)

def ellipsoid(cx,cy,cz,rx,ry,rz,layer,mat,n=9,rings=5):
    vertices=[(cx,cy,cz-rz)]
    for j in range(1,rings):
        phi=-math.pi/2+j*math.pi/rings
        for t in np.linspace(0,TAU,n,endpoint=False):
            vertices.append((cx+rx*math.cos(phi)*math.cos(t),cy+ry*math.cos(phi)*math.sin(t),cz+rz*math.sin(phi)))
    top=len(vertices); vertices.append((cx,cy,cz+rz));faces=[]
    for i in range(n):faces.append((0,1+(i+1)%n,1+i))
    for j in range(rings-2):
        for i in range(n):
            a=1+j*n+i;b=1+j*n+(i+1)%n;c=a+n;d=b+n
            faces.extend([(a,b,d),(a,d,c)])
    for i in range(n):faces.append((top,1+(rings-2)*n+i,1+(rings-2)*n+(i+1)%n))
    bucket(layer,mat).add(vertices,faces)

def ground_height(x,y):
    # The street grid and landscape share the zero-height datum. Only the
    # perimeter road mesh carries the A/B humps and the C/D rear slopes.
    return 0.0


def front_curve_height(fraction):
    # Both tangent joins are at zero. The midpoint of each A/B quarter-circle
    # is the 5 cm crest; zero end slopes give smooth joins with straight roads.
    return CFG['front_curve_height'] * math.sin(math.pi*fraction)**2

def gz(x,y,d=.002):return ground_height(x,y)+d

def original_yard_front_y(x):
    # Front boundary of the grey yard; notched in the middle to clear the round island.
    return 4.12+.47*math.exp(-((x-W/2)/.62)**2)

def bezier_points(start,controls,count=25):
    p0=np.asarray(start,float);p1,p2,p3=np.asarray(controls,float)
    return [tuple((1-t)**3*p0+3*(1-t)**2*t*p1+3*(1-t)*t*t*p2+t**3*p3)
            for t in np.linspace(0,1,count)]

ORIGINAL_YARD_FRONT=[(x,original_yard_front_y(x)) for x in np.linspace(.39,W-.39,50)]
YARD_JOIN_INDEX=PARKING_LEFT['yard_join_sample']
join_x,join_y=ORIGINAL_YARD_FRONT[YARD_JOIN_INDEX]
join_slope=-2*(join_x-W/2)/.62**2*(join_y-4.12)
LEFT_YARD_FRONT=bezier_points(PARKING_LEFT['yard_corner_start'],PARKING_LEFT['yard_corner_controls'])
LEFT_YARD_FRONT+=bezier_points(LEFT_YARD_FRONT[-1],[
    PARKING_LEFT['yard_blend_control'],[join_x-.10,join_y-.10*join_slope],[join_x,join_y]])[1:]
RIGHT_JOIN_INDEX=RIGHT_D['yard_join_sample']
right_join_x,right_join_y=ORIGINAL_YARD_FRONT[RIGHT_JOIN_INDEX]
right_join_slope=-2*(right_join_x-W/2)/.62**2*(right_join_y-4.12)
right_controls=[list(p) for p in RIGHT_D['yard_front_controls']]
right_controls[0]=[right_join_x+.10,right_join_y+.10*right_join_slope]
RIGHT_YARD_FRONT=bezier_points((right_join_x,right_join_y),right_controls,33)
RIGHT_YARD_FRONT+=bezier_points(RIGHT_YARD_FRONT[-1],RIGHT_D['yard_corner_controls'],33)[1:]

def yard_front_y(x):
    if x>=right_join_x:
        return float(np.interp(x,[p[0] for p in RIGHT_YARD_FRONT],[p[1] for p in RIGHT_YARD_FRONT]))
    if x>=join_x:return original_yard_front_y(x)
    return float(np.interp(x,[p[0] for p in LEFT_YARD_FRONT],[p[1] for p in LEFT_YARD_FRONT]))

def factory_north_curve():
    # West bulge followed by an inward sweep on the round-island side.
    # Each join has a horizontal tangent; the two ends meet vertical sides.
    points=[tuple(PARKING_LEFT['factory_north_start'])]
    for controls in PARKING_LEFT['factory_north_curves']:
        points+=bezier_points(points[-1],controls,PARKING_LEFT['factory_north_samples_per_curve']+1)[1:]
    return points

def factory_north_boundary_y(x):
    curve=factory_north_curve()
    return float(np.interp(x,[p[0] for p in curve],[p[1] for p in curve]))

def factory_north_outline(original):
    # Keep both lower corners and replace only the northern boundary.
    return original[:13]+list(reversed(factory_north_curve()))+original[39:]

def factory_north_tree_point(x,y):
    base=PARKING_LEFT['factory_north_start'][1]
    if y<=base:return x,y
    left=.37;right=W/2-CFG['central_road_width']/2
    cx=(left+right)/2;rx=(right-left)/2-.045
    old_ry=CFG['rear_island_end_y_assumed']-base-.045
    angle=math.atan2((y-base)/old_ry,(x-cx)/rx)
    cap=np.asarray(offset(factory_north_outline(P3_TREE_OUTLINE),.045)[13:-13])
    distance=np.r_[0,np.cumsum(np.linalg.norm(np.diff(cap,axis=0),axis=1))]
    target=angle/math.pi*distance[-1]
    return tuple(float(np.interp(target,distance,cap[:,i])) for i in (0,1))

def right_island_north_curve():
    points=[tuple(RIGHT_D['island_north_start'])]
    for controls in RIGHT_D['island_north_curves']:
        points+=bezier_points(points[-1],controls,RIGHT_D['samples_per_curve']+1)[1:]
    return points

def right_island_outline(original):
    return original[:13]+list(reversed(right_island_north_curve()))+original[39:]

def right_island_boundary_y(x):
    curve=right_island_north_curve()
    return float(np.interp(x,[p[0] for p in curve],[p[1] for p in curve]))

def right_island_tree_point(x,y):
    base=RIGHT_D['island_north_start'][1]
    if y<=base:return x,y
    left=W/2+CFG['central_road_width']/2;right=W-.37
    angle=math.atan2((y-base)/(CFG['rear_island_end_y_assumed']-base-.045),
                     (x-(left+right)/2)/((right-left)/2-.045))
    cap=np.asarray(offset(right_island_outline(P4_TREE_OUTLINE),.045)[13:-13])
    distance=np.r_[0,np.cumsum(np.linalg.norm(np.diff(cap,axis=0),axis=1))]
    return tuple(float(np.interp(angle/math.pi*distance[-1],distance,cap[:,i])) for i in (0,1))

# One vehicle opening at each front corner of the grey yard. The rail dashes,
# fence posts, white edge line, tree row, boundary lamps and the two barrier
# gates all key off these ranges, so they cannot drift apart.
YARD_OPENINGS=[]
YARD_GATE_LAMPS=(.90,W-.90)

FEATURES=[]
def feature(name,layer,x,y,z=0,**kw):FEATURES.append(dict(name=name,layer=layer,position=[x,y,z],**kw))

def rear_curve_height(y,tangent_y,rise,straight_run):
    """C2 height transition from the straight grade into the rear platform.

    Use actual Y, including each road edge, rather than a shared angle-based
    height: inner and outer radii must both inherit the same incoming grade.
    The inner edge reaches the plateau by the end of the quarter-circle.
    """
    length=CFG['outer_road_radius']-CFG['one_way_width']
    t=float(np.clip((y-tangent_y)/length,0,1))
    change=CFG['rear_road_height']-rise
    tangent=rise/straight_run*length
    # Quintic Hermite: endpoint heights fixed, incoming derivative inherited,
    # outgoing derivative zero, and zero second derivative at both endpoints.
    return rise+change*(10*t**3-15*t**4+6*t**5)+tangent*(t-6*t**3+8*t**4-3*t**5)

def perimeter():
    e=CFG['road_edge_inset_assumed'];R=CFG['outer_road_radius'];r=R-CFG['one_way_width']/2
    xl,xr=e+R,W-e-R;yb,yt=e+R,L-e-R
    rise=CFG['rear_straight_rise_assumed'];zt=CFG['rear_road_height']
    reference=CFG['rear_slope_start_reference']
    gate={'left_parking_gate':PARKING_LEFT,'right_parking_gate':RIGHT_D}[reference]
    start_y=gate['gate_base'][1];run=yt-start_y
    assert yb<start_y<yt, 'The shared ramp start must lie on the rear straight.'
    points=[]
    def add(x,y,nx,ny,z,seg): points.append((x,y,nx,ny,z,seg))
    def side_z(y):
        if y>start_y:return rise*(y-start_y)/run
        return 0.
    # CCW: bottom straight, B, right straight, C, rear straight, D, left straight, A
    for x in np.linspace(xl,xr,70,endpoint=False):add(x,yb-r,0,-1,0.0,'AB')
    # Include the exact midpoint so the exported mesh reaches the full crest.
    front_fractions=sorted(set(np.linspace(0,1,65,endpoint=False).tolist()+[.5]))
    for t in front_fractions:
        a=-math.pi/2+t*math.pi/2
        add(xr+r*math.cos(a),yb+r*math.sin(a),math.cos(a),math.sin(a),front_curve_height(t),'B')
    ys=sorted(set(np.linspace(yb,yt,180,endpoint=False).tolist()+[start_y]))
    for y in ys:add(xr+r,y,1,0,side_z(y),'BC')
    for a in np.linspace(0,math.pi/2,65,endpoint=False):
        y=yt+r*math.sin(a)
        add(xr+r*math.cos(a),y,math.cos(a),math.sin(a),rear_curve_height(y,yt,rise,run),'C')
    for x in np.linspace(xr,xl,70,endpoint=False):add(x,yt+r,0,1,zt,'CD')
    for a in np.linspace(math.pi/2,math.pi,65,endpoint=False):
        y=yt+r*math.sin(a)
        add(xl+r*math.cos(a),y,math.cos(a),math.sin(a),rear_curve_height(y,yt,rise,run),'D')
    for y in [yt]+list(reversed(ys)):add(xl-r,y,-1,0,side_z(y),'DA')
    for t in front_fractions:
        a=math.pi+t*math.pi/2
        add(xl+r*math.cos(a),yb+r*math.sin(a),math.cos(a),math.sin(a),front_curve_height(t),'A')
    clean=[]
    for p in points:
        if not clean or math.dist(p[:2],clean[-1][:2])>1e-9:clean.append(p)
    return clean,dict(xl=xl,xr=xr,yb=yb,yt=yt,straight_slope_run=run,straight_slope_rise=rise,
                      straight_slope_start_y=start_y,straight_slope_reference=reference)

def build_ground():
    box(W/2,L/2,-.14,W,L,.11,'base','base','base')
    # Flat shared-vertex terrain; do not recreate the former raised AB platform.
    xs=np.linspace(0,W,89);ys=np.linspace(0,L,180)
    vertices=[(x,y,ground_height(x,y)-.001) for y in ys for x in xs];faces=[]
    for j in range(len(ys)-1):
        for i in range(len(xs)-1):
            a=j*len(xs)+i;faces.extend([(a,a+1,a+len(xs)+1),(a,a+len(xs)+1,a+len(xs))])
    bucket('ground','asphalt').add(vertices,faces);COLLISION['terrain'].add(vertices,faces)
    # Four external boards; 3.3 x 5.4 measures their interior faces.
    for cx,cy,sx,sy in [(W/2,-.014,W+.056,.028),(W/2,L+.014,W+.056,.028),(-.014,L/2,.028,L),(W+.014,L/2,.028,L)]:
        box(cx,cy,-.14,sx,sy,.14,'boundary','blue_board','barriers')
        box(cx,cy,0,sx,sy,.19,'boundary','acrylic','barriers')

ROAD,ROAD_INFO=perimeter()

def rear_straight_height(y):
    """One shared grade for both roads and all attached entrance surfaces."""
    return ROAD_INFO['straight_slope_rise']*float(np.clip(
        (y-ROAD_INFO['straight_slope_start_y'])/ROAD_INFO['straight_slope_run'],0,1))

def perimeter_surface_height(y,center_height,segment):
    if segment in ('C','D'):
        return rear_curve_height(y,ROAD_INFO['yt'],ROAD_INFO['straight_slope_rise'],ROAD_INFO['straight_slope_run'])
    return center_height

def build_road():
    half=CFG['one_way_width']/2;top=[];bottom=[]
    for x,y,nx,ny,z,seg in ROAD:
        # Physical and visible deck tops use the actual road datum. The base
        # terrain is 1 mm below it, while markings sit slightly above it.
        top.extend([(x+half*nx,y+half*ny,perimeter_surface_height(y+half*ny,z,seg)),
                    (x-half*nx,y-half*ny,perimeter_surface_height(y-half*ny,z,seg))])
        bottom.extend([(x+half*nx,y+half*ny,-.014),(x-half*nx,y-half*ny,-.014)])
    faces=[]
    for i in range(len(ROAD)):
        a=2*i;b=2*((i+1)%len(ROAD));faces.extend([(a,b,b+1),(a,b+1,a+1)])
    bucket('elevated_road','asphalt').add(top,faces);COLLISION['road_deck'].add(top,faces)
    COLLISION['road_deck'].add(bottom,[(c,b,a) for a,b,c in faces])
    for i in range(len(ROAD)):
        j=(i+1)%len(ROAD)
        for side in [0,1]:
            a=2*i+side;b=2*j+side
            q=[bottom[a],bottom[b],top[b],top[a]]
            if side==1:q.reverse()
            bucket('elevated_road','deck_side').quad(q);COLLISION['road_deck'].quad(q)
    # Lines follow the actual road surface, not a flat ground decal.
    for sign in [-1,1]:
        m=bucket('markings','white')
        for i in range(len(ROAD)):
            j=(i+1)%len(ROAD);q=[]
            foreground=max(ROAD[i][1],ROAD[j][1])<=CFG['crossroad_near_y_assumed']
            # The foreground opens into the street grid; the existing island
            # kerbs provide its inner boundary instead of a line across it.
            if foreground and sign==-1:continue
            if sign==-1 and ROAD[i][5]=='DA' and min(ROAD[i][1],ROAD[j][1])<PARKING_LEFT['inner_rail_end_y'] and max(ROAD[i][1],ROAD[j][1])>3.30:
                continue
            if sign==-1 and ROAD[i][5]=='BC' and min(ROAD[i][1],ROAD[j][1])<RIGHT_D['inner_rail_end_y'] and max(ROAD[i][1],ROAD[j][1])>3.40:
                continue
            for index,delta in [(i,-.0025),(j,-.0025),(j,.0025),(i,.0025)]:
                x,y,nx,ny,z,seg=ROAD[index];d=sign*(half+delta)
                q.append((x+nx*d,y+ny*d,perimeter_surface_height(y+ny*d,z,seg)+.002))
            if (foreground and sign==1) or (not foreground and sign==-1):q.reverse()
            m.quad(q)
    # Outer rail continuous; inner rail open where the street grid connects.
    for sign in [-1,1]:
        distance=half+.007
        for i in range(0,len(ROAD),4):
            j=(i+4)%len(ROAD)
            x,y,nx,ny,z,seg=ROAD[i];xx,yy,nxx,nyy,zz,next_seg=ROAD[j]
            enabled=(sign==1) or seg in ('CD','C','D') or (seg in ('BC','DA') and y>3.72)
            if not enabled:continue
            ay=y+sign*distance*ny;by=yy+sign*distance*nyy
            a=(x+sign*distance*nx,ay,perimeter_surface_height(ay,z,seg))
            b=(xx+sign*distance*nxx,by,perimeter_surface_height(by,zz,next_seg))
            if sign==-1 and seg in ('DA','BC'):
                cutoff=(PARKING_LEFT if seg=='DA' else RIGHT_D)['inner_rail_end_y']
                if max(a[1],b[1])<=cutoff:continue
                if min(a[1],b[1])<cutoff:
                    t=(cutoff-a[1])/(b[1]-a[1])
                    clipped=tuple(np.asarray(a)+(np.asarray(b)-a)*t)
                    if a[1]<cutoff:a=clipped
                    else:b=clipped
            for h in [.031,CFG['guardrail_height_assumed']]:
                beam((a[0],a[1],a[2]+h),(b[0],b[1],b[2]+h),.0045,'rails','steel',None,n=5)
            if i%12==0:beam(a,(a[0],a[1],a[2]+CFG['guardrail_height_assumed']),.004,'rails','steel','barriers',n=6)
            # A thin, closed collision rail at the top, same clearance as visible rail.
            beam((a[0],a[1],a[2]+.065),(b[0],b[1],b[2]+.065),.0045,'rails','steel','barriers',n=5)

ISLANDS=[]
P3_TREE_OUTLINE=[]
P4_TREE_OUTLINE=[]
def add_island(name,pts):
    ISLANDS.append((name,pts));h=lambda x,y:gz(x,y,CFG['kerb_height_assumed'])
    poly(pts,h,'landscape','kerb','kerbs',thickness=CFG['kerb_height_assumed'])
    inner=offset(pts,.018);poly(inner,lambda x,y:h(x,y)+.001,'landscape','grass')
    ribbon(pts,.005,lambda x,y:h(x,y)+.0015,'markings','white',True)

def build_islands():
    hw=CFG['central_road_width']/2;left=W/2-hw;right=W/2+hw
    near,far=CFG['crossroad_near_y_assumed'],CFG['crossroad_far_y_assumed'];end=CFG['rear_island_end_y_assumed']
    p=rounded_rect(.37,.43,left,near,[.20,.14,.18,.44],15)
    add_island('P1 公园与水池',p)
    p2=[(W-x,y) for x,y in reversed(p)];add_island('P2 高楼群',p2)
    P3_TREE_OUTLINE[:]=rounded_rect(.37,far,left,end,[.18,.23,.20,.16],12)
    add_island('P3 低层工业建筑',factory_north_outline(P3_TREE_OUTLINE))
    P4_TREE_OUTLINE[:]=rounded_rect(right,far,W-.37,end,[.16,.22,.24,.18],12)
    add_island('P4 特色建筑',right_island_outline(P4_TREE_OUTLINE))
    cx,cy=CFG['round_island_centre_assumed'];r=CFG['round_island_radius_assumed']
    add_island('P5 圆形绿岛',ellipse(cx,cy,r,r,64))
    for name,pts in ISLANDS:
        feature(name,'landscape',sum(p[0] for p in pts)/len(pts),sum(p[1] for p in pts)/len(pts))
    # Grey courtyard, shaped to preserve the road around the central round island.
    front=LEFT_YARD_FRONT+ORIGINAL_YARD_FRONT[YARD_JOIN_INDEX+1:RIGHT_JOIN_INDEX]+RIGHT_YARD_FRONT
    upper=rounded_rect(.34,4.12,W-.34,L-.34,.5,20)
    # Explicit upper boundary follows the inner rear corner footprint.
    pts=front+[(W-.34,4.56)]+[(2.46+.5*math.cos(a),4.56+.5*math.sin(a)) for a in np.linspace(0,math.pi/2,20)]
    pts +=[(.84,5.06)]+[(.84+.5*math.cos(a),4.56+.5*math.sin(a)) for a in np.linspace(math.pi/2,math.pi,20)]
    # Duplicated corner entries are removed to keep triangulation clean.
    clean=[]
    for p in pts:
        if not clean or math.dist(p,clean[-1])>1e-7:clean.append(p)
    if math.dist(clean[0],clean[-1])<1e-7:clean.pop()
    build_parking_yard(clean)
    # The white edge line stops at the two openings, like a dropped kerb.
    YARD_OPENINGS[:]=[(LEFT_YARD_FRONT[0][0],PARKING_LEFT['booth_position'][0]+.04),
                     (RIGHT_D['kerb_end_x'],RIGHT_YARD_FRONT[-1][0])]
    ribbon(ORIGINAL_YARD_FRONT[YARD_JOIN_INDEX:RIGHT_JOIN_INDEX+1],.008,.004,'markings','white')
    build_left_parking_edge()
    build_right_parking_edge()
    # Small green marked rectangle seen in the reference courtyard.
    court=rounded_rect(1.42,4.65,1.88,4.98,.005,2)
    poly(court,.004,'yard','grass');ribbon(court,.006,.005,'markings','white',True)
    ribbon([(1.42,4.815),(1.88,4.815)],.004,.005,'markings','white')
    # The fence is 17 short bays; dropping the first and last three leaves the
    # vehicle openings at both front corners. Each bay spans front[3i..3i+1].
    open_bays=(0,1,2,14,15,16)
    for i,a in enumerate(ORIGINAL_YARD_FRONT[:-1:3]):
        if i in open_bays or 3*i<YARD_JOIN_INDEX or 3*i+1>RIGHT_JOIN_INDEX:continue
        beam((*a,.03),(*ORIGINAL_YARD_FRONT[3*i+1],.03),.003,'rails','steel',None,n=5)
    for i,(x,y) in enumerate(ORIGINAL_YARD_FRONT[::3]):
        if i in open_bays or 3*i<YARD_JOIN_INDEX or 3*i>RIGHT_JOIN_INDEX:continue
        beam((x,y,.002),(x,y,.052),.003,'rails','steel',None,n=5)

def build_left_parking_edge():
    """Rounded parking lip, dropped entrance and short dark garden fence."""
    p=PARKING_LEFT;start=p['kerb_start_x']
    path=[(start,yard_front_y(start))]+[point for point in LEFT_YARD_FRONT if point[0]>start]
    # The entrance remains flush; only its white outline crosses the gate mouth.
    entrance=[point for point in LEFT_YARD_FRONT if point[0]<start]+[path[0]]
    ribbon(entrance,.006,.004,'markings','white')
    for a,b in zip(path[:-1],path[1:]):
        delta=np.asarray(b)-a;normal=np.array([-delta[1],delta[0]])/np.linalg.norm(delta)*p['kerb_width']/2
        outline=[tuple(np.asarray(a)-normal),tuple(np.asarray(b)-normal),tuple(np.asarray(b)+normal),tuple(np.asarray(a)+normal)]
        poly(outline,p['kerb_height'],'landscape','kerb','kerbs',thickness=p['kerb_height']-.002)
    ribbon(path,.006,p['kerb_height']+.0015,'markings','white')
    fence=[(x,yard_front_y(x)+.018) for x in np.linspace(start+.035,join_x,48)]
    for height in [.025,.048]:
        for a,b in zip(fence[:-1],fence[1:]):
            beam((*a,height),(*b,height),.0018,'rails','black','barriers' if height==.048 else None,n=4)
    for x,y in evenly_spaced(fence,.048,closed=False)+[fence[-1]]:
        beam((x,y,.011),(x,y,.055),.002,'rails','black','barriers',n=4)
    for x,y in evenly_spaced(fence,.014,closed=False):
        beam((x,y,.016),(x,y,.046),.0008,'rails','black',n=4)

def build_parking_yard(yard):
    """Keep the entire yard level, with a matching closed collision slab."""
    if signed_area(yard)<0:yard=list(reversed(yard))
    top=[(x,y,PARKING_Z) for x,y in yard];faces=triangulate(yard)
    bucket('yard','grey_yard').add(top,faces)
    bottom=[(x,y,-.014) for x,y in yard]
    collision=COLLISION['yard'];collision.add(top,faces)
    collision.add(bottom,[(c,b,a) for a,b,c in faces])
    for a in range(len(yard)):
        b=(a+1)%len(yard)
        collision.quad([bottom[a],bottom[b],top[b],top[a]])

def build_right_parking_edge():
    end=RIGHT_D['kerb_end_x']
    path=[p for p in RIGHT_YARD_FRONT if p[0]<end]+[(end,yard_front_y(end))]
    for a,b in zip(path[:-1],path[1:]):
        d=np.asarray(b)-a;n=np.array([-d[1],d[0]])/np.linalg.norm(d)*.007
        p=[tuple(np.asarray(a)-n),tuple(np.asarray(b)-n),tuple(np.asarray(b)+n),tuple(np.asarray(a)+n)]
        poly(p,PARKING_Z+.009,'landscape','kerb','kerbs',thickness=.009)
    ribbon(path,.006,PARKING_Z+.0105,'markings','white')
    mouth=[path[-1]]+[p for p in RIGHT_YARD_FRONT if p[0]>end]
    ribbon(mouth,.006,PARKING_Z+.002,'markings','white')
    fence=[(x,yard_front_y(x)+.020) for x in np.linspace(right_join_x,end-.028,45)]
    for h in [.025,.048]:
        for a,b in zip(fence[:-1],fence[1:]):
            beam((*a,PARKING_Z+h),(*b,PARKING_Z+h),.0018,'rails','black','barriers' if h==.048 else None,n=4)
    for x,y in evenly_spaced(fence,.048,False)+[fence[-1]]:
        beam((x,y,PARKING_Z+.009),(x,y,PARKING_Z+.053),.002,'rails','black','barriers',n=4)
    for x,y in evenly_spaced(fence,.014,False):
        beam((x,y,PARKING_Z+.014),(x,y,PARKING_Z+.044),.0008,'rails','black',n=4)

def surface_rect(cx,cy,sx,sy,mat='white',zextra=.004):
    poly([(cx-sx/2,cy-sy/2),(cx+sx/2,cy-sy/2),(cx+sx/2,cy+sy/2),(cx-sx/2,cy+sy/2)],lambda x,y:gz(x,y,zextra),'markings',mat)

def arrow(x,y,angle=0,size=1):
    p=[(-.012,-.07),(.012,-.07),(.012,.014),(.040,.014),(0,.076),(-.040,.014),(-.012,.014)]
    c,s=math.cos(angle),math.sin(angle)
    p=[(x+size*(c*a-s*b),y+size*(s*a+c*b)) for a,b in p]
    poly(p,lambda x,y:gz(x,y,.004),'markings','white')

def foreground_arrow(x,y,kind='straight',angle=0,surface_height=None):
    """Photo-based single-polygon paint symbols; local +Y is forward."""
    shapes={
        'straight':[
            (-.006,-.10),(.006,-.10),(.006,.03),(.024,.03),
            (0,.105),(-.024,.03),(-.006,.03)],
        'bend_left':[
            (.006,-.105),(-.006,-.105),(-.006,.005),(-.040,.035),
            (-.040,-.005),(-.058,.050),(-.045,.105),(-.045,.067),
            (.006,.030)],
        'straight_left':[
            (-.006,-.10),(.006,-.10),(.006,.03),(.025,.03),
            (0,.105),(-.025,.03),(-.006,.03),(-.006,-.01),
            (-.045,.023),(-.045,.046),(-.095,.01),(-.045,-.023),
            (-.045,0),(-.006,-.048)],
        'threeway':[
            (-.005,-.11),(.007,-.11),(.004,-.045),
            (.047,-.009),(.044,-.035),(.062,.014),(.043,.062),
            (.044,.026),(.006,-.001),(.006,.060),(.025,.060),
            (0,.145),(-.025,.060),(-.006,.060),(-.006,-.001),
            (-.044,.026),(-.043,.062),(-.062,.014),(-.044,-.035),
            (-.047,-.009),(-.004,-.045)]
    }
    shapes['bend_right']=[(-a,b) for a,b in shapes['bend_left']]
    c,s=math.cos(angle),math.sin(angle)
    pts=[(x+c*a-s*b,y+s*a+c*b) for a,b in shapes[kind]]
    height=(lambda x,y:gz(x,y,.004)) if surface_height is None else (lambda x,y:surface_height(x,y)+.004)
    poly(pts,height,'markings','white')


def build_foreground_markings():
    """Only AB and the south approach to the central intersection."""
    p=FRONT_MARKINGS;cx=W/2;hw=CFG['central_road_width']/2
    a=CFG['crossroad_near_y_assumed']
    crossing_y=a-p['crosswalk_offset_from_crossroad']
    crossing_near=crossing_y-p['crosswalk_depth']/2
    ribbon([(cx,p['near_stop_y']),(cx,crossing_near)],
           CFG['centre_line_width'],lambda x,y:gz(x,y,.003),'markings','white')
    surface_rect(cx,p['near_stop_y'],2*hw,p['near_stop_thickness'])
    stripe_width=p['crosswalk_stripe_width'];pitch=p['crosswalk_stripe_pitch']
    count=math.floor((2*hw-stripe_width)/pitch)+1
    span=(count-1)*pitch
    for x in np.linspace(cx-span/2,cx+span/2,count):
        surface_rect(x,crossing_y,stripe_width,p['crosswalk_depth'])
    foreground_arrow(cx-.155,p['near_arrow_y'],'bend_left',math.pi)
    foreground_arrow(cx+.155,p['near_arrow_y'],'straight')
    foreground_arrow(cx+.155,a-p['junction_arrow_offset_from_crossroad'],'threeway')
    for x in p['front_transverse_line_x']:
        ribbon([(x,p['front_transverse_line_y'][0]),(x,p['front_transverse_line_y'][1])],
               p['front_transverse_line_width'],lambda x,y:gz(x,y,.004),'markings','white')
    foreground_arrow(*p['front_combination_arrow_position'],'straight_left',-math.pi/2)
    edge=CFG['road_edge_inset_assumed'];w=CFG['one_way_width']
    for y in [p['outer_side_stop_y'],crossing_y]:
        for x in [edge+w/2,W-edge-w/2]:surface_rect(x,y,w,.007)
    arrow(.19,1.5,math.pi,.9);arrow(W-.19,1.5,0,.9)


def build_central_crossroad_markings():
    """Two opposing lanes on each horizontal approach, matching the photo."""
    cx=W/2;hw=CFG['central_road_width']/2
    a=CFG['crossroad_near_y_assumed'];b=CFG['crossroad_far_y_assumed']
    centre_y=(a+b)/2;lane_offset=(b-a)/4
    outer_junction=CFG['road_edge_inset_assumed']+CFG['one_way_width']
    # Stop before the existing side crosswalks (offset .035, depth .105).
    crossing_edge=cx-hw-.035-.105/2
    for x0,x1 in [(outer_junction,crossing_edge-.008),
                  (W-crossing_edge+.008,W-outer_junction)]:
        ribbon([(x0,centre_y),(x1,centre_y)],CFG['centre_line_width'],
               lambda x,y:gz(x,y,.003),'markings','white')
    # Near-side traffic heads east; far-side traffic heads west.
    # Incoming lanes show the three intersection choices; outgoing lanes
    # bend toward the one-way outer road at the next junction.
    foreground_arrow(cx-hw-.30,centre_y-lane_offset,'threeway',-math.pi/2)
    foreground_arrow(outer_junction+.32,centre_y+lane_offset,'bend_left',math.pi/2)
    foreground_arrow(cx+hw+.30,centre_y+lane_offset,'threeway',math.pi/2)
    foreground_arrow(W-outer_junction-.32,centre_y-lane_offset,'bend_left',-math.pi/2)

def build_left_parking_markings():
    # Photo landmarks: the stop line meets the booth and the factory island;
    # the two arrows guide the outer lane and the link beside the round island.
    bx,by=PARKING_LEFT['booth_position']
    ribbon([(.55,factory_north_boundary_y(.55)),(bx,by-.037)],.008,
           lambda x,y:gz(x,y,.004),'markings','white')
    surface_rect(.2075,3.37,.335,.008)
    height=lambda x,y:rear_straight_height(y)
    foreground_arrow(.19,4.00,'straight_left',math.pi,surface_height=height)
    arrow(1.11,3.76,-3*math.pi/4,.85)


def build_right_d_markings():
    # Arrow on the inner/left side; the outer/right line stays parallel to Y.
    line_x=2.80
    ribbon([(line_x,right_island_boundary_y(line_x)),(line_x,yard_front_y(line_x))],.008,
           lambda x,y:gz(x,y,.004),'markings','white')
    ribbon([(2.91,3.50),(W-.04,3.50)],.008,.004,'markings','white')
    foreground_arrow(2.72,3.91,'bend_right')
    arrow(2.17,3.79,-math.pi/4,.85)

def build_markings():
    cx=W/2;hw=CFG['central_road_width']/2;a=CFG['crossroad_near_y_assumed'];b=CFG['crossroad_far_y_assumed']
    build_foreground_markings()
    for y0,y1 in [(b+.16,3.60)]:
        ribbon([(cx,y0),(cx,y1)],CFG['centre_line_width'],lambda x,y:gz(x,y,.003),'markings','white')
    for y in [b+.035]:
        for x in np.arange(cx-hw+.013,cx+hw-.01,.047):surface_rect(x,y,.025,.105)
    for x in [cx-hw-.035,cx+hw+.035]:
        for y in np.arange(a+.012,b-.01,.047):surface_rect(x,y,.105,.025)
    for x in [cx-hw,cx+hw]:
        for y0,y1 in [(3.5,3.67)]:ribbon([(x,y0),(x,y1)],.005,lambda x,y:gz(x,y,.003),'markings','white')
    for y in [b]:
        for x0,x1 in [(.19,.39),(W-.39,W-.19)]:ribbon([(x0,y),(x1,y)],.005,lambda x,y:gz(x,y,.003),'markings','white')
    for y in [3.08]:
        foreground_arrow(cx-.155,y,'threeway',math.pi)
        # The photo places the outgoing bent arrow farther up the right lane.
        foreground_arrow(cx+.155,y+.28,'bend_right')
    arrow(.19,3.05,math.pi,.9);arrow(W-.19,3.26,0,.9)
    build_central_crossroad_markings()
    build_left_parking_markings()
    build_right_d_markings()
    for y in [b+.135]:surface_rect(cx,y,.62,.007)
    # Grade-following arrows on the elevated rear road.
    for x,y,theta,z in [(1.3,5.21,math.pi/2,.132),(2.05,5.21,math.pi/2,.132)]:
        p=[(-.012,-.06),(.012,-.06),(.012,.01),(.035,.01),(0,.065),(-.035,.01),(-.012,.01)]
        c,s=math.cos(theta),math.sin(theta);poly([(x+c*a-s*b,y+s*a+c*b) for a,b in p],z,'markings','white')

def pond(cx,cy,rx,ry):
    p=ellipse(cx,cy,rx,ry,60,.15)
    poly(p,lambda x,y:gz(x,y,.014),'landscape','paving')
    poly(offset(p,.01),lambda x,y:gz(x,y,.015),'landscape','water')

def b_lake_outline():
    """One smooth outline joins the long lake to its narrow outlet channel."""
    points=[];start=np.asarray(B_LAKE['start'],float)
    for controls in B_LAKE['curves']:
        a,b,end=np.asarray(controls,float)
        for t in np.linspace(0,1,B_LAKE['samples_per_curve'],endpoint=False):
            p=(1-t)**3*start+3*(1-t)**2*t*a+3*(1-t)*t*t*b+t**3*end
            points.append(tuple(p))
        start=end
    if signed_area(points)<0:points.reverse()
    return points


def build_b_lake():
    p=b_lake_outline()
    poly(offset(p,-B_LAKE['bank_width']),lambda x,y:gz(x,y,B_LAKE['bank_z']),'landscape','grey_yard')
    poly(p,lambda x,y:gz(x,y,B_LAKE['water_z']),'landscape','water')


def build_park():
    for x0,y0,x1,y1 in [(.61,.87,1.08,.925),(.62,1.13,1.22,1.19),(.71,1.53,1.20,1.59),(.71,1.02,.77,1.9),(1.08,.65,1.14,1.94)]:
        poly([(x0,y0),(x1,y0),(x1,y1),(x0,y1)],lambda x,y:gz(x,y,.014),'landscape','paving')
    pond(.78,.75,.27,.20);pond(1.03,1.42,.075,.055);pond(.86,1.79,.08,.07)
    build_b_lake()
    for x0,y0,x1,y1 in [(2.25,.64,2.87,.68),(2.66,.71,2.7,1.92)]:
        poly([(x0,y0),(x1,y0),(x1,y1),(x0,y1)],lambda x,y:gz(x,y,.014),'landscape','paving')

def building(cx,cy,sx,sy,h,name,cross_brace=False):
    z=gz(cx,cy,.015);box(cx,cy,z,sx,sy,h,'buildings','glass','buildings')
    box(cx,cy,z+h,sx+.018,sy+.018,.012,'buildings','roof')
    box(cx,cy,z+h+.012,sx-.025,sy-.025,.005,'buildings','roof_dark')
    floors=max(2,round(h/.038))
    for zz in np.linspace(z+.012,z+h,floors+1):
        for yy in [cy-sy/2-.001,cy+sy/2+.001]:beam((cx-sx/2,yy,zz),(cx+sx/2,yy,zz),.0018,'buildings','frame',n=4)
        for xx in [cx-sx/2-.001,cx+sx/2+.001]:beam((xx,cy-sy/2,zz),(xx,cy+sy/2,zz),.0018,'buildings','frame',n=4)
    for xx in np.linspace(cx-sx/2,cx+sx/2,max(3,int(sx/.035)+1)):
        for yy in [cy-sy/2-.001,cy+sy/2+.001]:beam((xx,yy,z),(xx,yy,z+h),.0015,'buildings','frame',n=4)
    for yy in np.linspace(cy-sy/2,cy+sy/2,max(3,int(sy/.035)+1)):
        for xx in [cx-sx/2-.001,cx+sx/2+.001]:beam((xx,yy,z),(xx,yy,z+h),.0015,'buildings','frame',n=4)
    if cross_brace:
        for zz in np.arange(z,z+h-.005,.12):
            z1=min(z+h,zz+.12)
            for yy in [cy-sy/2-.003,cy+sy/2+.003]:
                for xx in np.arange(cx-sx/2,cx+sx/2-.005,.085):
                    x1=min(cx+sx/2,xx+.085)
                    beam((xx,yy,zz),(x1,yy,z1),.002,'buildings','white',n=4)
                    beam((x1,yy,zz),(xx,yy,z1),.002,'buildings','white',n=4)
            for xx in [cx-sx/2-.003,cx+sx/2+.003]:
                for yy in np.arange(cy-sy/2,cy+sy/2-.005,.085):
                    y1=min(cy+sy/2,yy+.085)
                    beam((xx,yy,zz),(xx,y1,z1),.002,'buildings','white',n=4)
                    beam((xx,y1,zz),(xx,yy,z1),.002,'buildings','white',n=4)
    feature(name,'buildings',cx,cy,z+h,height=h)

def b_facade_stroke(a,b,outward,width,material='frame',stand_off=.001):
    """A thin, outward-facing paint strip on a vertical B-district facade."""
    a,b=np.asarray(a,float),np.asarray(b,float);normal=np.asarray(outward,float)
    direction=b-a;length=np.linalg.norm(direction)
    if length<1e-7:return
    across=np.cross(normal,direction/length)*width/2
    points=[a-across+normal*stand_off,b-across+normal*stand_off,
            b+across+normal*stand_off,a+across+normal*stand_off]
    if np.dot(np.cross(points[1]-points[0],points[2]-points[0]),normal)<0:points.reverse()
    bucket('buildings',material).quad(points)


def b_facade(p0,p1,z,height,style):
    """Continuous diamond lattice; it does not repeat an X in every window."""
    origin=np.array([p0[0],p0[1],z]);delta=np.asarray(p1)-np.asarray(p0)
    length=float(np.linalg.norm(delta));u=np.array([delta[0]/length,delta[1]/length,0.])
    normal=np.array([u[1],-u[0],0.])
    def line(x0,z0,x1,z1,width,mat='frame',stand_off=.001):
        b_facade_stroke(origin+u*x0+[0,0,z0],origin+u*x1+[0,0,z1],normal,width,mat,stand_off)
    # Fine glass joints are deliberately finer than the white diagonal structure.
    pitch=.018 if style=='diamond' else .036
    for x in np.linspace(0,length,max(2,math.ceil(length/pitch))+1):
        line(x,0,x,height,.0004 if style=='diamond' else .0007)
    for h in np.linspace(0,height,max(2,round(height/.023))+1):
        line(0,h,length,h,.0004 if style=='diamond' else .0006)
    if style=='diamond':
        for slope in [-2.25,2.25]:
            step=.084
            low=min(0,-slope*length);high=max(height,height-slope*length)
            for intercept in np.arange(math.floor(low/step)*step,high+step/2,step):
                limits=sorted((-intercept/slope,(height-intercept)/slope))
                a=max(0,limits[0]);b=min(length,limits[1])
                if b-a>1e-6:line(a,slope*a+intercept,b,slope*b+intercept,.0012,'white',.0015)
    else:
        # Three storeys and a few broad facade bays on the low buildings.
        for h in np.linspace(0,height,max(2,round(height/.03))+1):line(0,h,length,h,.002,'frame',.0015)
        for x in np.linspace(0,length,max(1,round(length/.09))+1):line(x,0,x,height,.002,'frame',.0015)
    for x in [0,length]:line(x,0,x,height,.0014,'frame',.0018)


def b_roof_ribs(points,z):
    """Small parallel seams clipped to the entire L/C-shaped roof polygon."""
    for x in np.arange(min(p[0] for p in points)+.004,max(p[0] for p in points),.006):
        hits=[]
        for a,b in zip(points,points[1:]+points[:1]):
            if min(a[0],b[0])<=x<max(a[0],b[0]):
                hits.append(a[1]+(x-a[0])/(b[0]-a[0])*(b[1]-a[1]))
        hits.sort()
        for a,b in zip(hits[::2],hits[1::2]):
            if b-a>.003:
                bucket('buildings','frame').quad([(x-.0002,a,z),(x+.0002,a,z),
                                                   (x+.0002,b,z),(x-.0002,b,z)])


def b_volume(points,z,height,name,facade='grid',roof='flat',ribs=False,roof_points=None):
    """One closed concave prism, including collision in the true footprint."""
    points=list(points)
    if signed_area(points)<0:points.reverse()
    poly(points,z+height,'buildings','glass','buildings',thickness=height)
    for a,b in zip(points,points[1:]+points[:1]):b_facade(a,b,z,height,facade)
    if roof!='none':
        # A continuous light roof avoids seams where separate boxes used to meet.
        roof_points=points if roof_points is None else roof_points
        outline=offset(roof_points,-.002)
        poly(outline,z+height+.003,'buildings','roof',thickness=.003)
        ribbon(outline,.0015,z+height+.0034,'buildings','frame',True)
        if ribs:b_roof_ribs(offset(roof_points,.003),z+height+.0035)
        if roof=='recessed':
            # Thin glass parapet surrounding the inset pale roof of the tallest tower.
            inner=offset(points,.002)
            for a,b in zip(inner,inner[1:]+inner[:1]):
                direction=np.asarray(b)-np.asarray(a);length=float(np.linalg.norm(direction))
                midpoint=(np.asarray(a)+np.asarray(b))/2
                box(*midpoint,z+height,length,.003,.021,'buildings','glass_light',yaw=math.atan2(direction[1],direction[0]))
                beam((*a,z+height+.021),(*b,z+height+.021),.0008,'buildings','frame',n=4)
    feature(name,'buildings',sum(p[0] for p in points)/len(points),sum(p[1] for p in points)/len(points),z+height,height=height)


def build_b_district():
    """B buildings only. Landscape, road paint, lights and tree placement stay fixed."""
    base=B_BUILDINGS['base_z']
    for item in B_BUILDINGS['low_blocks']:
        c,s=math.cos(item['yaw']),math.sin(item['yaw']);x,y=item['origin']
        points=[(x+c*a-s*b,y+s*a+c*b) for a,b in item['outline']]
        b_volume(points,base,item['height'],item['name'],ribs=True)
    for item in B_BUILDINGS['tower_parts']:
        x0,y0,x1,y1=item['bounds']
        points=[(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
        roof_points=None
        if 'roof_bounds' in item:
            a,b,c,d=item['roof_bounds'];roof_points=[(a,b),(c,b),(c,d),(a,d)]
        b_volume(points,base+item['bottom'],item['height'],item['name'],item['facade'],item['roof'],ribs=item['roof']=='flat',roof_points=roof_points)
    # The low rooftop housing remains inside the large facade recess.
    box(2.724,1.60,base+.098,.083,.115,.018,'buildings','roof')
    box(2.724,1.60,base+.116,.071,.103,.003,'buildings','frame')


def factory_facade(bounds,z,height):
    """Horizontal storey bands dominate the real low-rise glass facades."""
    x0,y0,x1,y1=bounds
    points=[(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
    for a,b in zip(points,points[1:]+points[:1]):
        a=np.asarray(a);b=np.asarray(b);direction=b-a;length=np.linalg.norm(direction)
        normal=np.array([direction[1]/length,-direction[0]/length,0.])
        def stroke(t0,h0,t1,h1,width):
            p0=np.r_[a+direction*t0,z+h0];p1=np.r_[a+direction*t1,z+h1]
            b_facade_stroke(p0,p1,normal,width,'frame',.001)
        for t in np.linspace(0,1,max(2,round(length/.027))+1):
            stroke(t,0,t,height,.00055)
        for h in np.linspace(.008,height-.002,max(2,round(height/.033))+1):
            stroke(0,h,1,h,.0035)


def build_factory():
    cfg=FACTORY_BUILDING;z=gz(cfg['wing_center_x'],3.16,.015)
    sx,sy=cfg['wing_size'];h=cfg['wing_height'];cx=cfg['wing_center_x']
    # Three separate wings: the two full-height recesses remain empty in both
    # the visual model and the collision mesh, rather than painted on a slab.
    for cy in cfg['wing_centers_y']:
        box(cx,cy,z,sx,sy,h,'buildings','glass','buildings')
        factory_facade((cx-sx/2,cy-sy/2,cx+sx/2,cy+sy/2),z,h)
        roof_z=z+h
        box(cx,cy,roof_z,sx+.012,sy+.012,.006,'buildings','roof')
        for window in cfg['skylights']:
            wx=window['center_x'];wsx,wsy=window['size'];wz=roof_z+.006
            # Raised light rim with a lower coloured pane, including a small
            # outboard window and a larger dark window on each wing.
            box(wx,cy,wz,wsx,wsy,.008,'buildings','frame')
            box(wx,cy,wz+.0081,wsx-.008,wsy-.008,.0005,'buildings',window['material'])
            rim=[(wx-wsx/2,cy-wsy/2),(wx+wsx/2,cy-wsy/2),
                 (wx+wsx/2,cy+wsy/2),(wx-wsx/2,cy+wsy/2)]
            ribbon(offset(rim,.0015),.003,wz+.010,'buildings','white',True)
    # Long narrow connecting gallery, raised over the row of red columns.
    x0,y0,x1,y1=cfg['spine_bounds'];bottom=cfg['spine_bottom'];height=cfg['spine_height']
    box((x0+x1)/2,(y0+y1)/2,z+bottom,x1-x0,y1-y0,height,'buildings','glass','buildings')
    factory_facade((x0,y0,x1,y1),z+bottom,height)
    roof_z=z+bottom+height
    box((x0+x1)/2,(y0+y1)/2,roof_z,x1-x0+.010,y1-y0+.012,.004,'buildings','roof')
    rim=[(x0-.005,y0-.006),(x1+.005,y0-.006),(x1+.005,y1+.006),(x0-.005,y1+.006)]
    ribbon(offset(rim,.002),.0015,roof_z+.0045,'buildings','frame',True)
    for cy in np.linspace(y0+.020,y1-.020,cfg['column_count']):
        box(cfg['column_x'],cy,z,.009,.011,bottom,'buildings','red','buildings')
    feature('B4 工业建筑','buildings',(x0+cx)/2,3.16,roof_z+.004,height=bottom+height)


def build_buildings():
    build_b_district()
    build_factory()
    box(1.15,3.12,gz(1.15,3.12,.015),.07,.08,.11,'buildings','white','buildings')
    build_right_d_buildings()

def build_right_d_buildings():
    cfg=RIGHT_D;z=.015;u=cfg['u_building'];p=[tuple(a) for a in u['outline']];h=u['height']
    poly(p,z+h,'buildings','glass','buildings',thickness=h)
    roof=offset(p,-.004);poly(roof,z+h+.004,'buildings','white',thickness=.004)
    # Broad white facade bands and a real open courtyard, not two solid slabs.
    for a,b in zip(p,p[1:]+p[:1]):
        a=np.asarray(a);b=np.asarray(b);delta=b-a;length=np.linalg.norm(delta)
        normal=np.r_[delta[1]/length,-delta[0]/length,0]
        for hh in np.linspace(.012,h-.010,4):
            b_facade_stroke(np.r_[a,z+hh],np.r_[b,z+hh],normal,.006,'white')
        for t in np.linspace(0,1,max(2,round(length/.055))+1):
            pt=a+delta*t;b_facade_stroke(np.r_[pt,z],np.r_[pt,z+h],normal,.003,'white')
    canopy=offset(p,-.014);top=z+u['canopy_height']
    for a,b in zip(canopy,canopy[1:]+canopy[:1]):
        for hh in [top-.016,top]:beam((*a,hh),(*b,hh),.0012,'buildings','frame',n=4)
        for x,y in evenly_spaced([a,b],.027,False):
            beam((x,y,z+h+.005),(x,y,top),.0008,'buildings','frame',n=4)
    # Fine open lattice above each roof wing; the U-shaped opening stays clear.
    for y in np.arange(2.96,3.31,.027):
        ranges=[(2.055,2.18),(2.28,2.405)] if y<3.105 else [(2.055,2.405)]
        for a,b in ranges:beam((a,y,top),(b,y,top),.00065,'buildings','frame',n=4)
    for x in np.arange(2.055,2.405,.027):
        y0=3.105 if 2.175<x<2.285 else 2.96
        beam((x,y0,top),(x,3.31,top),.00065,'buildings','frame',n=4)
    for x,y in [(2.051,2.951),(2.409,2.951),(2.051,3.309),(2.409,3.309)]:
        beam((x,y,z),(x,y,top),.0018,'buildings','white','buildings',n=4)
    feature('D区 U形中庭楼','buildings',2.23,3.13,z+h,height=h)

    wedge=cfg['wedge'];x0,y0,x1,y1=wedge['bounds'];lo,hi=wedge['low_height'],wedge['high_height']
    height=lambda y:lo+(hi-lo)*(y-y0)/(y1-y0)
    roof_z=lambda x,y:z+height(y)
    rect=[(x0,y0),(x1,y0),(x1,y1),(x0,y1)]
    v=[(x,y,z) for x,y in rect]+[(x,y,roof_z(x,y)) for x,y in rect]
    for q in [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]:
        points=[v[i] for i in q];bucket('buildings','glass').quad(points);COLLISION['buildings'].quad(points)
    # Horizontal white bands stop at the sloped roof; sparse vertical joints.
    for zz in np.arange(.025,hi,.028):
        yy=y0+max(0.,(zz-lo)/(hi-lo))*(y1-y0)
        for x in [x0-.001,x1+.001]:
            b_facade_stroke(np.array([x,yy,z+zz]),np.array([x,y1,z+zz]),[-1 if x<x0 else 1,0,0],.0045,'white')
        for y in [y0,y1]:
            if zz<height(y):b_facade_stroke(np.array([x0,y,z+zz]),np.array([x1,y,z+zz]),[0,-1 if y==y0 else 1,0],.0035,'white')
    for y in np.arange(y0,y1+.0001,.035):
        for x in [x0-.001,x1+.001]:
            beam((x,y,z),(x,y,roof_z(x,y)),.00065,'buildings','frame',n=4)
    for x in np.linspace(x0,x1,8):
        beam((x,y1+.001,z),(x,y1+.001,z+hi),.0008,'buildings','frame',n=4)
    poly(offset(rect,-.004),lambda x,y:roof_z(x,y)+.003,'buildings','roof',thickness=.003)
    roof_window=[(x0+.035,y0+.048),(x1-.035,y0+.048),(x1-.035,y1-.048),(x0+.035,y1-.048)]
    poly(roof_window,lambda x,y:roof_z(x,y)+.0038,'buildings','roof_dark')
    ribbon(roof_window,.0025,lambda x,y:roof_z(x,y)+.0045,'buildings','white',True)
    for y in np.linspace(y0+.048,y1-.048,8):
        beam((x0+.035,y,roof_z(x0,y)+.0045),(x1-.035,y,roof_z(x0,y)+.0045),.0008,'buildings','frame',n=4)
    feature('D区斜顶楼与长条屋顶窗','buildings',(x0+x1)/2,(y0+y1)/2,z+hi,height=hi)

    shed=cfg['shed'];cx,cy=shed['center'];sx,sy=shed['size'];wall=shed['wall_height'];arch=shed['arch_height']
    # Closed barrel-roof shed, including its dark end walls and curved collision.
    section=[(cx-sx/2,z),(cx+sx/2,z)]+[(cx+sx/2*math.cos(t),z+wall+arch*math.sin(t)) for t in np.linspace(0,math.pi,25)]
    indices=triangulate(section);n=len(section)
    for y,reverse in [(cy-sy/2,False),(cy+sy/2,True)]:
        vv=[(x,y,zz) for x,zz in section];ff=[(c,b,a) if reverse else (a,b,c) for a,b,c in indices]
        bucket('buildings','glass').add(vv,ff);COLLISION['buildings'].add(vv,ff)
    for a,b in zip(section,section[1:]+section[:1]):
        q=[(a[0],cy-sy/2,a[1]),(a[0],cy+sy/2,a[1]),(b[0],cy+sy/2,b[1]),(b[0],cy-sy/2,b[1])]
        bucket('buildings','red').quad(q);COLLISION['buildings'].quad(q)
    # Small white utility box and short pale service path shown beside the slope.
    bx,by=cfg['utility_cabinet'];box(bx,by,z,.082,.085,.068,'buildings','white','buildings')
    box(bx,by,z+.068,.087,.09,.005,'buildings','roof')
    box(bx,by-.043,z+.012,.057,.001,.043,'buildings','glass_light')
    ribbon([(2.20,2.925),(2.48,2.925),(bx,by)],.012,.016,'landscape','paving')

def inside_polygon(x,y,p):
    inside=False
    for i in range(len(p)):
        a,b=p[i-1],p[i]
        if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:inside=not inside
    return inside

TREE_POS=[]
def tree(x,y,h=.14,wide=1,emit=True,base_height=None):
    z=gz(x,y,.015) if base_height is None else base_height+.015;r=h*.21*wide
    # Leaf materials are drawn even for an omitted tree, so clearing a gateway
    # never shifts the colours of the trees that follow it.
    leaf=[random.randrange(4) for _ in range(4)]
    if not emit:return
    beam((x,y,z),(x,y,z+h*.72),.006,'vegetation','bark','trees',n=6,r2=.003)
    for (dx,dy,dz,s),mat in zip([(0,0,.7,1),(-.6,0,.6,.75),(.5,.25,.68,.7),(0,-.4,.87,.75)],leaf):
        ellipsoid(x+dx*r,y+dy*r,z+dz*h,r*s,r*s,h*.24*s,'vegetation',f'leaf{mat}',n=7,rings=4)
    TREE_POS.append((x,y,h))

def evenly_spaced(p,spacing,closed=True):
    pts=list(p)+([p[0]] if closed else []);result=[];remaining=0
    for a,b in zip(pts[:-1],pts[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
        if length<1e-10:continue
        while remaining<length:
            t=remaining/length;result.append((a[0]+t*dx,a[1]+t*dy));remaining+=spacing
        remaining-=length
    return result

def build_trees():
    for name,p in ISLANDS[:4]:
        source=P3_TREE_OUTLINE if name=='P3 低层工业建筑' else P4_TREE_OUTLINE if name=='P4 特色建筑' else p
        for x,y in evenly_spaced(offset(source,.045),.115):
            if name=='P3 低层工业建筑':x,y=factory_north_tree_point(x,y)
            if name=='P4 特色建筑':x,y=right_island_tree_point(x,y)
            tree(x,y,random.uniform(.095,.135),.95)
    p=ISLANDS[0][1]
    for _ in range(250):
        x=random.uniform(.45,1.24);y=random.uniform(.62,1.96)
        if not inside_polygon(x,y,offset(p,.08)):continue
        if ((x-.78)/.30)**2+((y-.75)/.24)**2<1:continue
        if min(abs(x-.74),abs(x-1.11))<.07 or min(abs(y-1.16),abs(y-1.56))<.06:continue
        if any(math.hypot(x-a,y-b)<.105 for a,b,_ in TREE_POS):continue
        tree(x,y,random.uniform(.15,.24),1.15)
    cx,cy=CFG['round_island_centre_assumed']
    for x,y in [(cx,cy),(cx-.14,cy),(cx+.13,cy+.04),(cx-.06,cy+.14),(cx+.05,cy-.13),(cx-.14,cy-.12)]:tree(x,y,random.uniform(.17,.23),1.2)
    # Rows along the perimeter, limited to the flat middle straight sections.
    for x in [.055,W-.055]:
        for y in np.arange(1.3,3.68,.13):tree(x,y,.095,.7)
    for x in np.arange(.43,W-.42,.13):
        y=yard_front_y(x)
        # Keep the two gate mouths and the boundary lamps clear.
        keep=not any(a<=x<=b for a,b in YARD_OPENINGS) and all(abs(x-l)>.035 for l in YARD_GATE_LAMPS)
        ty=y+(.036 if x<join_x or x>right_join_x else -.016)
        tree(x,ty,.095,.8,emit=keep,base_height=PARKING_Z if x>right_join_x else None)

def lamp(x,y,angle=0,h=.43,base_height=None):
    z=gz(x,y,.012) if base_height is None else base_height+.012;dx,dy=math.cos(angle),math.sin(angle)
    box(x,y,z,.035,.035,.012,'street_furniture','steel','furniture')
    pts=[]
    for t in np.linspace(0,1,12):
        bend=.064*t**3;pts.append((x+dx*bend,y+dy*bend,z+h*t))
    for a,b in zip(pts[:-1],pts[1:]):beam(a,b,.004,'street_furniture','white',None,n=6,r2=.0035)
    ellipsoid(pts[-1][0],pts[-1][1],z+h,.016,.016,.04,'street_furniture','white',n=8,rings=5)
    beam((x,y,z),(x,y,z+h*.75),.004,'street_furniture','white','furniture',n=6)
    box(x,y-.003,z+h*.45,.078,.002,.046,'street_furniture','red')
    # One small yellow emblem on each flag, kept geometric for offline materials.
    ellipsoid(x-.023,y-.0045,z+h*.479,.005,.001,.005,'street_furniture','yellow',n=5,rings=3)

def signal(x,y,angle):
    z=gz(x,y,.01);h=.25;dx,dy=math.cos(angle),math.sin(angle)
    beam((x,y,z),(x,y,z+h),.006,'street_furniture','steel','furniture',n=8)
    beam((x,y,z+h),(x+.24*dx,y+.24*dy,z+h),.005,'street_furniture','steel',None,n=8)
    cx=x+.16*dx;cy=y+.16*dy
    box(cx,cy,z+h-.024,.19,.032,.049,'street_furniture','black',None,yaw=angle)
    for i in [-1,0,1]:
        px=cx+dx*i*.05;py=cy+dy*i*.05
        ellipsoid(px+dy*.018,py-dx*.018,z+h,.012,.012,.012,'street_furniture','light_green' if i==1 else 'steel',n=8,rings=4)

def gantry(x):
    z=.13;y0,y1=5.025,5.39;h=.44
    for y in [y0,y1]:
        box(x,y,z,.045,.045,h,'street_furniture','white','furniture')
        box(x,y,z,.065,.065,.02,'street_furniture','steel')
    for xx in [x-.035,x+.035]:
        for zz in [z+h,z+h+.045]:beam((xx,y0,zz),(xx,y1,zz),.003,'street_furniture','steel',n=5)
        ys=np.linspace(y0,y1,9)
        for i in range(8):
            beam((xx,ys[i],z+h),(xx,ys[i+1],z+h+.045),.0022,'street_furniture','steel',n=4)
            beam((xx,ys[i],z+h+.045),(xx,ys[i+1],z+h),.0022,'street_furniture','steel',n=4)
    for y in np.linspace(y0,y1,7):beam((x-.035,y,z+h+.045),(x+.035,y,z+h+.045),.0022,'street_furniture','steel',n=4)
    box(x+.06,y0+.025,z,.035,.045,.10,'street_furniture','yellow','furniture')

def yard_gate(post_x,far_x,inward):
    """Barrier gate closing one vehicle opening of the grey yard.

    One post only: post_x is the main post, standing on the yard corner where
    the old fence ended, and the boom cantilevers from it across the opening as
    far as far_x, the first fence bay that remains. Upright and boom are one
    member folded at boom height, so nothing stands above the corner. inward is
    +1/-1 towards the middle of the yard, the side the cabinet stands on.
    """
    py,fy=yard_front_y(post_x),yard_front_y(far_x);boom=.215
    box(post_x,py,.002,.038,.055,.10,'street_furniture','yellow','furniture')
    box(post_x+inward*.065,py+.03,.002,.065,.08,.05,'street_furniture','white','furniture')
    box(post_x+inward*.065,py+.03,.052,.075,.09,.009,'street_furniture','roof')
    beam((post_x,py,.10),(post_x,py,boom),.0055,'street_furniture','white',None,n=6)
    for z in [.126,.178]:
        beam((post_x,py,z),(post_x,py,z+.024),.0062,'street_furniture','red',None,n=6)
    beam((post_x,py,boom),(far_x,fy,boom),.0055,'street_furniture','white',None,n=6)
    for t in [.2,.5,.8]:
        beam((post_x+(far_x-post_x)*(t-.04),py+(fy-py)*(t-.04),boom),
             (post_x+(far_x-post_x)*(t+.04),py+(fy-py)*(t+.04),boom),.0062,'street_furniture','red',None,n=6)

def left_parking_gate():
    """Raised single boom and a separate glazed booth at the other jamb."""
    p=PARKING_LEFT;x,y=p['gate_base'];bx,by=p['booth_position']
    box(x,y,.002,.050,.063,.009,'street_furniture','yellow','furniture')
    box(x,y,.011,.034,.045,.090,'street_furniture','yellow','furniture')
    box(x,y,.101,.036,.047,.005,'street_furniture','roof')
    pivot=np.asarray([x+.010,y,.096]);direction=np.asarray([bx-x,by-y]);direction/=np.linalg.norm(direction)
    angle=math.radians(p['gate_open_angle_deg'])
    delta=p['gate_arm_length']*np.asarray([direction[0]*math.cos(angle),direction[1]*math.cos(angle),math.sin(angle)])
    beam(pivot,pivot+delta,.0035,'street_furniture','white','furniture',n=6)
    for t in [.13,.34,.55,.76,.94]:
        beam(pivot+delta*(t-.035),pivot+delta*(t+.035),.0038,'street_furniture','red',n=6)
    box(bx,by,.002,.071,.066,.012,'street_furniture','white','furniture')
    box(bx,by,.014,.062,.057,.075,'street_furniture','white','furniture')
    box(bx,by,.089,.077,.072,.009,'street_furniture','roof')
    # Dark glazing makes the booth distinct from a plain control cabinet.
    for sy in [-1,1]:box(bx,by+sy*.029,.046,.046,.0015,.029,'street_furniture','glass')
    for sx in [-1,1]:box(bx+sx*.0315,by,.046,.0015,.042,.029,'street_furniture','glass')

def right_parking_gate():
    p=RIGHT_D;x,y=p['gate_base'];bx,by=p['booth_position']
    z=PARKING_Z
    box(x,y,z,.050,.063,.009,'street_furniture','yellow','furniture')
    box(x,y,z+.009,.034,.045,.090,'street_furniture','yellow','furniture')
    box(x,y,z+.099,.036,.047,.005,'street_furniture','roof')
    pivot=np.asarray([x+.010,y,z+.094]);direction=np.asarray([bx-x,by-y]);direction/=np.linalg.norm(direction)
    angle=math.radians(p['gate_open_angle_deg'])
    delta=p['gate_arm_length']*np.asarray([direction[0]*math.cos(angle),direction[1]*math.cos(angle),math.sin(angle)])
    beam(pivot,pivot+delta,.0035,'street_furniture','white','furniture',n=6)
    for t in [.13,.34,.55,.76,.94]:
        beam(pivot+delta*(t-.035),pivot+delta*(t+.035),.0038,'street_furniture','red',n=6)
    # The booth's plinth rests on the same plane as the entire parking yard.
    top=PARKING_Z+.004
    box(bx,by,PARKING_Z,.072,.068,.004,'street_furniture','white','furniture')
    box(bx,by,top,.062,.057,.075,'street_furniture','white','furniture')
    box(bx,by,top+.075,.077,.072,.009,'street_furniture','roof')
    for sy in [-1,1]:box(bx,by+sy*.029,top+.032,.046,.0015,.029,'street_furniture','glass')
    for sx in [-1,1]:box(bx+sx*.0315,by,top+.032,.0015,.042,.029,'street_furniture','glass')

def build_furniture():
    for x,y,a in [(.35,.9,0),(.35,2.0,0),(.35,3.15,0),(1.28,.78,math.pi),(1.28,1.90,math.pi),(*PARKING_LEFT['factory_north_lamp_position'],math.pi),
                    (W-.35,.9,math.pi),(W-.35,2.0,math.pi),(W-.35,3.15,math.pi),(2.02,.8,0),(2.02,1.94,0),*RIGHT_D['island_lamps'],
                    (YARD_GATE_LAMPS[0],yard_front_y(YARD_GATE_LAMPS[0]),0),
                    (YARD_GATE_LAMPS[1],yard_front_y(YARD_GATE_LAMPS[1]),math.pi),
                    (1.65,4.5,math.pi/2)]:lamp(x,y,a,base_height=PARKING_Z if x>right_join_x and y>3.8 else None)
    for x,y,a in [(.41,2.05,0),(1.26,2.06,math.pi/2),(2.04,2.77,-math.pi/2),(2.89,2.77,math.pi),
                  (.42,2.77,0),(2.88,2.05,math.pi),(.075,3.37,0),(3.225,3.50,math.pi)]:signal(x,y,a)
    for x in [1.0,2.3]:gantry(x)
    # Both entrances use their independently observed gate and booth positions.
    left_parking_gate()
    right_parking_gate()

def write_obj(path,mesh,material=None):
    v,f,n=mesh.arrays()
    with path.open('w',encoding='utf-8',newline='\n') as out:
        out.write('# Dongfeng sandbox; metres; X right, Y far, Z up\n')
        if material:out.write(f'mtllib scene.mtl\nusemtl {material}\n')
        for a in v:out.write('v %.6f %.6f %.6f\n'%tuple(a))
        for a in n:out.write('vn %.6f %.6f %.6f\n'%tuple(a))
        for tri in f:
            out.write('f '+' '.join(f'{int(i)+1}//{int(i)+1}' for i in tri)+'\n')

def rgb(hexval):return [int(hexval[i:i+2],16)/255 for i in (1,3,5)]

def write_glb(path,arrays):
    binary=bytearray();views=[];access=[];meshes=[];nodes=[];materials=[];matids={}
    def pack_array(a,typ,component):
        nonlocal binary
        while len(binary)%4:binary.append(0)
        start=len(binary);data=a.tobytes();binary.extend(data)
        views.append(dict(buffer=0,byteOffset=start,byteLength=len(data)))
        info=dict(bufferView=len(views)-1,componentType=component,count=len(a),type=typ)
        if typ=='VEC3':info.update(min=a.min(axis=0).tolist(),max=a.max(axis=0).tolist())
        access.append(info);return len(access)-1
    for (layer,mat),(v,f,n) in arrays.items():
        if mat not in matids:
            color=rgb(PALETTE[mat]);linear=[((a+.055)/1.055)**2.4 if a>.04045 else a/12.92 for a in color]
            materials.append(dict(name=mat,pbrMetallicRoughness=dict(baseColorFactor=linear+[1],metallicFactor=.18 if mat in ('glass','glass_light','steel') else 0,roughnessFactor=.7),doubleSided=False))
            matids[mat]=len(materials)-1
        # glTF Y-up, Gazebo Z-up. Use one root node rotating the whole model.
        vi=pack_array(v.astype('<f4'),'VEC3',5126);ni=pack_array(n.astype('<f4'),'VEC3',5126);fi=pack_array(f.astype('<u4').reshape(-1),'SCALAR',5125)
        meshes.append(dict(name=f'{layer}_{mat}',primitives=[dict(attributes=dict(POSITION=vi,NORMAL=ni),indices=fi,material=matids[mat])]))
        nodes.append(dict(name=f'{layer}_{mat}',mesh=len(meshes)-1))
    rootidx=len(nodes);nodes.append(dict(name='Dongfeng sandbox 3.3m x 5.4m',children=list(range(rootidx)),rotation=[-math.sqrt(.5),0,0,math.sqrt(.5)]))
    doc=dict(asset=dict(version='2.0',generator='Dongfeng sandbox procedural generator'),scene=0,scenes=[dict(nodes=[rootidx])],nodes=nodes,meshes=meshes,materials=materials,buffers=[dict(byteLength=len(binary))],bufferViews=views,accessors=access,extras=dict(sourceUnits='metres',sourceAxes='X right, Y far, Z up',config=CFG))
    js=json.dumps(doc,separators=(',',':'),ensure_ascii=True).encode();js+=b' '*((-len(js))%4);binary+=b'\0'*((-len(binary))%4)
    content=struct.pack('<III',0x46546C67,2,12+8+len(js)+8+len(binary))+struct.pack('<II',len(js),0x4E4F534A)+js+struct.pack('<II',len(binary),0x004E4942)+binary
    path.write_bytes(content)

def write_sdf(model_dir):
    root=ET.Element('sdf',version='1.9');model=ET.SubElement(root,'model',name='dongfeng_sandbox');ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',name='site')
    for layer,mat in BUCKETS:
        visual=ET.SubElement(link,'visual',name=f'{layer}_{mat}');geo=ET.SubElement(visual,'geometry');mesh=ET.SubElement(geo,'mesh')
        ET.SubElement(mesh,'uri').text=f'model://dongfeng_sandbox/meshes/{layer}_{mat}.obj'
        material=ET.SubElement(visual,'material');color=' '.join(f'{a:.5f}' for a in rgb(PALETTE[mat]))+' 1'
        ET.SubElement(material,'ambient').text=color;ET.SubElement(material,'diffuse').text=color
        ET.SubElement(material,'specular').text='0.15 0.15 0.15 1';ET.SubElement(visual,'cast_shadows').text='true'
    for name in COLLISION:
        co=ET.SubElement(link,'collision',name=name);geo=ET.SubElement(co,'geometry');mesh=ET.SubElement(geo,'mesh');ET.SubElement(mesh,'uri').text=f'model://dongfeng_sandbox/meshes/collision_{name}.obj'
        surf=ET.SubElement(co,'surface');fr=ET.SubElement(surf,'friction');ode=ET.SubElement(fr,'ode');ET.SubElement(ode,'mu').text='0.9';ET.SubElement(ode,'mu2').text='0.9'
    ET.indent(root);ET.ElementTree(root).write(model_dir/'model.sdf',encoding='utf-8',xml_declaration=True)
    (model_dir/'model.config').write_text('<?xml version="1.0"?>\n<model><name>Dongfeng sandbox</name><version>1.0</version><sdf version="1.9">model.sdf</sdf><author><name>Course experiment</name></author><description>Measured 3.3 x 5.4 m sandbox; buildings and layout approximated from supplied photos.</description></model>\n',encoding='utf-8')
    world=ET.Element('sdf',version='1.9');w=ET.SubElement(world,'world',name='dongfeng_world');ET.SubElement(w,'gravity').text='0 0 -9.81'
    physics=ET.SubElement(w,'physics',name='physics',type='ignored');ET.SubElement(physics,'max_step_size').text='0.001';ET.SubElement(physics,'real_time_factor').text='1'
    for filename,name in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors')]:
        plug=ET.SubElement(w,'plugin',filename=f'gz-sim-{filename}-system',name=f'gz::sim::systems::{name}')
        if filename=='sensors':ET.SubElement(plug,'render_engine').text='ogre2'
    scene=ET.SubElement(w,'scene');ET.SubElement(scene,'ambient').text='0.65 0.65 0.65 1';ET.SubElement(scene,'background').text='0.88 0.9 0.92 1';ET.SubElement(scene,'shadows').text='true'
    sun=ET.SubElement(w,'light',name='sun',type='directional');ET.SubElement(sun,'pose').text='0 0 10 0 0 0';ET.SubElement(sun,'diffuse').text='0.85 0.85 0.85 1';ET.SubElement(sun,'specular').text='0.15 0.15 0.15 1';ET.SubElement(sun,'direction').text='-0.5 0.3 -0.9';ET.SubElement(sun,'cast_shadows').text='true'
    inc=ET.SubElement(w,'include');ET.SubElement(inc,'uri').text='model://dongfeng_sandbox'
    gui=ET.SubElement(w,'gui',fullscreen='0');view=ET.SubElement(gui,'plugin',filename='MinimalScene',name='3D View')
    ET.SubElement(view,'engine').text='ogre2';ET.SubElement(view,'scene').text='scene';ET.SubElement(view,'camera_pose').text='4 -5 6 0 0.60 1.28'
    ET.SubElement(gui,'plugin',filename='GzSceneManager',name='Scene Manager');ET.SubElement(gui,'plugin',filename='InteractiveViewControl',name='Interactive view control');ET.SubElement(gui,'plugin',filename='EntityTree',name='Entity tree');ET.SubElement(gui,'plugin',filename='ComponentInspector',name='Component inspector')
    control=ET.SubElement(gui,'plugin',filename='WorldControl',name='World control');ET.SubElement(control,'play_pause').text='true';ET.SubElement(control,'start_paused').text='false';ET.SubElement(control,'service').text='/world/dongfeng_world/control';ET.SubElement(control,'stats_topic').text='/world/dongfeng_world/stats'
    ET.indent(world);ET.ElementTree(world).write(ROOT/'worlds'/'dongfeng.sdf',encoding='utf-8',xml_declaration=True)

def validate(arrays):
    checks={};checks['finite_geometry']=all(np.isfinite(v).all() and np.isfinite(n).all() for v,f,n in arrays.values())
    checks['valid_triangle_indices']=all(int(f.max())<len(v) for v,f,n in arrays.values())
    # Exact source geometry constraints, independent of photo-estimated placement.
    checks['interior_dimensions_m']=[W,L]
    widths=[]
    for x,y,nx,ny,z,seg in ROAD:
        h=CFG['one_way_width']/2
        widths.append(math.dist((x+h*nx,y+h*ny),(x-h*nx,y-h*ny)))
    checks['perimeter_width_min_max_m']=[min(widths),max(widths)]
    checks['outer_curve_radius_m']=CFG['outer_road_radius']
    checks['inner_curve_radius_m']=CFG['outer_road_radius']-CFG['one_way_width']
    checks['central_white_line_spacing_m']=CFG['central_road_width']
    checks['front_curve_height_m']=CFG['front_curve_height']
    checks['front_straight_height_min_max_m']=[min(p[4] for p in ROAD if p[5]=='AB'),max(p[4] for p in ROAD if p[5]=='AB')]
    checks['front_curve_peaks_m']={name:max(p[4] for p in ROAD if p[5]==name) for name in ('A','B')}
    checks['front_curve_profile']='Zero-height straight joins, smooth 5 cm crest at each arc midpoint.'
    assert checks['front_straight_height_min_max_m']==[0.0,0.0]
    assert all(abs(h-CFG['front_curve_height'])<1e-10 for h in checks['front_curve_peaks_m'].values())
    checks['rear_height_m']=CFG['rear_road_height']
    checks['rear_straight_slope_length_m']=math.hypot(ROAD_INFO['straight_slope_run'],ROAD_INFO['straight_slope_rise'])
    checks['rear_straight_slope_start_y_m']=ROAD_INFO['straight_slope_start_y']
    checks['rear_straight_slope_reference']=ROAD_INFO['straight_slope_reference']
    checks['rear_straight_horizontal_run_m']=ROAD_INFO['straight_slope_run']
    checks['rear_straight_grade_degrees']=math.degrees(math.atan2(ROAD_INFO['straight_slope_rise'],ROAD_INFO['straight_slope_run']))
    checks['rear_curve_profile']='Quintic height over Y: incoming straight grade matched at both road edges, easing to a level rear platform.'
    checks['maximum_adjacent_road_height_step_m']=max(abs(ROAD[(i+1)%len(ROAD)][4]-p[4]) for i,p in enumerate(ROAD))
    checks['closing_segment_length_m']=math.dist(ROAD[0][:2],ROAD[-1][:2])
    checks['closing_segment_note']='The final section is connected to the first by mesh faces; this is a segment length, not a gap.'
    checks['road_cross_sections']=len(ROAD)
    checks['visual_mesh_groups']=len(arrays);checks['visual_triangles']=sum(len(f) for v,f,n in arrays.values());checks['trees']=len(TREE_POS)
    checks['collision_triangles']=sum(len(m.f) for m in COLLISION.values())
    checks['gazebo_runtime_test']='Not run: this generator performs static geometry checks only.'
    assert checks['finite_geometry'] and checks['valid_triangle_indices']
    assert abs(min(widths)-.30)<1e-8 and abs(max(widths)-.30)<1e-8
    for side in ['BC','DA']:
        points=[p for p in ROAD if p[5]==side]
        assert any(abs(p[1]-ROAD_INFO['straight_slope_start_y'])<1e-10 and p[4]==0 for p in points)
        assert all(p[4]==0 for p in points if p[1]<=ROAD_INFO['straight_slope_start_y'])
        assert all(abs(p[4]-rear_straight_height(p[1]))<1e-10 for p in points)
    refs=[]
    for path in [ROOT/'worlds'/'dongfeng.sdf',ROOT/'models'/'dongfeng_sandbox'/'model.sdf']:
        root=ET.parse(path).getroot()
        for elem in root.iter('uri'):
            if elem.text.startswith('model://'):
                target=ROOT/'models'/elem.text[8:];assert target.exists(),str(target);refs.append(str(target.relative_to(ROOT)))
    checks['sdf_xml_and_asset_references']='passed';checks['resolved_references']=len(refs)
    REPORT_DIR.mkdir(parents=True,exist_ok=True)
    (REPORT_DIR/'validation.json').write_text(json.dumps(checks,indent=2,ensure_ascii=False),encoding='utf-8')
    return checks

def export_all():
    model_dir=ROOT/'models'/'dongfeng_sandbox';mesh_dir=model_dir/'meshes';mesh_dir.mkdir(parents=True,exist_ok=True);(ROOT/'worlds').mkdir(exist_ok=True)
    for directory in (EXPORT_DIR, PREVIEW_DIR):
        directory.mkdir(parents=True,exist_ok=True)
    arrays={k:m.arrays() for k,m in BUCKETS.items() if m.f}
    for (layer,mat),m in BUCKETS.items():write_obj(mesh_dir/f'{layer}_{mat}.obj',m,mat)
    for name,m in COLLISION.items():write_obj(mesh_dir/f'collision_{name}.obj',m)
    mtl=[]
    for mat,color in PALETTE.items():mtl.append(f'newmtl {mat}\nKd '+' '.join(map(str,rgb(color)))+'\nKa 0.2 0.2 0.2\nKs 0.12 0.12 0.12\nNs 20\n')
    (mesh_dir/'scene.mtl').write_text('\n'.join(mtl),encoding='utf-8')
    # A single easy-to-import OBJ alongside the model, retaining layer/material groups.
    with (EXPORT_DIR/'dongfeng_sandbox.obj').open('w',encoding='utf-8',newline='\n') as out:
        out.write('mtllib dongfeng_sandbox.mtl\n');count=0
        for (layer,mat),(v,f,n) in arrays.items():
            out.write(f'o {layer}_{mat}\nusemtl {mat}\n')
            for a in v:out.write('v %.6f %.6f %.6f\n'%tuple(a))
            for a in n:out.write('vn %.6f %.6f %.6f\n'%tuple(a))
            for tri in f:out.write('f '+' '.join(f'{int(i)+1+count}//{int(i)+1+count}' for i in tri)+'\n')
            count+=len(v)
    (EXPORT_DIR/'dongfeng_sandbox.mtl').write_text('\n'.join(mtl),encoding='utf-8')
    write_glb(EXPORT_DIR/'dongfeng_sandbox.glb',arrays);write_sdf(model_dir)
    # Compact base64 typed arrays, reused verbatim by the WebGL preview.
    import base64
    data=[]
    for (layer,mat),(v,f,n) in arrays.items():
        enc=lambda a:base64.b64encode(a.tobytes()).decode('ascii')
        data.append(dict(layer=layer,material=mat,color=PALETTE[mat],p=enc(v.astype('<f4')),n=enc(n.astype('<f4')),i=enc(f.astype('<u4'))))
    (PREVIEW_DIR/'scene-data.json').write_text(json.dumps(dict(meshes=data,features=FEATURES,config=CFG),separators=(',',':'),ensure_ascii=False),encoding='utf-8')
    report=validate(arrays)
    print(json.dumps(report,indent=2,ensure_ascii=True))

if __name__=='__main__':
    build_ground();build_road();build_islands();build_markings();build_park();build_buildings();build_trees();build_furniture();export_all()
