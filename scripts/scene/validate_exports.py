"""Read the saved GLB / OBJ files independently of the generator (numpy only)."""
from pathlib import Path
from collections import Counter
import json, struct
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
EXPORT_DIR=ROOT/'exports/scene'
REPORT_DIR=ROOT/'reports/scene'

def read_obj(path):
    vertices=[];faces=[]
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.startswith('v '):vertices.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('f '):faces.append([int(x.split('/')[0])-1 for x in line.split()[1:]])
    v=np.asarray(vertices,float);f=np.asarray(faces,int)
    assert f.shape[1]==3 and f.min()>=0 and f.max()<len(v)
    assert np.isfinite(v).all()
    area2=np.linalg.norm(np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]]),axis=1)
    return v,f,int(np.sum(area2<1e-12))


def validate_front_road():
    """Check the requested heights in saved meshes, including hidden terrain."""
    cfg=json.loads((ROOT/'config/scene/scene_config.json').read_text(encoding='utf-8'))
    mesh_dir=ROOT/'models/dongfeng_sandbox/meshes'
    v,f,_=read_obj(mesh_dir/'elevated_road_asphalt.obj')
    e=cfg['road_edge_inset_assumed'];radius=cfg['outer_road_radius']
    width=cfg['one_way_width'];total_width=cfg['interior_width']
    xl,xr,yb=e+radius,total_width-e-radius,e+radius
    tol=2e-6
    straight=(v[:,0]>=xl-tol)&(v[:,0]<=xr+tol)&(v[:,1]<=e+width+tol)
    assert straight.any() and np.all(np.abs(v[straight,2])<tol)
    peaks={}
    for name,mask,sign,cx in [
        ('A',(v[:,0]<=xl+tol)&(v[:,1]<=yb+tol),-1,xl),
        ('B',(v[:,0]>=xr-tol)&(v[:,1]<=yb+tol),1,xr)]:
        points=v[mask]
        assert len(points)>4 and abs(points[:,2].min())<tol
        assert abs(points[:,2].max()-cfg['front_curve_height'])<tol
        # Crest must be on the 45-degree radial cross-section, at both edges.
        crest=points[np.abs(points[:,2]-cfg['front_curve_height'])<tol]
        assert len(crest)==2 and np.allclose(np.abs(crest[:,0]-cx),yb-crest[:,1],atol=tol)
        assert np.all(np.abs(points[np.abs(points[:,1]-yb)<tol,2])<tol)
        peaks[name]=float(points[:,2].max())
    # No leftover elevated platform may occlude the lowered front road.
    terrain,_,_=read_obj(mesh_dir/'collision_terrain.obj')
    assert np.allclose(terrain[:,2],-.001,atol=tol)
    # Collision top must exactly match the visible graded deck everywhere.
    collision,cf,_=read_obj(mesh_dir/'collision_road_deck.obj')
    triangles=collision[cf]
    normal=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    top=np.unique(triangles[normal[:,2]>1e-12].reshape(-1,3),axis=0)
    expected=np.unique(v,axis=0)
    assert top.shape==expected.shape and np.allclose(top,expected,atol=tol)
    return {'AB_straight_height_min_max_m':[float(v[straight,2].min()),float(v[straight,2].max())],
            'arc_crest_height_m':peaks,'arc_tangent_join_height_m':0.0,
            'front_terrain_elevation_m':-.001,'visual_and_collision_deck_match':True,
            'peak_location':'midpoint of each A/B quarter-circle'}

def main():
    raw=(EXPORT_DIR/'dongfeng_sandbox.glb').read_bytes()
    magic,version,total=struct.unpack_from('<III',raw)
    assert magic==0x46546c67 and version==2 and total==len(raw)
    size,kind=struct.unpack_from('<II',raw,12);assert kind==0x4e4f534a
    doc=json.loads(raw[20:20+size]);start=20+size
    size,kind=struct.unpack_from('<II',raw,start);assert kind==0x004e4942
    binary=raw[start+8:];assert len(binary)==size
    assert doc['buffers'][0]['byteLength']<=len(binary)
    decoded=[]
    for acc in doc['accessors']:
        view=doc['bufferViews'][acc['bufferView']]
        offset=view.get('byteOffset',0)+acc.get('byteOffset',0)
        width={'SCALAR':1,'VEC3':3}[acc['type']]
        dtype={5126:'<f4',5125:'<u4'}[acc['componentType']]
        length=acc['count']*width*4
        assert offset%4==0 and length<=view['byteLength'] and offset+length<=len(binary)
        a=np.frombuffer(binary,dtype=dtype,count=acc['count']*width,offset=offset)
        if width>1:a=a.reshape(-1,width)
        assert np.isfinite(a).all()
        if 'min' in acc:
            assert np.allclose(a.min(axis=0),acc['min'])
            assert np.allclose(a.max(axis=0),acc['max'])
        decoded.append(a)
    visual_count=0;zero_area=0
    glb_vertices=[]
    for mesh in doc['meshes']:
        for primitive in mesh['primitives']:
            v=decoded[primitive['attributes']['POSITION']]
            n=decoded[primitive['attributes']['NORMAL']]
            f=decoded[primitive['indices']].reshape(-1,3)
            assert f.max()<len(v) and len(v)==len(n)
            area2=np.linalg.norm(np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]]),axis=1)
            zero_area+=int(np.sum(area2<1e-12));visual_count+=len(f);glb_vertices.append(v)
    obj_v,obj_f,obj_zero=read_obj(EXPORT_DIR/'dongfeng_sandbox.obj')
    assert len(obj_f)==visual_count
    assert np.allclose(np.concatenate(glb_vertices),obj_v,atol=6e-7,rtol=0)
    assert zero_area==0 and obj_zero==0
    collision_reports={}
    for path in sorted((ROOT/'models/dongfeng_sandbox/meshes').glob('collision_*.obj')):
        v,f,zero=read_obj(path)
        assert zero==0, str(path)
        collision_reports[path.stem]={'triangles':len(f),'zero_area_triangles':zero}
        if path.stem=='collision_road_deck':
            # Weld duplicate OBJ positions, then check closure and oriented edges.
            _,inverse=np.unique(np.round(v,6),axis=0,return_inverse=True)
            triangles=inverse[f];edges=Counter();orientation=Counter()
            for tri in triangles:
                for a,b in zip(tri,np.roll(tri,-1)):
                    key=tuple(sorted((int(a),int(b))))
                    edges[key]+=1;orientation[key]+=1 if a<b else -1
            boundary=sum(n==1 for n in edges.values())
            nonmanifold=sum(n!=2 for n in edges.values())
            inconsistent=sum(n!=0 for n in orientation.values())
            collision_reports[path.stem].update(boundary_edges=boundary,nonmanifold_edges=nonmanifold,inconsistent_winding_edges=inconsistent)
            assert boundary==0 and nonmanifold==0 and inconsistent==0
    report={'glb_header_accessors_and_bounds':'passed','glb_and_obj_same_geometry':'passed',
            'visual_triangles':visual_count,'glb_zero_area_triangles':zero_area,
            'obj_zero_area_triangles':obj_zero,'collision_meshes':collision_reports,
            'front_road_height_checks':validate_front_road(),
            'note':'Static export checks only. This is not a Gazebo physics or robot navigation runtime test.'}
    REPORT_DIR.mkdir(parents=True,exist_ok=True)
    (REPORT_DIR/'export_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=True,indent=2))

if __name__=='__main__':main()
