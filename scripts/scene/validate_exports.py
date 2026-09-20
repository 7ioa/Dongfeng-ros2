"""Read the saved GLB / OBJ files independently of the generator (numpy only)."""
from pathlib import Path
from collections import Counter
import json, struct, xml.etree.ElementTree as ET
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

def validate_rear_road():
    """Read back both grades and check their common gate-aligned start."""
    cfg=json.loads((ROOT/'config/scene/scene_config.json').read_text(encoding='utf-8'))
    reference=cfg['rear_slope_start_reference']
    filename={'left_parking_gate':'parking_left.json','right_parking_gate':'right_d.json'}[reference]
    gate=json.loads((ROOT/'config/scene'/filename).read_text(encoding='utf-8'))
    start=gate['gate_base'][1];e=cfg['road_edge_inset_assumed'];width=cfg['one_way_width']
    y0=e+cfg['outer_road_radius'];end=cfg['interior_length']-e-cfg['outer_road_radius']
    rise=cfg['rear_straight_rise_assumed'];tol=2e-6
    assert y0<start<end
    v,faces,_=read_obj(ROOT/'models/dongfeng_sandbox/meshes/elevated_road_asphalt.obj')
    profiles=[]
    for edges in [(e,e+width),(cfg['interior_width']-e-width,cfg['interior_width']-e)]:
        mask=(np.isclose(v[:,0],edges[0],atol=tol,rtol=0)|np.isclose(v[:,0],edges[1],atol=tol,rtol=0))&(v[:,1]>=y0-tol)&(v[:,1]<=end+tol)
        points=np.unique(v[mask,1:],axis=0);assert len(points)>100
        at_start=np.abs(points[:,0]-start)<tol
        assert at_start.any() and np.all(np.abs(points[at_start,1])<tol)
        flat=points[:,0]<=start+tol
        assert np.all(np.abs(points[flat,1])<tol)
        expected=np.maximum(0,points[:,0]-start)*rise/(end-start)
        assert np.allclose(points[:,1],expected,atol=tol,rtol=0)
        assert abs(points[:,1].max()-rise)<tol
        profiles.append(points)
    assert profiles[0].shape==profiles[1].shape and np.allclose(*profiles,atol=tol,rtol=0)
    # Check actual inner/outer arc edges, not just a centreline formula.
    grade=rise/(end-start);arcs={};profiles_by_radius={}
    for name,cx,sign in [('left',y0,-1),('right',cfg['interior_width']-y0,1)]:
        arcs[name]={}
        for edge,radius in [('inner',cfg['outer_road_radius']-width),('outer',cfg['outer_road_radius'])]:
            radial=np.hypot(v[:,0]-cx,v[:,1]-end)
            mask=(sign*(v[:,0]-cx)>=-tol)&(v[:,1]>=end-tol)&(np.abs(radial-radius)<tol)
            points=np.unique(v[mask],axis=0)
            angle=np.arctan2(points[:,1]-end,sign*(points[:,0]-cx))
            points=points[np.argsort(angle)];assert len(points)>=60
            assert abs(points[0,2]-rise)<tol and abs(points[-1,2]-cfg['rear_road_height'])<tol
            assert np.all(np.diff(points[:,2])>=-tol)
            assert points[:,2].max()<=cfg['rear_road_height']+tol
            incoming=(points[1,2]-points[0,2])/np.linalg.norm(points[1,:2]-points[0,:2])
            outgoing=(points[-1,2]-points[-2,2])/np.linalg.norm(points[-1,:2]-points[-2,:2])
            join_error=float(np.degrees(abs(np.arctan(incoming)-np.arctan(grade))))
            assert join_error<.10 and abs(outgoing)<.0002,(name,edge,join_error,outgoing)
            if edge in profiles_by_radius:assert np.allclose(points[:,1:],profiles_by_radius[edge],atol=tol,rtol=0)
            else:profiles_by_radius[edge]=points[:,1:]
            arcs[name][edge]={'incoming_grade_angle_error_deg':join_error,'outgoing_grade':float(outgoing)}
    triangles=v[faces]
    join=(np.abs(triangles[:,:,1].min(axis=1)-end)<tol)&(triangles[:,:,1].max(axis=1)<end+.03)
    joined=triangles[join];assert len(joined)==4
    normals=np.cross(joined[:,1]-joined[:,0],joined[:,2]-joined[:,0]);normals/=np.linalg.norm(normals,axis=1)[:,None]
    straight_normal=np.array([0.,-grade,1.]);straight_normal/=np.linalg.norm(straight_normal)
    face_error=float(np.degrees(np.arccos(np.clip(normals@straight_normal,-1,1))).max())
    assert face_error<.12,face_error
    return {'reference':reference,'shared_start_y_m':start,'flat_before_start':True,'left_right_profiles_identical':True,
            'straight_horizontal_run_m':end-start,'straight_slope_length_m':float(np.hypot(end-start,rise)),
            'straight_rise_m':rise,'final_rear_height_m':float(v[:,2].max()),
            'arc_edge_tangency_checks':arcs,'maximum_straight_to_arc_face_angle_deg':face_error,
            'arcs_monotone_without_overshoot':True,'arcs_end_level':True}

def validate_parking_yard():
    """Check the saved full yard surface and its physical support, not a sample."""
    mesh_dir=ROOT/'models/dongfeng_sandbox/meshes';height=.002;tol=1e-6
    v,f,_=read_obj(mesh_dir/'yard_grey_yard.obj')
    assert np.all(np.abs(v[:,2]-height)<tol), 'Parking yard must be horizontal everywhere'
    triangles=v[f]
    normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
    assert np.all(normals[:,2]>0) and np.all(np.abs(normals[:,:2])<1e-12)
    # The boundary polygon must be fully filled, including its concave front.
    area=.5*abs(np.sum(v[:,0]*np.roll(v[:,1],-1)-np.roll(v[:,0],-1)*v[:,1]))
    assert abs(normals[:,2].sum()/2-area)<1e-9
    cv,cf,_=read_obj(mesh_dir/'collision_yard.obj');ct=cv[cf]
    cn=np.cross(ct[:,1]-ct[:,0],ct[:,2]-ct[:,0])
    top=ct[cn[:,2]>1e-12]
    assert top.shape==triangles.shape and np.allclose(top,triangles,atol=tol,rtol=0)
    assert abs(cv[:,2].max()-height)<tol and abs(cv[:,2].min()+.014)<tol
    sdf=ET.parse(ROOT/'models/dongfeng_sandbox/model.sdf').getroot()
    uris=[node.text for node in sdf.iter('uri')]
    assert any(uri.endswith('/collision_yard.obj') for uri in uris)
    assert not any('right_entrance' in uri for uri in uris), 'Obsolete raised entrance remains active'
    return {'surface_height_min_max_m':[float(v[:,2].min()),float(v[:,2].max())],
            'height_variation_m':float(np.ptp(v[:,2])),
            'covered_area_m2':float(area),'all_surface_normals_vertical':True,
            'visual_and_collision_top_triangles_identical':True,
            'obsolete_graded_entrance_removed_from_model':True}

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
        if path.stem in ('collision_road_deck','collision_yard'):
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
            'rear_road_height_checks':validate_rear_road(),
            'parking_yard_checks':validate_parking_yard(),
            'note':'Static export checks only. This is not a Gazebo physics or robot navigation runtime test.'}
    REPORT_DIR.mkdir(parents=True,exist_ok=True)
    (REPORT_DIR/'export_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=True,indent=2))

if __name__=='__main__':main()
