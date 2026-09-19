"""Validate exported files independently of the procedural generator (numpy only)."""
from pathlib import Path
import argparse, base64, hashlib, json, math, struct
import xml.etree.ElementTree as ET
import numpy as np

PACKAGE=Path(__file__).resolve().parents[1]
ROOT=PACKAGE.parents[1]

def vec(element,attr,default='0 0 0'):
    return np.fromstring(element.get(attr,default),sep=' ')

def rotation(rpy):
    r,p,y=rpy;cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);cy,sy=math.cos(y),math.sin(y)
    return np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])

def obj(path):
    v=[];n=[];f=[]
    for line in path.read_text().splitlines():
        a=line.split()
        if not a:continue
        if a[0]=='v':v.append([float(x) for x in a[1:]])
        if a[0]=='vn':n.append([float(x) for x in a[1:]])
        if a[0]=='f':
            assert len(a)==4,(path,'non-triangle')
            entries=[x.split('//') for x in a[1:]]
            assert all(x[0]==x[1] for x in entries)
            f.append([int(x[0])-1 for x in entries])
    v,n,f=np.array(v),np.array(n),np.array(f)
    assert np.isfinite(v).all() and np.isfinite(n).all()
    assert len(n)==len(v) and f.min()>=0 and f.max()<len(v)
    cross=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
    assert np.all(np.linalg.norm(cross,axis=1)>1e-14),(path,'degenerate face')
    assert np.allclose(np.linalg.norm(n,axis=1),1,atol=1e-5),(path,'invalid normal')
    return v,n,f

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--scope-baseline',type=Path);parser.add_argument('--check-install',action='store_true');args=parser.parse_args()
    robot=ET.parse(PACKAGE/'urdf/dongfeng_car.urdf.xacro').getroot()
    links={e.get('name'):e for e in robot.findall('link')};joints={e.get('name'):e for e in robot.findall('joint')}
    assert len(links)==len(robot.findall('link')) and len(joints)==len(robot.findall('joint'))
    parents={}
    for name,j in joints.items():
        p=j.find('parent').get('link');c=j.find('child').get('link')
        assert p in links and c in links and c not in parents
        parents[c]=p
    assert set(links)-set(parents)=={'base_link'}
    for name in links:
        visited=set();node=name
        while node in parents:
            assert node not in visited;visited.add(node);node=parents[node]
        assert node=='base_link'
    total_mass=0
    for link in links.values():
        inertial=link.find('inertial')
        if inertial is None:
            assert link.get('name')=='camera_optical_frame';continue
        mass=float(inertial.find('mass').get('value'));assert mass>0;total_mass+=mass
        t=inertial.find('inertia').attrib
        matrix=np.array([[float(t['ixx']),float(t['ixy']),float(t['ixz'])],[float(t['ixy']),float(t['iyy']),float(t['iyz'])],[float(t['ixz']),float(t['iyz']),float(t['izz'])]])
        eig=np.linalg.eigvalsh(matrix);assert eig.min()>0 and eig[-1]<=eig[:2].sum()+1e-12
    drive=robot.find("gazebo/plugin[@name='gz::sim::systems::DiffDrive']")
    radius=float(drive.findtext('wheel_radius'));track=float(drive.findtext('wheel_separation'))
    wheel_joints=[];centres=[]
    for side,sign in [('left',1),('right',-1)]:
        names=[e.text for e in drive.findall(side+'_joint')];assert len(names)==2
        wheel_joints+=names
        for name in names:
            j=joints[name];assert j.get('type')=='continuous';assert np.allclose(vec(j.find('axis'),'xyz'),[0,1,0])
            centre=vec(j.find('origin'),'xyz');centres.append(centre)
            assert np.isclose(centre[1],sign*track/2) and centre[2]==0
            link=links[j.find('child').get('link')];c=link.find('collision');shape=c.find('geometry/cylinder')
            assert np.isclose(float(shape.get('radius')),radius)
            axis=rotation(vec(c.find('origin'),'rpy'))@np.array([0,0,1]);assert np.allclose(axis,[0,1,0],atol=1e-8)
            assert np.allclose(vec(c.find('origin'),'xyz'),[0,0,0])
    assert len(set(wheel_joints))==4 and len({tuple(p) for p in centres})==4
    assert len(set(p[0] for p in centres))==2
    for tag,value in [('topic','/cmd_vel'),('odom_topic','/odom'),('tf_topic','/tf'),('frame_id','odom'),('child_frame_id','base_link')]:assert drive.findtext(tag)==value
    assert robot.find("gazebo/plugin[@name='gz::sim::systems::JointStatePublisher']/topic").text=='/joint_states'
    optical=joints['camera_optical_joint'];assert optical.find('parent').get('link')=='camera_link'
    assert np.allclose(rotation(vec(optical.find('origin'),'rpy'))@np.array([0,0,1]),[1,0,0],atol=1e-8)
    assert vec(joints['camera_joint'].find('origin'),'xyz')[0]>.1
    assert vec(joints['lidar_joint'].find('origin'),'xyz')[0]>0
    assert not robot.findall('.//sensor')
    mesh_files={}
    for m in robot.findall('.//visual/geometry/mesh'):
        uri=m.get('filename');assert uri.startswith('package://dongfeng_description/')
        relative=uri.removeprefix('package://dongfeng_description/');path=PACKAGE/relative
        assert path.is_file();mesh_files[path.stem]=obj(path)
    data=json.loads((ROOT/'previews/vehicle/vehicle-data.json').read_text());triangles=0;points=[]
    for m in data['meshes']:
        # Material names may contain underscores; split with the explicit suffix.
        suffix='_'+m['material'];prefix=m['name'][:-len(suffix)];part,index=prefix.rsplit('_',1)
        v,n,f=mesh_files[part+'_'+m['material']]
        dec=lambda k,d:np.frombuffer(base64.b64decode(m[k]),dtype=d).reshape(-1,3)
        pv,pn,pf=dec('p','<f4'),dec('n','<f4'),dec('i','<u4')
        if part=='base':offset=np.zeros(3)
        elif part in ('camera','lidar'):offset=vec(joints[part+'_joint'].find('origin'),'xyz')
        else:
            # Assembly indices: front left, rear left, front right, rear right.
            order={1:'wheel_left_front_joint',2:'wheel_left_joint',3:'wheel_right_front_joint',4:'wheel_right_joint'}
            offset=vec(joints[order[int(index)]].find('origin'),'xyz')
        assert np.allclose(pv,v+offset,atol=2e-8) and np.array_equal(pf,f)
        assert np.allclose(pn,n,atol=2e-8);triangles+=len(f);points.append(pv)
    points=np.concatenate(points);assert np.isclose(points[:,2].min(),-radius,atol=1e-5)
    # Validate GLB buffer structure and that each exported primitive is complete.
    raw=(ROOT/'exports/vehicle/dongfeng_car.glb').read_bytes();magic,version,length=struct.unpack_from('<III',raw)
    assert magic==0x46546c67 and version==2 and length==len(raw)
    jslen,jstype=struct.unpack_from('<II',raw,12);assert jstype==0x4e4f534a
    glb=json.loads(raw[20:20+jslen]);binlen,bintype=struct.unpack_from('<II',raw,20+jslen);assert bintype==0x004e4942
    assert 28+jslen+binlen==len(raw);assert len(glb['meshes'])==len(data['meshes'])
    assert all(v.get('byteOffset',0)+v['byteLength']<=binlen for v in glb['bufferViews'])
    assert sum(glb['accessors'][m['primitives'][0]['indices']]['count']//3 for m in glb['meshes'])==triangles
    report={'status':'PASS','links':len(links),'wheel_joints':wheel_joints,'resolved_mesh_files':len(mesh_files),'displayed_triangles':triangles,'overall_size_m':(points.max(0)-points.min(0)).tolist(),'mass_kg_estimated':total_mass,'camera_optical_forward':'base +X','sensor_data_implemented':False,'gazebo_runtime_tested':False}
    if args.scope_baseline:
        baseline=json.loads(args.scope_baseline.read_text());changed=[p for p,h in baseline.items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
        assert not changed,changed;report['unchanged_map_and_archive_files']=len(baseline)
    if args.check_install:
        count=0
        for name,subdirs in [('dongfeng_description',['urdf','meshes','scripts']),('dongfeng_bringup',['launch'])]:
            source=ROOT/'src'/name;target=ROOT/'install'/name/'share'/name
            for sub in subdirs:
                for path in (source/sub).rglob('*'):
                    if path.is_file() and '__pycache__' not in path.parts:
                        assert path.read_bytes()==(target/path.relative_to(source)).read_bytes(),path;count+=1
        report['matching_installed_files']=count
    report_dir=ROOT/'reports/vehicle';report_dir.mkdir(parents=True,exist_ok=True)
    (report_dir/'validation_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
