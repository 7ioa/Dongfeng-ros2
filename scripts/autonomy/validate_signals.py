#!/usr/bin/env python3
"""Compare commanded lamps with sensor images and actual Ogre GUI materials."""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2,numpy as np,rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from gz.transport13 import Node as GzNode
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from dongfeng_autonomy.signal_node import Signals
from dongfeng_autonomy.vision import detect_light,signal_roi


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',default='reports/autonomy/signal_probe');parser.add_argument('--domain',type=int,default=96);parser.add_argument('--stall-gui',action='store_true');args=parser.parse_args()
    repo=Path(__file__).resolve().parents[2];root=Path(args.output).resolve();root.mkdir(parents=True,exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),GZ_PARTITION=f'dongfeng_signal_probe_{args.domain}',QT_QPA_PLATFORM='xcb')
    os.environ['GZ_GUI_PLUGIN_PATH']=str(repo/'build_control/gui_probe')+os.pathsep+os.environ.get('GZ_GUI_PLUGIN_PATH','')
    world=ET.parse(repo/'worlds/dongfeng.sdf')
    plugin=ET.SubElement(world.getroot().find('world/gui'),'plugin',filename='GuiSignalProbe',name='GUI signal validation')
    ET.SubElement(plugin,'capture').text=str(root)
    world.write(root/'probe.sdf')
    log=(root/'launch.log').open('w')
    process=subprocess.Popen(['bash',str(repo/'launch_car.sh'),'sensor_software_rendering:=true',f'world:={root}/probe.sdf','x:=3.11','y:=3.014','yaw:=1.57079632679'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true']);node=Signals();gz=GzNode();records=[];checks=[];camera=[None];gui=[None];k=[(254.,254.,320.,240.)]
    def now():return node.get_clock().now().nanoseconds/1e9
    def receive_gui(msg):sample=json.loads(msg.data);gui[0]=sample['bulbs'];records.append(sample)
    gz.subscribe(StringMsg,'/evaluation/gui_signals',receive_gui)
    def receive_image(m):
        im=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        camera[0]=im[:,:,::-1].copy() if m.encoding=='rgb8' else im.copy()
    node.create_subscription(Image,'/camera/image_raw',receive_image,qos_profile_sensor_data)
    node.create_subscription(CameraInfo,'/camera/camera_info',lambda m:k.__setitem__(0,(m.k[0],m.k[4],m.k[2],m.k[5])),qos_profile_sensor_data)
    def spin(seconds,predicate=None):
        start=time.monotonic()
        while time.monotonic()-start<seconds and process.poll() is None:
            rclpy.spin_once(node,timeout_sec=.02)
            if predicate and predicate():return True
        return False
    try:
        if not spin(40,lambda:camera[0] is not None and bool(gui[0]) and now()>.5 and bool(node.ids)):raise RuntimeError('GUI probe or camera not ready')
        for side,pose,sid in [('right',(3.11,3.014,math.pi/2),'signal_7'),('left',(.19,3.856,-math.pi/2),'signal_6')]:
            msg=Pose();msg.name='dongfeng_car';msg.position.x,msg.position.y=pose[:2];msg.position.z=.055;msg.orientation.z=math.sin(pose[2]/2);msg.orientation.w=math.cos(pose[2]/2)
            if side=='left':
                for attempt in range(4):
                    ok,response=gz.request('/world/dongfeng_world/set_pose',msg,Pose,Boolean,1500)
                    if ok and response.data:break
                    spin(.5)
                else:raise RuntimeError('pose service failed: '+str(ok)+' '+str(response))
            for color in ('red','green','yellow','red'):
                node.set_parameters([Parameter('force_color',value=color)])
                stalled=[]
                if args.stall_gui and color=='green':
                    rows=subprocess.check_output(['ps','-eo','pid,pgid,args'],text=True).splitlines()
                    stalled=[int(row.split(None,2)[0]) for row in rows[1:] if len(row.split(None,2))==3 and row.split(None,2)[1]==str(process.pid) and ('gz sim gui' in row or 'gz sim -g' in row)]
                    if not stalled:raise RuntimeError('GUI process not found for stall test')
                    for pid in stalled:os.kill(pid,signal.SIGSTOP)
                start=now()
                try:spin(12,lambda:now()-start>=1.5)
                finally:
                    for pid in stalled:os.kill(pid,signal.SIGCONT)
                spin(12,lambda:now()-start>=3.)
                expected={s['id']:color for s in node.config}
                observed={}
                for sig in node.config:
                    lit=[c for c in ('red','yellow','green') if max(gui[0].get(sig['id']+'_'+c,[0]))>.5]
                    observed[sig['id']]=lit[0] if len(lit)==1 else 'unknown'
                sig=next(s for s in node.config if s['id']==sid)
                seen=detect_light(camera[0],signal_roi(pose,sig,k[0]))[0]
                cv2.imwrite(str(root/f'{side}_{len(checks)}_{color}_camera.png'),camera[0])
                result=dict(side=side,stalled_pids=stalled,t=now(),expected=color,gui=observed,camera=seen,passed=observed==expected and seen==color)
                checks.append(result);print(json.dumps(result),flush=True)
    except Exception as error:checks.append(dict(passed=False,error=str(error)));print(str(error),flush=True)
    finally:
        try:os.killpg(process.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
        (root/'gui.json').write_text(json.dumps(records,indent=2));(root/'summary.json').write_text(json.dumps(dict(passed=bool(checks) and all(c['passed'] for c in checks),checks=checks),indent=2))
        node.destroy_node();rclpy.shutdown();log.close()

if __name__=='__main__':main()
