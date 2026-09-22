#!/usr/bin/env python3
"""Inject faults into real sensor relays; truth is recorded only by this tester."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
from google.protobuf import text_format

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from gz.transport13 import Node as GzNode
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.world_control_pb2 import WorldControl
from dongfeng_autonomy.route import Route


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',default='reports/autonomy/faults')
    parser.add_argument('--domain',type=int,default=92)
    parser.add_argument('--speed',type=float,default=.15)
    args=parser.parse_args()
    root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain),GZ_PARTITION=f'dongfeng_faults_{args.domain}',QT_QPA_PLATFORM='xcb')
    log=(root/'launch.log').open('w')
    process=subprocess.Popen(['bash',str(Path(__file__).resolve().parents[2]/'launch_car.sh'),'--autonomy','headless:=true',f'speed:={args.speed}','image_topic:=/fault_test/image','scan_topic:=/fault_test/scan'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('fault_evaluator');gz=GzNode()
    states=[];commands=[];truth=[];results=[];gate={'image':'live','scan':'live'};last={}
    image_pub=node.create_publisher(Image,'/fault_test/image',qos_profile_sensor_data)
    scan_pub=node.create_publisher(LaserScan,'/fault_test/scan',qos_profile_sensor_data)
    def forward(key,publisher,message):
        if gate[key]=='live':last[key]=message;publisher.publish(message)
        elif gate[key]=='stale' and key in last:publisher.publish(last[key])
    node.create_subscription(Image,'/camera/image_raw',lambda m:forward('image',image_pub,m),qos_profile_sensor_data)
    node.create_subscription(LaserScan,'/scan',lambda m:forward('scan',scan_pub,m),qos_profile_sensor_data)
    node.create_subscription(String,'/autonomy/status',lambda m:states.append(dict(json.loads(m.data),wall=time.monotonic())),10)
    node.create_subscription(Twist,'/cmd_vel_safe',lambda m:commands.append(dict(wall=time.monotonic(),v=m.linear.x,w=m.angular.z)),10)
    manual=node.create_publisher(Twist,'/cmd_vel_manual',1)
    enable=node.create_client(SetBool,'/autonomy/enable')
    def poses(msg):
        for p in msg.pose:
            if p.name=='dongfeng_car':
                q=p.orientation
                truth.append(dict(wall=time.monotonic(),t=msg.header.stamp.sec+msg.header.stamp.nsec/1e9,x=p.position.x,y=p.position.y,z=p.position.z,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
    gz.subscribe(Pose_V,'/world/dongfeng_world/dynamic_pose/info',poses)
    def spin(seconds,predicate=None):
        start=time.monotonic()
        while time.monotonic()-start<seconds and process.poll() is None:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate and predicate():return True
        return False
    def moving():return bool(commands and commands[-1]['v']>.01 and states and states[-1]['state'] in ('DRIVE','APPROACH_SIGNAL'))
    def call(service,request,kind):
        name='/world/dongfeng_world/'+service
        if not spin(5,lambda:bool(gz.service_info(name))):
            raise RuntimeError('Gazebo service not discovered: '+service)
        # Keep sensor relays spinning while a separate transport client waits.
        # A blocking request in this process can stall its subscriber callbacks.
        client=subprocess.Popen(['gz','service','-s',name,'--reqtype',kind.DESCRIPTOR.full_name,
            '--reptype',Boolean.DESCRIPTOR.full_name,'--timeout','5000','--req',text_format.MessageToString(request)],
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        if not spin(8,lambda:client.poll() is not None):
            client.kill();client.wait();raise RuntimeError('Gazebo CLI request timeout: '+service)
        output=client.communicate()[0];response=Boolean()
        try:text_format.Parse(output,response)
        except text_format.ParseError:raise RuntimeError('Gazebo CLI response invalid: '+service+' '+output)
        if client.returncode!=0 or not response.data:raise RuntimeError('Gazebo request rejected: '+service+' '+output)
    def stopped_since(t):return next((c for c in commands if c['wall']>=t and abs(c['v'])<1e-6 and abs(c['w'])<1e-6),None)
    def record(name,passed,**data):
        item=dict(name=name,passed=bool(passed),**data);results.append(item);print(json.dumps(item),flush=True)
    def sensor_fault(key,mode):
        if not spin(10,moving):raise RuntimeError('No movement before '+key+' '+mode)
        start=time.monotonic();origin=truth[-1];gate[key]=mode
        spin(1.3)
        stop=stopped_since(start);delay=stop['wall']-start if stop else None
        displacement=math.hypot(truth[-1]['x']-origin['x'],truth[-1]['y']-origin['y'])
        stable=all(abs(c['v'])<1e-6 for c in commands if c['wall']>start+.65)
        # A 50 ms watchdog period gives the 0.5 s timeout up to one tick to fire.
        record(key+'_'+mode,stop is not None and delay<=.56 and stable,stop_delay=delay,distance=displacement)
        gate[key]='live';record(key+'_'+mode+'_recovery',spin(10,moving))
    try:
        if not spin(35,moving):raise RuntimeError('Simulation did not become ready')
        for key in ('image','scan'):
            for mode in ('drop','stale'):sensor_fault(key,mode)
        start=time.monotonic();manual.publish(Twist());spin(1.)
        record('manual_stop_latches',stopped_since(start) is not None and not moving() and states[-1]['state']=='MANUAL')
        request=SetBool.Request();request.data=True;future=enable.call_async(request)
        spin(3,lambda:future.done());record('manual_reenable',future.done() and future.result().success and spin(10,moving))
        # Place a real collision and visible obstacle on the local forward path.
        p=truth[-1];x=p['x']+.26*math.cos(p['yaw']);y=p['y']+.26*math.sin(p['yaw'])
        obstacle=EntityFactory();obstacle.sdf=f'<sdf version="1.9"><model name="fault_obstacle"><static>true</static><pose>{x} {y} {p["z"]+.045} 0 0 {p["yaw"]}</pose><link name="box"><collision name="collision"><geometry><box><size>.04 .14 .15</size></box></geometry></collision><visual name="visual"><geometry><box><size>.04 .14 .15</size></box></geometry><material><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>'
        call('create',obstacle,EntityFactory)
        blocked=spin(12,lambda:bool(states and states[-1]['state']=='OBSTACLE_STOP'))
        spin(.7);p=truth[-1];gap=math.hypot(x-p['x'],y-p['y'])-.1032-.02
        record('forward_obstacle',blocked and gap>0 and commands[-1]['v']==0,front_gap=gap)
        entity=Entity();entity.name='fault_obstacle';entity.type=Entity.MODEL;call('remove',entity,Entity)
        record('obstacle_removed_recovery',spin(10,moving))
        # Wait for the actual B bend, then put a collision on its curved path.
        route=Route.perimeter(3.3,5.4,.04,.8,.3)
        if not spin(35,lambda:bool(truth and route.project(truth[-1]['x'],truth[-1]['y'])[0]>=1.15)):
            raise RuntimeError('Vehicle did not reach B bend')
        p=truth[-1];progress=route.project(p['x'],p['y'])[0]
        center=route.target(progress+.24);ahead=route.target(progress+.25)
        x,y=center;obstacle_yaw=math.atan2(ahead[1]-y,ahead[0]-x)
        obstacle=EntityFactory();obstacle.sdf=f'<sdf version="1.9"><model name="curve_obstacle"><static>true</static><pose>{x} {y} {p["z"]+.045} 0 0 {obstacle_yaw}</pose><link name="box"><collision name="collision"><geometry><box><size>.04 .14 .15</size></box></geometry></collision><visual name="visual"><geometry><box><size>.04 .14 .15</size></box></geometry><material><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>'
        call('create',obstacle,EntityFactory)
        blocked=spin(12,lambda:bool(states and states[-1]['state']=='OBSTACLE_STOP'))
        spin(.7);p=truth[-1]
        u=(math.cos(p['yaw']),math.sin(p['yaw']));v=(-u[1],u[0])
        a=(math.cos(obstacle_yaw),math.sin(obstacle_yaw));b=(-a[1],a[0])
        def dot(a,b):return a[0]*b[0]+a[1]*b[1]
        delta=(x-p['x'],y-p['y'])
        gap=max(abs(dot(delta,axis))-.1032*abs(dot(u,axis))-.0773*abs(dot(v,axis))-.02*abs(dot(a,axis))-.07*abs(dot(b,axis)) for axis in (u,v,a,b))
        record('curve_obstacle',blocked and gap>0 and commands[-1]['v']==0,separating_gap=gap,progress=progress)
        entity=Entity();entity.name='curve_obstacle';entity.type=Entity.MODEL;call('remove',entity,Entity)
        record('curve_obstacle_removed_recovery',spin(10,moving))
        pause=WorldControl();pause.pause=True;call('control',pause,WorldControl);start=time.monotonic();spin(1.2)
        stop=stopped_since(start)
        record('pause_stops_commands',stop is not None and stop['wall']-start<=.56,stop_delay=None if stop is None else stop['wall']-start)
        pause.pause=False;call('control',pause,WorldControl);record('pause_resume',spin(10,moving))
        reset=WorldControl();reset.reset.time_only=True;call('control',reset,WorldControl);spin(1.5)
        record('clock_rewind_latches',bool(states and states[-1]['state']=='FAULT_STOP' and 'clock reset' in states[-1]['reason']) and commands[-1]['v']==0)
        # Verify launch teardown on unexpected driver exit, targeting only our group.
        rows=subprocess.check_output(['ps','-eo','pid,pgid,args'],text=True).splitlines()
        targets=[int(row.split(None,2)[0]) for row in rows[1:] if len(row.split(None,2))==3 and row.split(None,2)[1]==str(process.pid) and '/lib/dongfeng_autonomy/autonomy_node ' in row]
        for pid in targets:os.kill(pid,signal.SIGTERM)
        spin(8);record('driver_exit_shuts_launch',bool(targets) and process.poll() is not None)
    except Exception as error:
        record('runner',False,error=str(error))
    finally:
        try:os.killpg(process.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            process.wait()
        for name,data in [('summary',dict(passed=bool(results) and all(r['passed'] for r in results),checks=results)),('status',states),('commands',commands),('truth',truth)]:
            (root/(name+'.json')).write_text(json.dumps(data,indent=2))
        node.destroy_node();rclpy.shutdown();log.close()

if __name__=='__main__':main()
