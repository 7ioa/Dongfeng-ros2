#!/usr/bin/env python3
"""Independent high-speed fault probes. Simulator truth is evaluator-only."""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from std_msgs.msg import String
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V


def run_probe(repo,root,domain,kind):
    root.mkdir(parents=True,exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(domain),GZ_PARTITION=f'dongfeng_braking_{domain}',QT_QPA_PLATFORM='xcb')
    log=(root/'launch.log').open('w')
    proc=subprocess.Popen(['bash',str(repo/'launch_car.sh'),'headless:=true',
        'driving_mode:=fast','auto_mode:=true','sensor_software_rendering:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    driver_log=(root/'driver.log').open('w')
    driver_env=os.environ.copy()
    driver_env['AMENT_PREFIX_PATH']=str(repo/'install_control/dongfeng_autonomy')+os.pathsep+driver_env.get('AMENT_PREFIX_PATH','')
    driver=subprocess.Popen(['python3',str(repo/'scripts/autonomy/speed_fault_driver.py'),'--ros-args',
        '-p','use_sim_time:=true','-p','driving_mode:=fast','-p','mission:=full_demo',
        '-p',f'map_output:={root}/parking_map'],env=driver_env,stdout=driver_log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('independent_speed_braking');gz=GzNode()
    states=[];commands=[];truth=[]
    parameters=node.create_client(SetParameters,'/autonomy/set_parameters')
    node.create_subscription(String,'/autonomy/status',lambda m:states.append(dict(json.loads(m.data),wall=time.monotonic())),10)
    node.create_subscription(Twist,'/cmd_vel_safe',lambda m:commands.append(dict(wall=time.monotonic(),v=m.linear.x,w=m.angular.z)),10)
    def poses(message):
        for p in message.pose:
            if p.name=='dongfeng_car':
                truth.append(dict(wall=time.monotonic(),t=message.header.stamp.sec+message.header.stamp.nsec/1e9,
                                  x=p.position.x,y=p.position.y,z=p.position.z))
    gz.subscribe(Pose_V,'/world/dongfeng_world/dynamic_pose/info',poses)
    def spin(seconds,predicate=None):
        start=time.monotonic()
        while time.monotonic()-start<seconds and proc.poll() is None and driver.poll() is None:
            rclpy.spin_once(node,timeout_sec=.01)
            if predicate and predicate():return True
        return False
    result=dict(probe=kind,passed=False)
    try:
        ready=spin(70,lambda:bool(states and truth and states[-1]['state']=='DRIVE'
                    and states[-1]['measured_speed']>=.38 and states[-1]['segment_index']==2
                    and abs(states[-1]['tracking_curvature'])<.25))
        if not ready:raise RuntimeError('Did not reach measured speed >= 0.38 m/s on the long right straight')
        origin=copy.deepcopy(truth[-1]);initial=states[-1]['measured_speed'];start=time.monotonic()
        result.update(initial_measured_speed=initial,initial_truth=origin)
        if kind=='obstacle':
            yaw=states[-1]['pose'][2]
            x=origin['x']+.62*math.cos(yaw);y=origin['y']+.62*math.sin(yaw)
            sdf=f'<sdf version="1.9"><model name="speed_obstacle"><static>true</static><pose>{x} {y} {origin["z"]+.045} 0 0 {yaw}</pose><link name="box"><collision name="collision"><geometry><box><size>.04 .14 .15</size></box></geometry></collision><visual name="visual"><geometry><box><size>.04 .14 .15</size></box></geometry><material><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>'
            request='sdf: '+json.dumps(sdf)
            client=subprocess.Popen(['gz','service','-s','/world/dongfeng_world/create','--reqtype','gz.msgs.EntityFactory',
                '--reptype','gz.msgs.Boolean','--timeout','5000','--req',request],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            if not spin(8,lambda:client.poll() is not None):client.kill();client.wait();raise RuntimeError('Obstacle creation timed out')
            response=client.communicate()[0]
            if client.returncode!=0 or 'data: true' not in response:raise RuntimeError('Obstacle creation failed: '+response)
            blocked=spin(8,lambda:bool(states and states[-1]['state']=='OBSTACLE_STOP'))
            spin(1.2);gap=(x-truth[-1]['x'])*math.cos(yaw)+(y-truth[-1]['y'])*math.sin(yaw)-.1032-.02
            stopped=[c for c in commands if c['wall']>time.monotonic()-.5]
            result.update(front_gap=gap,stop_state=states[-1]['state'],passed=blocked and gap>.035
                and bool(stopped) and all(abs(c['v'])<1e-6 for c in stopped))
            client=subprocess.Popen(['gz','service','-s','/world/dongfeng_world/remove','--reqtype','gz.msgs.Entity',
                '--reptype','gz.msgs.Boolean','--timeout','5000','--req','name: "speed_obstacle", type: MODEL'],
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            if not spin(8,lambda:client.poll() is not None):client.kill();client.wait();raise RuntimeError('Obstacle removal timed out')
            response=client.communicate()[0]
            if client.returncode!=0 or 'data: true' not in response:raise RuntimeError('Obstacle removal failed: '+response)
            result['recovered']=spin(10,lambda:bool(commands and commands[-1]['v']>.01))
            result['passed'] &= result['recovered']
        else:
            request=SetParameters.Request()
            request.parameters=[Parameter('probe_image_gate',value='drop' if kind=='image_drop' else 'stale').to_parameter_msg()]
            future=parameters.call_async(request)
            if not spin(3,lambda:future.done()) or not all(r.successful for r in future.result().results):
                raise RuntimeError('Image fault injection rejected')
            spin(1.3)
            stop=next((c for c in commands if c['wall']>=start and c['v']==0. and c['w']==0.),None)
            delay=stop['wall']-start if stop else None
            distance=math.hypot(truth[-1]['x']-origin['x'],truth[-1]['y']-origin['y'])
            stable=all(c['v']==0. for c in commands if c['wall']>start+.65)
            result.update(stop_delay=delay,distance=distance,deadline=.62,
                passed=stop is not None and delay<=.62 and stable and distance<.40)
            request.parameters=[Parameter('probe_image_gate',value='live').to_parameter_msg()]
            future=parameters.call_async(request);spin(3,lambda:future.done())
            result['recovered']=spin(10,lambda:bool(commands and commands[-1]['v']>.01))
            result['passed'] &= result['recovered']
        # Truth speed over a 0.2 s window independently confirms the wheel reading.
        before=[p for p in truth if p['t']<=origin['t']]
        old=next((p for p in reversed(before) if origin['t']-p['t']>=.2),None)
        actual=(math.hypot(origin['x']-old['x'],origin['y']-old['y'])/(origin['t']-old['t'])) if old else 0.
        result['initial_actual_speed']=actual
        result['passed'] &= actual>=.35
    except Exception as error:result['error']=str(error)
    finally:
        try:os.killpg(driver.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:driver.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(driver.pid,signal.SIGKILL);driver.wait()
        driver_log.close()
        try:os.killpg(proc.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:os.killpg(proc.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            proc.wait()
        for name,data in (('summary',result),('status',states),('truth',truth),('commands',commands)):
            (root/(name+'.json')).write_text(json.dumps(data,indent=2))
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        log.close()
    print(json.dumps(result),flush=True)
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--domain',type=int,default=111)
    parser.add_argument('--output',default='reports/autonomy/speed_braking')
    args=parser.parse_args();repo=Path(__file__).resolve().parents[2];root=Path(args.output)
    results=[run_probe(repo,root/kind,args.domain+i,kind) for i,kind in enumerate(('obstacle','image_drop','image_stale'))]
    result=dict(passed=all(r['passed'] for r in results),checks=results)
    (root/'summary.json').write_text(json.dumps(result,indent=2));return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
