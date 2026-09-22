#!/usr/bin/env python3
"""Launch and independently measure a lap. Truth poses never reach the driver."""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2,numpy as np,rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from dongfeng_autonomy.route import Route
from dongfeng_autonomy.evaluation import evaluate_lap, evaluate_signals, gui_signal_sample


def main():
    p=argparse.ArgumentParser();p.add_argument('--seconds',type=float,default=420);p.add_argument('--phase',type=float,default=0);p.add_argument('--domain',type=int,default=74);p.add_argument('--output',default='reports/autonomy/run');p.add_argument('--gui',action='store_true');p.add_argument('--controlled-signals',action='store_true');a=p.parse_args()
    root=Path(a.output).resolve();root.mkdir(parents=True,exist_ok=True)
    repo=Path(__file__).resolve().parents[2]
    os.environ['ROS_DOMAIN_ID']=str(a.domain);os.environ['GZ_PARTITION']='dongfeng_validation_'+str(a.domain);os.environ['QT_QPA_PLATFORM']='xcb'
    log=(root/'launch.log').open('w')
    command=['bash',str(repo/'launch_car.sh'),'--autonomy',f'headless:={str(not a.gui).lower()}',f'phase_offset:={a.phase}']
    if a.controlled_signals:command.append('force_color:=red')
    if a.gui:
        subprocess.run(['bash',str(repo/'scripts/autonomy/build_gui_probe.sh')],check=True,stdout=log,stderr=subprocess.STDOUT)
        os.environ['GZ_GUI_PLUGIN_PATH']=str(repo/'build_control/gui_probe')+os.pathsep+os.environ.get('GZ_GUI_PLUGIN_PATH','')
        gui=ET.parse(repo/'worlds/dongfeng.sdf').getroot().find('world/gui')
        probe=ET.SubElement(gui,'plugin',filename='GuiSignalProbe',name='GUI signal validation')
        if a.controlled_signals:ET.SubElement(probe,'capture').text=str(root)
        (root/'gui.config').write_text(''.join(ET.tostring(child,encoding='unicode') for child in gui))
        command.append(f'gui_config:={root}/gui.config')
    process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();n=rclpy.create_node('independent_lap_evaluator');gz=GzNode();truth=[];states=[];lamps=[];gui_lamps=[];last_print=[0.];last_image=[0.];complete=[None];complete_sim=[None];latest=[None]
    route=Route.perimeter(3.3,5.4,.04,.8,.3)
    signal_parameters=n.create_client(SetParameters,'/traffic_signals/set_parameters')
    holds={};released=set();stimuli=[];requests=[];right_reset=[False]
    def force_color(color,sid,stamp):
        request=SetParameters.Request();request.parameters=[Parameter('force_color',value=color).to_parameter_msg()]
        requests.append(signal_parameters.call_async(request));stimuli.append(dict(t=stamp,color=color,signal=sid))
    def poses(msg):
        for pose in msg.pose:
            if pose.name=='dongfeng_car':
                stamp=msg.header.stamp.sec+msg.header.stamp.nsec/1e9
                if truth and 0<=stamp-truth[-1]['t']<.05:continue
                q=pose.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                x,y,z=pose.position.x,pose.position.y,pose.position.z
                s,e=route.project(x,y)
                truth.append(dict(t=msg.header.stamp.sec+msg.header.stamp.nsec/1e9,x=x,y=y,z=z,yaw=yaw,progress=s,error=e))
    gz.subscribe(Pose_V,'/world/dongfeng_world/dynamic_pose/info',poses)
    def gui_signals(msg):
        sample=gui_signal_sample(json.loads(msg.data),time.monotonic())
        if sample is not None:gui_lamps.append(sample)
    if a.gui:gz.subscribe(StringMsg,'/evaluation/gui_signals',gui_signals)
    def status(m):
        d=json.loads(m.data);d['wall_time']=time.time();states.append(d);latest[0]=d
        if a.controlled_signals:
            sid=d['signal'];stamp=d['sim_time']
            if d['state']=='WAIT_SIGNAL' and d['light']=='red' and sid not in released:
                holds.setdefault(sid,stamp)
                if stamp-holds[sid]>=2 and signal_parameters.service_is_ready():
                    force_color('green',sid,stamp);released.add(sid)
            if 'signal_7' in released and not right_reset[0] and d['state']=='DRIVE' and d['progress']>4.6:
                force_color('red','signal_6',stamp);right_reset[0]=True
        if time.monotonic()-last_print[0]>4:
            print(json.dumps(dict(d,truth=truth[-1] if truth else None)),flush=True);last_print[0]=time.monotonic()
        if d['state']=='COMPLETE' and complete[0] is None:
            complete[0]=time.monotonic();complete_sim[0]=d['sim_time']
    def image(m):
        if time.monotonic()-last_image[0]<2:return
        last_image[0]=time.monotonic()
        im=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        if m.encoding=='rgb8':im=im[:,:,::-1]
        cv2.imwrite(str(root/'camera.png'),im)
        if latest[0] and latest[0]['state'] in ('APPROACH_SIGNAL','WAIT_SIGNAL','CROSSING','COMPLETE'):
            cv2.imwrite(str(root/(latest[0]['state']+'.png')),im)
            cv2.imwrite(str(root/f"{latest[0]['sim_time']:.2f}_{latest[0]['state']}.png"),im)
    n.create_subscription(String,'/autonomy/status',status,10)
    n.create_subscription(String,'/evaluation/signal_state',lambda m:lamps.append(json.loads(m.data)),10)
    n.create_subscription(Image,'/autonomy/debug_image',image,qos_profile_sensor_data)
    start=time.monotonic()
    try:
        while time.monotonic()-start<a.seconds and process.poll() is None:
            rclpy.spin_once(n,timeout_sec=.1)
            if a.gui and time.monotonic()-start>45 and not gui_lamps:
                print('GUI probe has no data; validation aborted',flush=True);break
            if complete[0] is not None and truth and truth[-1]['t']-complete_sim[0]>=2.5:break
    finally:
        def terminate_group(sig):
            try:os.killpg(process.pid,sig)
            except ProcessLookupError:pass
        terminate_group(signal.SIGINT)
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            terminate_group(signal.SIGTERM)
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:terminate_group(signal.SIGKILL);process.wait()
        (root/'status.json').write_text(json.dumps(states,indent=2));(root/'truth.json').write_text(json.dumps(truth,indent=2))
        result={'complete':bool(complete[0]),'wall_seconds':time.monotonic()-start,'status_samples':len(states),'truth_samples':len(truth),'max_center_error':max([abs(t['error']) for t in truth],default=None),'last_status':states[-1] if states else None,'last_truth':truth[-1] if truth else None,'wait_signal_samples':sum(s['state']=='WAIT_SIGNAL' for s in states)}
        result.update(evaluate_lap(truth,bool(complete[0]),route))
        result['motion_lap_pass']=result['lap_pass']
        signals=[{'id':'signal_7','line_s':route.project(3.11,3.19)[0]}, {'id':'signal_6','line_s':route.project(.19,3.68)[0]}]
        result.update(evaluate_signals(truth,lamps,signals,route))
        result['gui_traffic_pass']=None
        if a.gui:
            gui_result=evaluate_signals(truth,gui_lamps,signals,route)
            result['gui_traffic_pass']=gui_result['traffic_pass']
            result['gui_signal_crossings']=gui_result['signal_crossings']
            result['gui_red_stops']=gui_result['red_stops']
        result['lap_pass']=result['motion_lap_pass'] and result['traffic_pass'] and (not a.gui or result['gui_traffic_pass'])
        (root/'gui_signals.json').write_text(json.dumps(gui_lamps,indent=2))
        if a.controlled_signals:
            responses_ok=all(f.done() and all(r.successful for r in f.result().results) for f in requests)
            result['controlled_signals_pass']=released=={'signal_6','signal_7'} and responses_ok and len(result['red_stops'])==2
            result['signal_stimuli']=stimuli
            result['lap_pass']=result['lap_pass'] and result['controlled_signals_pass']
        (root/'signal_stimuli.json').write_text(json.dumps(stimuli,indent=2))
        (root/'signals.json').write_text(json.dumps(lamps,indent=2))
        (root/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
        n.destroy_node();rclpy.shutdown();log.close()

if __name__=='__main__':main()
