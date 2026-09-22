#!/usr/bin/env python3
"""Short repeatable manual dynamics experiment; results are not autonomous laps."""
import argparse,json,math,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu,JointState,LaserScan,Image
from nav_msgs.msg import Odometry
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.wheel_slip_parameters_cmd_pb2 import WheelSlipParametersCmd
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--lateral',type=float,default=0);p.add_argument('--longitudinal',type=float,default=0)
    p.add_argument('--domain',type=int,default=85);p.add_argument('--output',required=True)
    p.add_argument('--x',type=float,default=2.48);p.add_argument('--y',type=float,default=.195)
    p.add_argument('--z',type=float,default=.055);p.add_argument('--yaw',type=float,default=.08)
    p.add_argument('--linear',type=float,default=.06);p.add_argument('--angular',type=float,default=.15)
    p.add_argument('--seconds',type=float,default=6)
    a=p.parse_args();root=Path(a.output);root.mkdir(parents=True,exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID=str(a.domain),GZ_PARTITION=f'dongfeng_probe_{a.domain}',QT_QPA_PLATFORM='xcb')
    log=(root/'launch.log').open('w')
    proc=subprocess.Popen(['bash',str(Path(__file__).resolve().parents[2]/'launch_car.sh'),'headless:=true','sensor_software_rendering:=true',f'x:={a.x}',f'y:={a.y}',f'z:={a.z}',f'yaw:={a.yaw}'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('dynamics_probe');gz=GzNode();truth=[];imu=[];odom=[];joints=[];scans=[];sensors={'scan':0,'image':0}
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec/1e9
    def poses(m):
        t=m.header.stamp.sec+m.header.stamp.nsec/1e9
        if truth and t-truth[-1]['t']<.02:return
        for pose in m.pose:
            if pose.name=='dongfeng_car':
                q=pose.orientation
                truth.append(dict(t=t,x=pose.position.x,y=pose.position.y,z=pose.position.z,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
    gz.subscribe(Pose_V,'/world/dongfeng_world/dynamic_pose/info',poses)
    node.create_subscription(Imu,'/imu/data',lambda m:imu.append([stamp(m),m.angular_velocity.z,m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w]),qos_profile_sensor_data)
    node.create_subscription(Odometry,'/odom',lambda m:odom.append([stamp(m),m.twist.twist.linear.x,m.twist.twist.angular.z]),qos_profile_sensor_data)
    node.create_subscription(JointState,'/joint_states',lambda m:joints.append([stamp(m),list(m.name),list(m.velocity)]),qos_profile_sensor_data)
    def scan(m):
        sensors.update(scan=sensors['scan']+1,finite_ranges=int(np.isfinite(m.ranges).sum()))
        scans.append(dict(t=stamp(m),angle_min=m.angle_min,angle_increment=m.angle_increment,ranges=list(m.ranges)))
    def image(m):sensors.update(image=sensors['image']+1,image_std=float(np.frombuffer(m.data,np.uint8).std()))
    node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data);node.create_subscription(Image,'/camera/image_raw',image,qos_profile_sensor_data)
    pub=node.create_publisher(Twist,'/cmd_vel_manual',1);start=time.monotonic();begin=None;configured=False
    try:
        while proc.poll() is None and time.monotonic()-start<45:
            rclpy.spin_once(node,timeout_sec=.01)
            if not truth or not odom:continue
            if not configured:
                cmd=WheelSlipParametersCmd();cmd.entity.name='dongfeng_car';cmd.entity.type=Entity.MODEL
                cmd.slip_compliance_lateral=a.lateral;cmd.slip_compliance_longitudinal=a.longitudinal
                ok,response=gz.request('/world/dongfeng_world/wheel_slip',cmd,WheelSlipParametersCmd,Boolean,500)
                if not (ok and response.data):continue
                configured=True;begin=truth[-1]['t']+1.
            elapsed=truth[-1]['t']-begin
            msg=Twist()
            if 0<=elapsed<a.seconds:msg.linear.x=a.linear;msg.angular.z=a.angular
            pub.publish(msg)
            if elapsed>=a.seconds+1.:break
    finally:
        pub.publish(Twist())
        try:os.killpg(proc.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:os.killpg(proc.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            proc.wait()
        active=[t for t in truth if begin is not None and begin+1<=t['t']<=begin+a.seconds]
        result=dict(parameters=vars(a),configured=configured,sensors=sensors,begin=begin)
        if len(active)>1:
            first,last=active[0],active[-1];dt=last['t']-first['t']
            result.update(distance=math.hypot(last['x']-first['x'],last['y']-first['y']),yaw_change=last['yaw']-first['yaw'],actual_speed=math.hypot(last['x']-first['x'],last['y']-first['y'])/dt,yaw_rate=(last['yaw']-first['yaw'])/dt,last_truth=last)
        (root/'data.json').write_text(json.dumps(dict(truth=truth,imu=imu,odom=odom,joints=joints,scans=scans)))
        (root/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
        node.destroy_node();rclpy.shutdown();log.close()

if __name__=='__main__':main()
