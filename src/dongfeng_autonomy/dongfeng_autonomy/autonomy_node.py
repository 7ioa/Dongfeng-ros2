"""Sensor-based controller. No subscriptions to simulator truth or lamp phase."""
import json
import math
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan, Imu, CameraInfo
from std_msgs.msg import String, Bool
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from .route import Route, wrap
from .control import Driver, Observation, obstacle_distance, YawController
from .vision import detect_lane, detect_light, signal_roi, detect_stop_line
from .localization import LandmarkMap, rotation
from .sensing import Freshness, orientation_rpy


def rpy(q):
    roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
    pitch=math.asin(np.clip(2*(q.w*q.y-q.z*q.x),-1,1))
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    return roll,pitch,yaw


class Autonomy(Node):
    def __init__(self):
        super().__init__('autonomy')
        root=Path(get_package_share_directory('dongfeng_autonomy'))/'config'
        cfg=json.loads((root/'scene.json').read_text());self.cfg=cfg
        self.route=Route.perimeter(cfg['interior_width'],cfg['interior_length'],cfg['road_edge_inset_assumed'],cfg['outer_road_radius'],cfg['one_way_width'])
        signals=json.loads((root/'signals.json').read_text())
        self.signals=[]
        for sig in signals:
            if sig['id'] in ('signal_6','signal_7'):
                x= .19 if sig['id']=='signal_6' else 3.11
                y=3.68 if sig['id']=='signal_6' else 3.19
                sig['line_s']=self.route.project(x,y)[0]
                sig['stop_s']=sig['line_s']-.1032-.035
                self.signals.append(sig)
        self.driver=Driver(self.route,self.signals,float(self.declare_parameter('speed',.15).value))
        self.pose=np.array([1.65,.19,0.]);self.pitch=0.;self.roll=0.
        self.yaw_rate=0.;self.yaw_controller=YawController();self.control_time=None
        self.prev_odom=None;self.imu_ready=False
        self.observation=Observation(fresh=False,lane_valid=False)
        self.freshness=Freshness(('image','scan','imu','odom','info','localization'),.4)
        self.received=self.freshness.received;self.source_stamp=self.freshness.stamps;self.frame=0
        data=np.load(root/'landmarks.npz')
        self.landmarks=LandmarkMap(data['points'],data['normals']);self.road=data['road']
        self.ground_height=0.;self.localization_good=False;self.path_curvature=0.
        self.ranges=None;self.angles=None;self.last_command=(0.,0.)
        self.camera_k=(254.,254.,320.,240.)
        self.debug=bool(self.declare_parameter('debug_images',True).value)
        self.pub=self.create_publisher(Twist,'/cmd_vel_auto',1)
        self.status_pub=self.create_publisher(String,'/autonomy/status',5)
        self.image_pub=self.create_publisher(Image,'/autonomy/debug_image',qos_profile_sensor_data)
        for typ,topic,cb in [(Image,'/camera/image_raw',self.image),(CameraInfo,'/camera/camera_info',self.info),(LaserScan,'/scan',self.scan),(Imu,'/imu/data',self.imu),(Odometry,'/odom',self.odom)]:
            self.create_subscription(typ,topic,cb,qos_profile_sensor_data)
        self.timer=self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.last_report=0.;self.last_progress=0.;self.progress_time=time.monotonic()
        self.enabled=False
        self.create_subscription(Bool,'/autonomy/enabled',self.mode,1)
        self.tf=TransformBroadcaster(self)

    def mode(self,m):
        if not m.data:
            self.yaw_controller.integral=0.;self.driver.green_frames=0
        self.enabled=m.data

    def mark(self,key,m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        return self.freshness.update(key,stamp,time.monotonic())

    def info(self,m):
        self.camera_k=(m.k[0],m.k[4],m.k[2],m.k[5]);self.mark('info',m)

    def odom(self,m):
        q=m.pose.pose.orientation
        orientation=orientation_rpy((q.x,q.y,q.z,q.w))
        p=m.pose.pose.position
        if orientation is None or not all(math.isfinite(v) for v in (p.x,p.y)):return
        if not self.mark('odom',m):return
        oyaw=orientation[2]
        current=np.array([p.x,p.y,oyaw])
        if self.prev_odom is not None and self.imu_ready:
            delta=current[:2]-self.prev_odom[:2]
            forward=float(np.dot(delta,[math.cos(oyaw),math.sin(oyaw)]))
            if abs(forward)<.05:
                self.pose[:2]+=forward*np.array([math.cos(self.pose[2]),math.sin(self.pose[2])])
        self.prev_odom=current

    def imu(self,m):
        q=m.orientation
        orientation=orientation_rpy((q.x,q.y,q.z,q.w),available=m.orientation_covariance[0]!=-1)
        if orientation is None or not math.isfinite(m.angular_velocity.z):return
        if not self.mark('imu',m):return
        self.roll,self.pitch,self.pose[2]=orientation
        self.yaw_rate=m.angular_velocity.z
        self.imu_ready=True

    def scan(self,m):
        if not self.mark('scan',m):return
        self.ranges=np.array(m.ranges,dtype=float)
        self.angles=m.angle_min+np.arange(len(m.ranges))*m.angle_increment
        # Match actual range endpoints against known static scene surfaces.
        if not self.imu_ready:return
        nearest=int(np.argmin(np.linalg.norm(self.road[:,:2]-self.pose[:2],axis=1)))
        self.ground_height=float(self.road[nearest,2])
        good=np.isfinite(self.ranges)&(self.ranges>=.04)&(self.ranges<4.8)
        indices=np.flatnonzero(good)[::2]
        ranges=self.ranges[indices];angles=self.angles[indices]
        local=np.c_[ranges*np.cos(angles)+.075,ranges*np.sin(angles),np.full(len(indices),.065)]
        world=local@rotation(self.roll,self.pitch,self.pose[2]).T
        world+=np.array([*self.pose[:2],self.ground_height+.028])
        delta,valid=self.landmarks.correction(world)
        self.localization_good=bool(valid and np.linalg.norm(delta)<.15)
        if self.localization_good:
            self.pose[:2]+=delta*.7
            self.mark('localization',m)

    def image(self,m):
        if m.encoding not in ('rgb8','bgr8') or m.step<m.width*3 or len(m.data)!=m.height*m.step:return
        if not self.mark('image',m):return
        frame=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        if m.encoding=='rgb8':frame=cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
        self.frame+=1
        error,valid=detect_lane(frame)
        self.observation.lane_error=error;self.observation.lane_valid=valid
        self.observation.light='unknown';self.observation.light_id='';self.observation.frame=self.frame
        self.observation.stop_distance=None
        index=self.driver.signal_index
        roi=None
        if index<len(self.driver.signals):
            sig=self.driver.signals[index]
            distance=sig['stop_s']-self.driver.progress+.035
            self.observation.stop_distance=detect_stop_line(frame,self.camera_k,self.pitch,distance)
            roi=signal_roi(self.pose,sig,self.camera_k,self.pitch,height=self.ground_height+.036)
            if roi:
                color,confidence=detect_light(frame,roi)
                if confidence>.1:self.observation.light=color;self.observation.light_id=sig['id']
        if self.debug and self.frame%3==0:
            debug=frame.copy()
            if roi:
                x,y,w,h=roi;cv2.rectangle(debug,(x,y),(x+w,y+h),(0,255,255),1)
            cv2.putText(debug,f'{self.driver.state} lane={valid} light={self.observation.light}',(10,20),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,180,255),1)
            out=Image();out.header=m.header;out.height=m.height;out.width=m.width;out.encoding='bgr8';out.step=m.width*3;out.data=debug.tobytes();self.image_pub.publish(out)

    def tick(self):
        wall=time.monotonic();now=self.get_clock().now().nanoseconds/1e9
        ages={k:wall-self.received.get(k,-math.inf) for k in self.freshness.required}
        source_ages={k:now-self.source_stamp.get(k,-math.inf) for k in ages}
        self.observation.fresh=self.freshness.ready(now,wall)
        self.observation.pitch=self.pitch
        if self.ranges is not None and self.observation.fresh and self.enabled:
            v,w=self.last_command
            self.observation.clearance=obstacle_distance(self.ranges,self.angles,self.path_curvature)
        command=self.driver.step(self.pose,self.observation,now,enabled=self.enabled)
        self.path_curvature=command[1]/max(.03,command[0])
        dt=0. if self.control_time is None else now-self.control_time
        self.control_time=now
        command=(command[0],self.yaw_controller.step(command[1],self.yaw_rate,dt,stopped=command[0]==0.))
        if self.driver.progress-self.last_progress>.01:
            self.last_progress=self.driver.progress;self.progress_time=wall
        if self.driver.state in ('DRIVE','CROSSING') and wall-self.progress_time>15:
            command=self.driver.stop('FAULT_STOP','no progress for 15 seconds')
        elif self.driver.state not in ('DRIVE','CROSSING'):self.progress_time=wall
        if self.prev_odom is not None:
            yaw=wrap(self.pose[2]-self.prev_odom[2]);c,s=math.cos(yaw),math.sin(yaw)
            xy=self.pose[:2]-np.array([[c,-s],[s,c]])@self.prev_odom[:2]
            tf=TransformStamped();tf.header.stamp=self.get_clock().now().to_msg();tf.header.frame_id='map';tf.child_frame_id='odom'
            tf.transform.translation.x=float(xy[0]);tf.transform.translation.y=float(xy[1])
            tf.transform.rotation.z=math.sin(yaw/2);tf.transform.rotation.w=math.cos(yaw/2);self.tf.sendTransform(tf)
        self.last_command=command
        if not self.enabled:
            command=(0.,0.);self.yaw_controller.integral=0.
            self.progress_time=wall
        msg=Twist();msg.linear.x,msg.angular.z=command;self.pub.publish(msg)
        if wall-self.last_report>.25:
            self.last_report=wall
            data=dict(state=self.driver.state,reason=self.driver.reason,progress=round(self.driver.progress,3),length=round(self.route.length,3),pose=self.pose.tolist(),pitch=self.pitch,localized=self.localization_good,lane_valid=self.observation.lane_valid,lane_error=self.observation.lane_error,light=self.observation.light,signal=self.observation.light_id,clearance=self.observation.clearance if math.isfinite(self.observation.clearance) else None,ages={k:v if math.isfinite(v) else None for k,v in ages.items()},command=command,sim_time=now,frame=self.frame,stop_distance=self.observation.stop_distance)
            self.status_pub.publish(String(data=json.dumps(data,allow_nan=False)))


def main():
    rclpy.init();n=Autonomy()
    try:rclpy.spin(n)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():n.pub.publish(Twist())
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
