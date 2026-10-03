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
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, LaserScan, Imu, CameraInfo
from std_msgs.msg import String, Bool
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from .route import Route, wrap
from .mission import Mission
from .speed import SpeedProfile
from .mapping import ParkingMapper, outside_robot
from .control import Driver, Observation, obstacle_distance, YawController
from .vision import detect_lane, detect_light, signal_roi, detect_stop_line
from .localization import LandmarkMap, rotation, consistent_correction
from .sensing import Freshness, orientation_rpy, tracking_age


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
        mode=str(self.declare_parameter('mission','full_demo').value)
        if mode not in ('perimeter','full_demo'):raise ValueError('mission must be perimeter or full_demo')
        self.signals=[]
        for sig in signals:
            if sig['id'] in ('signal_6','signal_7'):
                x= .19 if sig['id']=='signal_6' else 3.11
                y=3.68 if sig['id']=='signal_6' else 3.19
                sig['line_s']=self.route.project(x,y)[0]
                sig['stop_s']=sig['line_s']-.1032-.035
                self.signals.append(sig)
        if mode=='full_demo':
            self.route=Mission.load(root/'full_demo.json',signals)
            self.signals=self.route.signals
        self.driving_mode=str(self.declare_parameter('driving_mode','fast').value)
        if self.driving_mode!='fast':raise ValueError('Only the optimized fast driving profile is supported')
        requested=float(self.declare_parameter('speed',0.).value)
        profile=SpeedProfile.load(root/'speed_fast.json',max_speed=requested if requested!=0. else None)
        speed=profile.max_speed
        self.driver=Driver(self.route,self.signals,speed,profile=profile)
        self.pose=(self.route.initial_pose if isinstance(self.route,Mission)
                   else np.r_[self.route.target(0),self.route.heading(0)])
        self.pitch=0.;self.roll=0.
        self.yaw_rate=0.;self.yaw_controller=YawController();self.control_time=None
        self.prev_odom=None;self.imu_ready=False
        self.sensor_faults=set()
        self.observation=Observation(fresh=False,lane_valid=False)
        self.freshness=Freshness(('image','scan','imu','odom','info','localization'),.5,{'localization':1.5,'info':2.})
        self.received=self.freshness.received;self.source_stamp=self.freshness.stamps;self.frame=0
        data=np.load(root/'landmarks.npz')
        self.landmarks=LandmarkMap(data['points'],data['normals']);self.road=data['road']
        self.ground_height=0.;self.localization_good=False;self.path_curvature=0.
        self.localization_delta=[0.,0.];self.localization_rejected=0;self.measured_speed=0.
        self.mapper=ParkingMapper();self.map_pub=self.create_publisher(OccupancyGrid,'/autonomy/parking_map',1)
        self.map_output=str(self.declare_parameter('map_output','reports/autonomy/parking_map').value)
        self.last_map=0.;self.map_saved=False
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
        if not all(math.isfinite(v) for v in m.k) or m.k[0]<=0 or m.k[4]<=0:
            self.sensor_faults.add('info');return
        self.sensor_faults.discard('info')
        self.camera_k=(m.k[0],m.k[4],m.k[2],m.k[5]);self.mark('info',m)

    def odom(self,m):
        q=m.pose.pose.orientation
        orientation=orientation_rpy((q.x,q.y,q.z,q.w))
        p=m.pose.pose.position
        if orientation is None or not all(math.isfinite(v) for v in (p.x,p.y,m.twist.twist.linear.x)):
            self.sensor_faults.add('odom');return
        self.sensor_faults.discard('odom')
        if not self.mark('odom',m):return
        oyaw=orientation[2]
        current=np.array([p.x,p.y,oyaw])
        if self.prev_odom is not None and self.imu_ready:
            delta=current[:2]-self.prev_odom[:2]
            forward=float(np.dot(delta,[math.cos(oyaw),math.sin(oyaw)]))
            if abs(forward)<.05:
                self.pose[:2]+=forward*np.array([math.cos(self.pose[2]),math.sin(self.pose[2])])
        self.prev_odom=current
        self.measured_speed=float(m.twist.twist.linear.x)

    def imu(self,m):
        q=m.orientation
        orientation=orientation_rpy((q.x,q.y,q.z,q.w),available=m.orientation_covariance[0]!=-1)
        if orientation is None or not math.isfinite(m.angular_velocity.z):
            self.sensor_faults.add('imu');return
        self.sensor_faults.discard('imu')
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
        seg=self.driver.segment
        self.ground_height=float(self.road[nearest,2]) if seg is None or seg.surface=='perimeter' else (.002 if seg.surface=='yard' else 0.)
        if not self.enabled:
            # A manual drive may cross a segment boundary before the mission is
            # explicitly resumed. Do not keep the old segment's road height.
            self.ground_height=float(self.road[nearest,2]) if np.linalg.norm(self.road[nearest,:2]-self.pose[:2])<.155 else 0.
        good=np.isfinite(self.ranges)&(self.ranges>=.04)&(self.ranges<4.8)
        indices=np.flatnonzero(good)[::2]
        ranges=self.ranges[indices];angles=self.angles[indices]
        local=np.c_[ranges*np.cos(angles)+.075,ranges*np.sin(angles),np.full(len(indices),.065)]
        world=local@rotation(self.roll,self.pitch,self.pose[2]).T
        world+=np.array([*self.pose[:2],self.ground_height+.028])
        delta,valid=self.landmarks.correction(world,tracking=True)
        self.localization_delta=delta.tolist()
        self.localization_good=bool(valid and consistent_correction(delta,self.pose,self.route,self.driver.progress))
        parking=seg is not None and seg.kind=='parking'
        if parking:
            external=outside_robot(local)
            local_delta,local_valid=self.mapper.correction(world[external])
            local_valid=bool(local_valid and consistent_correction(local_delta,self.pose,self.route,self.driver.progress,limit=.025))
            if local_valid:
                if self.localization_good:
                    if np.linalg.norm(local_delta-delta)<.03:delta=.7*delta+.3*local_delta
                else:delta=local_delta;self.localization_good=True
        if self.localization_good:
            self.pose[:2]+=delta*.7
            world[:,:2]+=delta*.7
            self.mark('localization',m)
        else:self.localization_rejected+=1
        if parking and self.localization_good:
            sensor_origin=self.pose[:2]+(rotation(self.roll,self.pitch,self.pose[2])@np.array([.075,0.,.065]))[:2]
            self.mapper.update(sensor_origin,world[external])

    def image(self,m):
        if m.encoding not in ('rgb8','bgr8') or not m.width or not m.height or m.step<m.width*3 or len(m.data)!=m.height*m.step:
            self.sensor_faults.add('image');return
        self.sensor_faults.discard('image')
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
        self.observation.fresh=self.freshness.ready(now,wall) and not self.sensor_faults
        self.observation.pitch=self.pitch
        self.observation.localization_age=max(ages['localization'],source_ages['localization'])
        self.observation.measured_speed=self.measured_speed
        self.observation.pose_age=tracking_age(ages,source_ages)
        if self.ranges is not None and self.observation.fresh and self.enabled:
            self.path_curvature=self.driver.tracking_curvature(self.pose)
            self.observation.clearance=obstacle_distance(self.ranges,self.angles,self.path_curvature)
        command=self.driver.step(self.pose,self.observation,now,enabled=self.enabled)
        dt=0. if self.control_time is None else now-self.control_time
        self.control_time=now
        command=(command[0],self.yaw_controller.step(command[1],self.yaw_rate,dt,stopped=command[0]==0.))
        if self.driver.progress<self.last_progress-.05:
            self.last_progress=self.driver.progress;self.progress_time=wall
        if self.driver.progress-self.last_progress>.01:
            self.last_progress=self.driver.progress;self.progress_time=wall
        moving_states=('DRIVE','CROSSING','ROUNDABOUT','PARKING_AREA','SLOPE','APPROACH_SIGNAL')
        if self.driver.state in moving_states and command[0]>.025 and wall-self.progress_time>15:
            command=self.driver.stop('FAULT_STOP','no progress for 15 seconds')
        elif self.driver.state not in moving_states:self.progress_time=wall
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
        if self.mapper.scans and wall-self.last_map>1.:
            self.last_map=wall
            grid=OccupancyGrid();grid.header.stamp=self.get_clock().now().to_msg();grid.header.frame_id='map'
            grid.info.resolution=self.mapper.resolution;grid.info.width=132;grid.info.height=72
            grid.info.origin.position.y=3.5;grid.info.origin.orientation.w=1.
            grid.data=self.mapper.occupancy().ravel().tolist();self.map_pub.publish(grid)
        if self.driver.state=='COMPLETE' and not self.map_saved:
            self.map_saved=True
            if self.mapper.scans:self.mapper.save(self.map_output)
            self.get_logger().info('MISSION COMPLETE')
        if wall-self.last_report>.25:
            self.last_report=wall
            data=dict(state=self.driver.state,reason=self.driver.reason,progress=round(self.driver.progress,3),length=round(self.route.length,3),pose=self.pose.tolist(),pitch=self.pitch,localized=self.localization_good,lane_valid=self.observation.lane_valid,lane_error=self.observation.lane_error,light=self.observation.light,signal=self.observation.light_id,clearance=self.observation.clearance if math.isfinite(self.observation.clearance) else None,ages={k:v if math.isfinite(v) else None for k,v in ages.items()},command=command,sim_time=now,frame=self.frame,stop_distance=self.observation.stop_distance)
            data.update(self.route.status(self.driver.progress) if isinstance(self.route,Mission) else dict(mission='perimeter',segment='perimeter',segment_index=0,segment_progress=self.driver.progress,mission_progress=self.driver.progress/self.route.length))
            if self.driver.state=='COMPLETE':data['mission_progress']=1.
            data.update(driving_mode=self.driving_mode,speed_target=self.driver.speed_target,tracking_curvature=self.path_curvature,localization_delta=self.localization_delta,localization_rejected=self.localization_rejected,measured_speed=self.measured_speed,mapping_scans=self.mapper.scans,mapping_matches=self.mapper.matches,green_cycle_ready=self.driver.green_cycle_ready,green_frames=self.driver.green_frames,committed=self.driver.committed,expected_signal=self.driver.signals[self.driver.signal_index]['id'] if self.driver.signal_index<len(self.driver.signals) else '')
            self.status_pub.publish(String(data=json.dumps(data,allow_nan=False)))


def main():
    rclpy.init();n=Autonomy()
    try:rclpy.spin(n)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if n.mapper.scans and not n.map_saved:n.mapper.save(n.map_output)
        if rclpy.ok():n.pub.publish(Twist())
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
