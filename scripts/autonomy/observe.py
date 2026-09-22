#!/usr/bin/env python3
"""Bounded recording of sensor/status data. Does not issue vehicle commands."""
import argparse,json,time
from pathlib import Path
import cv2,numpy as np,rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,LaserScan
from std_msgs.msg import String


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seconds',type=float,default=30);parser.add_argument('--output',default='/tmp/dongfeng_observation');args=parser.parse_args()
    root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('autonomy_observer');records=[];counts={'image':0,'scan':0};last_print=[0.]
    def status(m):
        data=json.loads(m.data);data['wall_time']=time.time();records.append(data)
        if time.monotonic()-last_print[0]>2:
            print(json.dumps(data),flush=True);last_print[0]=time.monotonic()
    def image(m):
        counts['image']+=1
        if counts['image']%10==0:
            a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
            if m.encoding=='rgb8':a=a[:,:,::-1]
            cv2.imwrite(str(root/'camera.png'),a)
    def scan(m):
        counts['scan']+=1
        if counts['scan']%20==0:print('scan finite',int(np.isfinite(m.ranges).sum()),flush=True)
    node.create_subscription(String,'/autonomy/status',status,5)
    node.create_subscription(Image,'/autonomy/debug_image',image,qos_profile_sensor_data)
    node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)
    end=time.monotonic()+args.seconds
    try:
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    finally:
        (root/'status.json').write_text(json.dumps(records,indent=2));print(counts,flush=True)
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
