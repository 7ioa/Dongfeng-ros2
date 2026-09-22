"""Sensor configuration contracts for the generated vehicle."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]


class SensorsTest(unittest.TestCase):
    def test_vehicle_has_real_sensors(self):
        tree = ET.parse(ROOT / 'src/dongfeng_description/urdf/dongfeng_car.urdf.xacro')
        sensors = {s.get('type'): s for s in tree.findall('.//sensor')}
        self.assertTrue({'camera', 'gpu_lidar', 'imu'} <= sensors.keys())
        self.assertEqual(sensors['camera'].findtext('topic'), '/camera/image_raw')
        self.assertEqual(sensors['camera'].findtext('gz_frame_id'), 'camera_optical_frame')
        self.assertEqual(sensors['gpu_lidar'].findtext('topic'), '/scan')
        self.assertEqual(sensors['imu'].findtext('topic'), '/imu/data')

    def test_bridge_sensor_types(self):
        import yaml
        entries = yaml.safe_load((ROOT / 'src/dongfeng_bringup/config/bridge.yaml').read_text())
        names = {e['ros_topic_name']: e['ros_type_name'] for e in entries}
        for topic, kind in [('image_raw', 'Image'), ('camera_info', 'CameraInfo')]:
            self.assertEqual(names.get('/camera/' + topic), 'sensor_msgs/msg/' + kind)
        self.assertEqual(names.get('/scan'), 'sensor_msgs/msg/LaserScan')
        self.assertEqual(names.get('/imu/data'), 'sensor_msgs/msg/Imu')

class WheelSlipTest(unittest.TestCase):
    def test_all_wheels_have_lateral_slip_model(self):
        tree=ET.parse(ROOT/'src/dongfeng_description/urdf/dongfeng_car.urdf.xacro')
        plugin=tree.find('.//plugin[@name="gz::sim::systems::WheelSlip"]')
        self.assertIsNotNone(plugin)
        self.assertEqual(len(plugin.findall('wheel')),4)
        for wheel in plugin.findall('wheel'):
            self.assertGreater(float(wheel.findtext('slip_compliance_lateral')),0)
            self.assertEqual(float(wheel.findtext('slip_compliance_longitudinal')),0)

    def test_collision_friction_direction_is_parallel_to_axle(self):
        # DART defaults to the collision shape frame, which is rotated -90°
        # about X to put the cylinder's Z axis along the wheel's Y axle.
        import math
        import numpy as np
        tree=ET.parse(ROOT/'src/dongfeng_description/urdf/dongfeng_car.urdf.xacro')
        for joint in tree.findall("joint[@type='continuous']"):
            name=joint.find('child').get('link')
            collision=tree.find(f"link[@name='{name}']/collision")
            roll,pitch,yaw=map(float,collision.find('origin').get('rpy').split())
            self.assertAlmostEqual(pitch,0);self.assertAlmostEqual(yaw,0)
            c,s=math.cos(roll),math.sin(roll)
            direction=np.array(list(map(float,tree.find(f"gazebo[@reference='{name}']/fdir1").text.split())))
            direction=np.array([[1,0,0],[0,c,-s],[0,s,c]])@direction
            axis=np.array(list(map(float,joint.find('axis').get('xyz').split())))
            self.assertAlmostEqual(abs(float(np.dot(direction,axis))),1.,places=6)

    def test_friction_survives_actual_urdf_to_sdf_conversion(self):
        import shutil
        import subprocess
        if shutil.which('gz') is None:self.skipTest('Gazebo CLI not sourced')
        converted=subprocess.run(['gz','sdf','-p',str(ROOT/'src/dongfeng_description/urdf/dongfeng_car.urdf.xacro')],capture_output=True,text=True,check=True)
        tree=ET.fromstring(converted.stdout)
        wheels=[link for link in tree.findall('.//link') if link.get('name').startswith('wheel_')]
        self.assertEqual(len(wheels),4)
        for link in wheels:
            surface=link.find('collision/surface/friction/ode')
            self.assertIsNotNone(surface,link.get('name'))
            self.assertEqual(list(map(float,surface.findtext('fdir1').split())),[0.,0.,1.])
            self.assertGreaterEqual(float(surface.findtext('mu')),0.9)
