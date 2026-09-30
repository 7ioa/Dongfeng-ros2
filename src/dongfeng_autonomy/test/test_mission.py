import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from dongfeng_autonomy.route import Route
from dongfeng_autonomy.mission import Mission, RouteSegment
from dongfeng_autonomy.control import Driver, Observation
from dongfeng_autonomy.sensing import Freshness
from dongfeng_autonomy.localization import consistent_correction
from dongfeng_autonomy.mapping import ParkingMapper, outside_robot

CONFIG=Path(__file__).resolve().parents[1]/'config'
def mission():return Mission.load(CONFIG/'full_demo.json',json.loads((CONFIG/'signals.json').read_text()))

class MissionTest(unittest.TestCase):
    def test_invalid_configuration_fails_closed(self):
        for option in ({'speed':math.nan},{'lookahead':-.1},{'completion_distance':0},{'kind':'unknown'}):
            with self.assertRaises(ValueError):
                Mission('bad',[RouteSegment('one',Route([[0,0],[1,0]],False),**option)])

    def test_ordered_segments_and_signal_directions(self):
        m=mission()
        self.assertEqual(len(m.segments),19)
        self.assertEqual([s['id'] for s in m.signals],['signal_1','signal_6','signal_2'])
        for sig in m.signals:
            seg=m.segments[sig['segment_index']]
            heading=seg.route.heading(sig['stop_s']-m.offsets[sig['segment_index']])
            approach=np.array([math.cos(heading),math.sin(heading)])
            front=sig['side']*np.array([math.sin(sig['angle']),-math.cos(sig['angle'])])
            self.assertLess(float(approach@front),-.95)

    def test_open_path_never_wraps_at_endpoint(self):
        r=Route([[0,0],[1,0]],closed=False)
        np.testing.assert_allclose(r.target(2),[1,0])
        self.assertGreater(abs(r.project(2,0)[1]),.99)

    def test_crossing_cannot_select_future_visit(self):
        a=RouteSegment('first',Route([[-1,0],[1,0]],False),next_segment='later')
        b=RouteSegment('later',Route([[1,0],[0,-1],[0,1]],False))
        m=Mission('cross',[a,b])
        self.assertAlmostEqual(m.project(0,0,1)[0],1)
        self.assertEqual(m.index,0)
        m.index=1
        self.assertGreater(m.project(0,0)[0],m.offsets[1])

    def test_wrong_projection_window_and_teleport_stop(self):
        m=mission();d=Driver(m,m.signals)
        self.assertEqual(d.step((3.11,2.,math.pi/2),Observation(),0),(0.,0.))
        self.assertEqual(d.state,'FAULT_STOP')

    def test_all_segments_complete_with_no_lanes_in_special_areas(self):
        m=mission();d=Driver(m,m.signals);p=np.array([1.65,.19,0.]);seen=set()
        for i in range(12000):
            sid=d.signals[d.signal_index]['id'] if d.signal_index<len(d.signals) else ''
            obs=Observation(frame=i,light='green' if d.green_cycle_ready else 'red',light_id=sid,lane_valid=m.active.lane_required)
            v,w=d.step(p,obs,i*.05);p += [.05*v*math.cos(p[2]),.05*v*math.sin(p[2]),.05*w]
            seen.add(m.active.kind)
            if d.state in ('FAULT_STOP','COMPLETE'):break
        self.assertEqual(d.state,'COMPLETE',d.reason)
        self.assertEqual(m.index,len(m.segments)-1)
        self.assertTrue({'roundabout','parking','slope'}<=seen)
        self.assertLess(math.dist(p[:2],[1.65,.19]),.08)

    def test_resume_prefers_current_visit_and_rejects_distant_segment(self):
        m=mission();m.index=7;s=m.offsets[7]+.5;p=(*m.target(s),m.active.route.heading(.5))
        d=Driver(m,m.signals);d.progress=s-.3
        d.step(p,Observation(),0,False)
        self.assertEqual(d.state,'MANUAL')
        self.assertGreater(d.step(p,Observation(),.1,True)[0],0)
        self.assertEqual(m.index,7)
        self.assertAlmostEqual(d.progress,s,places=3)
        self.assertFalse(d.resume((1.65,.19,0)))

    def test_localization_tolerance_is_bounded(self):
        f=Freshness(('scan','odom','imu','localization'),.5,{'localization':1.5})
        for k in f.required:f.update(k,0,0)
        for k in ('scan','odom','imu'):f.update(k,1,1)
        self.assertTrue(f.ready(1.1,1.1))
        self.assertFalse(f.ready(1.6,1.6))
        d=Driver(mission(),[])
        self.assertLessEqual(d.step((1.65,.19,0),Observation(localization_age=.8),0)[0],.09)
        self.assertEqual(d.step((1.65,.19,0),Observation(localization_age=1.6),.1),(0.,0.))

    def test_slope_retains_traction_speed_and_obstacle_protection(self):
        m=mission();m.index=11;d=Driver(m,m.signals);d.progress=m.offsets[11]+.10
        pose=(*m.target(d.progress),math.pi/2)
        self.assertGreater(d.step(pose,Observation(pitch=-.23,lane_valid=False),0)[0],.12)
        self.assertEqual(d.state,'SLOPE')
        self.assertEqual(d.step(pose,Observation(clearance=.02,pitch=-.23),.1),(0.,0.))
        self.assertEqual(d.state,'OBSTACLE_STOP')

    def test_scan_innovation_gate_rejects_crest_runaway(self):
        m=mission();m.index=11;s=m.offsets[11]+.8;pose=np.r_[m.target(s),0]
        self.assertFalse(consistent_correction(np.array([-.12,-.03]),pose,m,s))
        self.assertTrue(consistent_correction(np.array([.002,-.001]),pose,m,s))

    def test_fault_requires_explicit_reenable(self):
        d=Driver(mission(),[]);pose=(1.65,.19,0.)
        d.step(pose,Observation(clearance=math.nan),0)
        self.assertEqual(d.step(pose,Observation(),.1),(0.,0.))
        d.step(pose,Observation(),.2,False)
        self.assertGreater(d.step(pose,Observation(),.3,True)[0],0.)

    def test_perimeter_resume_status_is_json_serializable(self):
        route=Route.perimeter(3.3,5.4,.04,.8,.3)
        driver=Driver(route,[dict(id='lamp',stop_s=3.)])
        self.assertTrue(driver.resume(np.array([1.65,.19,0.])))
        status=json.loads(json.dumps(dict(progress=driver.progress,committed=driver.committed)))
        self.assertIs(status['committed'],False)

class MappingTest(unittest.TestCase):
    def test_self_returns_do_not_leave_a_trail_in_empty_parking_space(self):
        m=ParkingMapper();wall=np.c_[np.full(100,2.),np.linspace(4.1,4.9,100)]
        for x in np.linspace(.8,1.2,6):
            pose=np.array([x,4.4]);supports=pose+np.c_[np.linspace(-.1,.1,30),np.zeros(30)]
            world=np.vstack((wall,supports));local=world-pose
            m.update(pose+[.075,0.],world[outside_robot(local)])
        self.assertGreaterEqual(m.landmarks[:,0].min(),2.)
        # A real return immediately outside the body must remain observable.
        self.assertTrue(outside_robot(np.array([[.104,0.]]))[0])

    def test_online_submap_and_export(self):
        m=ParkingMapper();y=np.linspace(3.8,5.1,100)
        p=np.vstack([np.c_[np.full(100,.6),y],np.c_[np.linspace(.6,2.7,100),np.full(100,5.1)]])
        for _ in range(4):m.update(np.array([1.6,4.6]),p)
        correction,valid=m.correction(p+np.array([.013,-.009]))
        self.assertTrue(valid)
        self.assertLess(np.linalg.norm(correction+np.array([.013,-.009])),.01)
        self.assertTrue((m.occupancy()>65).any())
        self.assertTrue((m.occupancy()<35).any())
        with tempfile.TemporaryDirectory() as tmp:
            m.save(tmp)
            self.assertTrue((Path(tmp)/'parking.pgm').exists())
            with np.load(Path(tmp)/'parking_slam.npz') as saved:
                self.assertEqual(int(saved['scans']),m.scans)
                self.assertEqual(int(saved['matches']),m.matches)
                np.testing.assert_array_equal(saved['logodds'],m.logodds)

    def test_interrupted_export_preserves_previous_archive(self):
        m=ParkingMapper();m.scans=5
        with tempfile.TemporaryDirectory() as tmp:
            m.save(tmp)
            def interrupted(stream,**arrays):
                stream.write(b'partial archive')
                raise OSError('interrupted write')
            m.scans=6
            with patch('dongfeng_autonomy.mapping.np.savez_compressed',side_effect=interrupted):
                with self.assertRaises(OSError):m.save(tmp)
            with np.load(Path(tmp)/'parking_slam.npz') as saved:self.assertEqual(int(saved['scans']),5)

    def test_export_preserves_unknown_free_and_occupied_cells(self):
        m=ParkingMapper();m.seen[0,:2]=True;m.logodds[0,:2]=[-4,4]
        with tempfile.TemporaryDirectory() as tmp:
            m.save(tmp);root=Path(tmp)
            metadata=dict(line.split(': ',1) for line in (root/'parking.yaml').read_text().splitlines())
            raw=(root/'parking.pgm').read_bytes().split(b'\n',3)[3]
            occupancy=1-np.frombuffer(raw,np.uint8).reshape(72,132)[::-1]/255.
            free=float(metadata['free_thresh']);occupied=float(metadata['occupied_thresh'])
            self.assertLess(occupancy[0,0],free)
            self.assertGreater(occupancy[0,1],occupied)
            self.assertTrue(free<occupancy[1,1]<occupied)

class MissionEvaluationTest(unittest.TestCase):
    def trajectory(self):
        m=mission();truth=[];t=0.
        for i,seg in enumerate(m.segments):
            for s in np.arange(0,seg.route.length,.015):
                x,y=seg.route.target(s)
                truth.append(dict(t=t,x=x,y=y,yaw=seg.route.heading(s),progress=float(m.offsets[i]+s),error=0.,segment_index=i));t+=.1
        for _ in range(26):
            truth.append(dict(t=t,x=1.65,y=.19,yaw=0.,progress=m.length,error=0.,segment_index=len(m.segments)-1));t+=.1
        return m,truth

    def test_complete_ordered_mission_and_final_stop(self):
        from dongfeng_autonomy.evaluation import evaluate_mission
        m,truth=self.trajectory()
        self.assertTrue(evaluate_mission(truth,True,m)['lap_pass'])
        self.assertFalse(evaluate_mission(truth,False,m)['lap_pass'])
        self.assertFalse(evaluate_mission(truth[:-25],True,m)['lap_pass'])

    def test_skipped_or_teleported_segments_fail(self):
        from dongfeng_autonomy.evaluation import evaluate_mission
        m,truth=self.trajectory()
        self.assertFalse(evaluate_mission([p for p in truth if p['segment_index']!=7],True,m)['lap_pass'])
        truth[len(truth)//2]['x']+=.5
        self.assertFalse(evaluate_mission(truth,True,m)['lap_pass'])

class FreshGreenCycleTest(unittest.TestCase):
    def test_existing_green_cannot_release_but_new_cycle_does(self):
        r=Route([[0,0],[2,0]],False)
        d=Driver(r,[dict(id='lamp',stop_s=.4,line_s=.5382,require_new_green=True)])
        for i in range(4):
            self.assertEqual(d.step((.37,0,0),Observation(light='green',light_id='lamp',frame=i),i*.1),(0.,0.))
        self.assertIn('cycle',d.reason)
        d.step((.37,0,0),Observation(light='red',light_id='lamp',frame=4),.4)
        self.assertTrue(d.green_cycle_ready)
        for i in (5,6):self.assertEqual(d.step((.37,0,0),Observation(light='green',light_id='lamp',frame=i),i*.1),(0.,0.))
        self.assertGreater(d.step((.37,0,0),Observation(light='green',light_id='lamp',frame=7),.7)[0],0.)

    def test_short_outage_requires_new_frames_within_observed_transition(self):
        d=Driver(Route([[0,0],[2,0]],False),[dict(id='lamp',stop_s=.4,require_new_green=True)])
        d.step((.37,0,0),Observation(light='red',light_id='lamp',frame=0),0.)
        self.assertTrue(d.green_cycle_ready)
        d.step((.37,0,0),Observation(fresh=False),.1)
        self.assertTrue(d.green_cycle_ready)
        for i in (1,2):self.assertEqual(d.step((.37,0,0),Observation(light='green',light_id='lamp',frame=i),.1+i*.1),(0.,0.))
        self.assertGreater(d.step((.37,0,0),Observation(light='green',light_id='lamp',frame=3),.4)[0],0.)

    def test_old_red_observation_cannot_authorize_late_green(self):
        for stale in (True,False):
            d=Driver(Route([[0,0],[2,0]],False),[dict(id='lamp',stop_s=.4,require_new_green=True)])
            d.step((.37,0,0),Observation(light='red',light_id='lamp',frame=0),0.)
            d.step((.37,0,0),Observation(fresh=not stale),2.1)
            for i in (1,2,3):self.assertEqual(d.step((.37,0,0),Observation(light='green',light_id='lamp',frame=i),2.1+i*.1),(0.,0.))
        self.assertFalse(d.green_cycle_ready)
