"""Physical speed-envelope and fast-mode safety regressions."""
import math
from pathlib import Path
import unittest
import numpy as np
from dongfeng_autonomy.route import Route
from dongfeng_autonomy.mission import Mission, RouteSegment
from dongfeng_autonomy.control import Driver, Observation, CommandMux
from dongfeng_autonomy.speed import SpeedProfile, SpeedPlanner, braking_limit


def two_segments():
    return Mission('preview', [
        RouteSegment('straight', Route([[0,0],[2,0]], False), next_segment='narrow'),
        RouteSegment('narrow', Route([[2,0],[3,0]], False), speed=.075, kind='parking', corridor=.045)])


class SpeedTest(unittest.TestCase):
    def test_default_arbiter_preserves_fast_auto_speed_and_manual_limit(self):
        mux=CommandMux();mux.enable(True)
        self.assertEqual(mux.sample(.1,(.4,0.),0.),(.4,0.))
        mux.manual((.4,0.),.2)
        self.assertEqual(mux.sample(.3,(.4,0.),.3),(.25,0.))

    def test_delay_and_braking_distance_are_respected(self):
        for distance in (0., .02, .1, .3, .6):
            v=braking_limit(distance, 0., .25, .25)
            self.assertLessEqual(v*.25+v*v/(2*.25), distance+1e-9)
        self.assertEqual(braking_limit(0., 0., .25, .25), 0.)

    def test_slows_before_next_segment_and_accelerates_after_turn(self):
        planner=SpeedPlanner(two_segments(), SpeedProfile.fast())
        self.assertGreater(planner.limit(.4), .30)
        self.assertLess(planner.limit(1.85), .26)
        self.assertLessEqual(planner.limit(2.01), .095)
        r=Route([[0,0],[1,0],[1,1],[1,3]], False)
        p=SpeedPlanner(r, SpeedProfile.fast())
        self.assertLess(p.limit(.9),p.limit(.2))
        self.assertGreater(p.limit(2.5),p.limit(1.0))

    def test_curvature_caps_lateral_acceleration(self):
        angles=np.linspace(0,math.pi,200)
        p=SpeedPlanner(Route(np.c_[.4*np.cos(angles),.4*np.sin(angles)],False),SpeedProfile.fast())
        self.assertLessEqual(p.limit(.5)**2/.4,.046)

    def test_startup_and_recovery_ramp_use_sim_time(self):
        d=Driver(Route([[0,0],[4,0]],False),[],speed=.4,profile=SpeedProfile.fast())
        first=d.step((0,0,0),Observation(),10)[0]
        second=d.step((.001,0,0),Observation(),10.05)[0]
        self.assertLessEqual(first,.02)
        self.assertLessEqual(second-first,.25*.05+1e-8)
        self.assertEqual(d.step((.002,0,0),Observation(fresh=False),10.1),(0.,0.))
        self.assertLessEqual(d.step((.002,0,0),Observation(),10.15)[0],.02)

    def test_high_measured_speed_stops_even_if_target_is_low(self):
        d=Driver(Route([[0,0],[4,0]],False),[],speed=.4,profile=SpeedProfile.fast())
        self.assertEqual(d.step((0,0,0),Observation(clearance=.25,measured_speed=.4),0),(0.,0.))
        self.assertEqual(d.state,'OBSTACLE_STOP')

    def test_stop_line_in_next_segment_is_previewed(self):
        m=two_segments();sig=dict(id='lamp',stop_s=2.1,line_s=2.2382,exit_s=2.6,segment_index=1)
        d=Driver(m,[sig],speed=.4,profile=SpeedProfile.fast())
        for i in range(50):v,_=d.step((1.8,0,0),Observation(),i*.05)
        self.assertLessEqual(v,braking_limit(.30-.035,0.,.25,.25)+1e-8)
        self.assertFalse(d.committed)

    def test_profile_and_mux_reject_invalid_configuration(self):
        for speed in (math.nan, math.inf,0.,-.1,.41):
            with self.assertRaises(ValueError):SpeedProfile.fast(max_speed=speed)
        m=CommandMux(auto_limit=.4);m.enable(True)
        self.assertEqual(m.sample(.1,(.4,0),0),(.4,0))
        m.manual((.4,0),.2)
        self.assertEqual(m.sample(.3,(.4,0),.3),(.25,0))
        self.assertEqual(m.sample(1.,(.4,0),0),(0.,0.))

    def test_original_profile_is_unchanged(self):
        m=two_segments();d=Driver(m,[])
        self.assertEqual(d.step((0,0,0),Observation(),0)[0],.16)
        with self.assertRaises(ValueError):Driver(m,[],speed=.4)

    def test_fast_full_mission_completes_with_original_corridors(self):
        import json
        root=Path(__file__).parents[1]/'config'
        m=Mission.load(root/'full_demo.json',json.loads((root/'signals.json').read_text()))
        d=Driver(m,[],speed=.4,profile=SpeedProfile.fast())
        p=np.array([1.65,.19,0.]);seen=set()
        for i in range(12000):
            v,w=d.step(p,Observation(measured_speed=0.),i*.05)
            seen.add(m.index);p+=[.05*v*math.cos(p[2]),.05*v*math.sin(p[2]),.05*w]
            if d.state in ('COMPLETE','FAULT_STOP'):break
        self.assertEqual(d.state,'COMPLETE',d.reason)
        self.assertEqual(len(seen),19)


class ClosedRouteStopTest(unittest.TestCase):
    def test_fast_perimeter_slows_before_return_to_start(self):
        route=Route.perimeter(3.3,5.4,.04,.8,.3)
        d=Driver(route,[],.4,SpeedProfile.fast());s=route.length-.15
        d.progress=s
        pose=(*route.target(s),route.heading(s))
        d.command_speed=.4;d.last_time=0.
        v,_=d.step(pose,Observation(measured_speed=.4),.05)
        self.assertLessEqual(v,.121)

class BlockedRecoveryTest(unittest.TestCase):
    def test_blocked_car_does_not_creep_during_acceleration_ramp(self):
        d=Driver(Route([[0,0],[4,0]],False),[],.4,SpeedProfile.fast())
        for i in range(10):
            self.assertEqual(d.step((0,0,0),Observation(clearance=.30,measured_speed=0.),i*.05),(0.,0.))
            self.assertEqual(d.state,'OBSTACLE_STOP')

    def test_lower_target_does_not_release_an_existing_obstacle_stop(self):
        d=Driver(Route([[0,0],[4,0]],False),[],.4,SpeedProfile.fast())
        self.assertEqual(d.step((0,0,0),Observation(clearance=.30,measured_speed=.4),0),(0.,0.))
        # A box can hide the lane; localization can also request the same low cap.
        # Neither condition means the blocking object has moved away.
        for i in range(1,10):
            obs=Observation(clearance=.30,lane_valid=False,localization_age=.6)
            self.assertEqual(d.step((0,0,0),obs,i*.05),(0.,0.))
            self.assertEqual(d.state,'OBSTACLE_STOP')
        v,_=d.step((0,0,0),Observation(clearance=math.inf),.5)
        self.assertGreater(v,0.)
        self.assertLessEqual(v,.0125+1e-8)

class PreviewDelayTest(unittest.TestCase):
    def test_actual_speed_and_pose_age_make_stop_limit_more_conservative(self):
        p=SpeedPlanner(Route([[0,0],[4,0]],False),SpeedProfile.fast())
        slow=p.stop_limit(.1,measured_speed=0.)
        fast=p.stop_limit(.1,measured_speed=.4)
        stale=p.stop_limit(.3,measured_speed=.3,pose_age=.4)
        self.assertEqual(fast,0.)
        self.assertEqual(p.stop_limit(.1,measured_speed=.25),0.)
        self.assertLess(stale,p.stop_limit(.3,measured_speed=.3))
        self.assertGreater(slow,0.)

    def test_safety_margins_cannot_be_disabled_by_configuration(self):
        for kwargs in (dict(planning_delay=1e-12),dict(margin=1e-12),dict(obstacle_delay=.55),dict(sample_distance=1e-12)):
            with self.assertRaises(ValueError):SpeedProfile.fast(**kwargs)
