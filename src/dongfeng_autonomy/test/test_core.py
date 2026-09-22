import math
import unittest
import numpy as np
from dongfeng_autonomy.route import Route
from dongfeng_autonomy.control import Driver, Observation, CommandMux, obstacle_distance


def route():
    return Route.perimeter(3.3, 5.4, .04, .8, .3)


class CoreTest(unittest.TestCase):
    def test_route_start_and_closed_progress(self):
        r = route()
        self.assertLess(np.linalg.norm(r.target(0) - [1.65, .19]), .001)
        self.assertLess(np.linalg.norm(r.target(r.length) - r.target(0)), .001)
        self.assertGreater(r.length, 14)
        self.assertLess(r.length, 16)
        self.assertLess(r.project(1.65, .19, 0)[0], .01)

    def test_closed_loop_completes_once(self):
        d = Driver(route(), [])
        p = np.array([1.65, .19, 0.])
        self.assertNotEqual(d.state, 'COMPLETE')
        for i in range(10000):
            o = Observation(lane_valid=True)
            v,w = d.step(p,o,i*.05)
            p += [.05*v*math.cos(p[2]), .05*v*math.sin(p[2]), .05*w]
            if d.state=='COMPLETE':break
        self.assertEqual(d.state,'COMPLETE')
        self.assertLess(np.linalg.norm(p[:2]-[1.65,.19]),.08)
        self.assertEqual(d.step(p,o,1000),(0.,0.))

    def test_bad_sensor_stops(self):
        d=Driver(route(),[])
        for o in [Observation(fresh=False),Observation(clearance=.05)]:
            self.assertEqual(d.step((1.65,.19,0),o,0),(0.,0.))
        self.assertEqual(d.step((float('nan'),0,0),Observation(),1),(0.,0.))

    def test_light_stop_then_stable_green(self):
        r=route(); d=Driver(r,[{'id':'test','stop_s':.40}])
        p=(2.02,.19,0)
        for i in range(4):
            self.assertEqual(d.step(p,Observation(light='red',light_id='test',frame=i),i*.05),(0.,0.))
        self.assertEqual(d.step(p,Observation(light='green',light_id='other',frame=5),.25),(0.,0.))
        for i in (6,7):self.assertEqual(d.step(p,Observation(light='green',light_id='test',frame=i),i*.05),(0.,0.))
        self.assertGreater(d.step(p,Observation(light='green',light_id='test',frame=8),.4)[0],0)

    def test_duplicate_green_frames_never_release(self):
        d=Driver(route(),[{'id':'test','stop_s':.4}])
        for i in range(10):
            self.assertEqual(d.step((2.02,.19,0),Observation(light='green',light_id='test',frame=1),i*.05),(0.,0.))

    def test_manual_stop_latches(self):
        m=CommandMux();m.enable(True)
        self.assertEqual(m.sample(.1,(.1,0),0),( .1,0))
        m.manual((0,0),.2)
        self.assertEqual(m.sample(.3,(.1,0),.3),(0,0))
        m.enable(True)
        self.assertEqual(m.sample(.4,(.1,0),.4),(.1,0))
        self.assertEqual(m.sample(1.,(.1,0),.4),(0,0))

    def test_obstacle_front_and_side(self):
        a=np.linspace(-math.pi, math.pi,720)
        ranges=np.full(720,math.inf);ranges[np.argmin(abs(a))]=.20
        self.assertLess(obstacle_distance(ranges,a,0),.25)
        ranges[:]=math.inf;ranges[np.argmin(abs(a-math.pi/2))]=.20
        self.assertEqual(obstacle_distance(ranges,a,0),math.inf)


if __name__=='__main__':unittest.main()

class SafetyRegressionTest(unittest.TestCase):
    def test_invalid_scan_is_not_clear(self):
        for ranges in ([],[float('nan')]*20,[0.]*20,[-float('inf')]*20):
            self.assertTrue(math.isnan(obstacle_distance(ranges,np.linspace(-1,1,len(ranges)))))

    def test_clock_reset_remains_stopped(self):
        d=Driver(route(),[]);d.step((1.65,.19,0),Observation(),10)
        for now in (1,11,12):self.assertEqual(d.step((1.65,.19,0),Observation(),now),(0.,0.))

    def test_yaw_feedback_compensates_slip(self):
        from dongfeng_autonomy.control import YawController
        c=YawController();measured=0.
        for _ in range(200):
            cmd=c.step(.15,measured,.05)
            measured += .2*(.45*cmd-measured)
        self.assertLess(abs(measured-.15),.015)
        self.assertEqual(c.step(0,measured,.05,stopped=True),0.)


class SignalBoundaryTest(unittest.TestCase):
    def driver(self):
        return Driver(route(), [{'id': 'test', 'stop_s': .4, 'line_s': .5382}])

    def green(self, d, x):
        for frame in range(1, 4):
            d.step((x, .19, 0), Observation(light='green', light_id='test', frame=frame), frame*.05)

    def test_green_to_red_before_line_does_not_commit(self):
        d = self.driver()
        self.green(d, 1.98)  # 7 cm before the center stopping target
        command = d.step((2.02, .19, 0), Observation(light='red', light_id='test', frame=4), .2)
        self.assertEqual(command, (0., 0.))
        self.assertFalse(d.committed)

    def test_manual_hold_does_not_advance_or_accumulate_green(self):
        d = self.driver()
        self.green(d, 2.02)
        for frame in range(4, 8):
            self.assertEqual(d.step((2.02, .19, 0), Observation(light='green', light_id='test', frame=frame), frame*.05, enabled=False), (0., 0.))
        self.assertEqual(d.step((2.02, .19, 0), Observation(light='red', light_id='test', frame=8), .4), (0., 0.))
        self.assertFalse(d.committed)

    def test_crossing_latches_only_after_front_passes_line(self):
        d = self.driver()
        self.green(d, 2.02)
        self.assertFalse(d.committed)
        d.step((2.095, .19, 0), Observation(light='green', light_id='test', frame=4), .2)
        self.assertTrue(d.committed)
        self.assertGreater(d.step((2.10, .19, 0), Observation(light='red', light_id='test', frame=5), .25)[0], 0.)

    def test_manual_mode_does_not_unlock_completed_lap(self):
        d = Driver(route(), [])
        d.state = 'COMPLETE'
        self.assertEqual(d.step((1.65, .19, 0), Observation(), 1., enabled=False), (0., 0.))
        self.assertEqual(d.step((1.65, .19, 0), Observation(), 2.), (0., 0.))
        self.assertEqual(d.state, 'COMPLETE')


class ScanCoverageTest(unittest.TestCase):
    def test_front_nan_with_clear_rear_is_not_clear(self):
        angles = np.linspace(-math.pi, math.pi, 720)
        ranges = np.full(720, math.inf)
        ranges[abs(angles) < math.pi/6] = math.nan
        self.assertTrue(math.isnan(obstacle_distance(ranges, angles)))

    def test_narrow_blind_patch_in_swept_turn_is_not_clear(self):
        angles = np.linspace(-math.pi, math.pi, 720)
        ranges = np.full(720, math.inf)
        ranges[(angles > .9) & (angles < 1.05)] = math.nan
        self.assertTrue(math.isnan(obstacle_distance(ranges, angles, 2.)))

    def test_rear_only_scan_does_not_cover_forward_motion(self):
        angles = np.linspace(2., 4., 100)
        self.assertTrue(math.isnan(obstacle_distance(np.full(100, math.inf), angles)))

    def test_valid_no_returns_remain_clear(self):
        angles = np.linspace(-math.pi, math.pi, 720)
        self.assertEqual(obstacle_distance(np.full(720, math.inf), angles), math.inf)


class ManualReverseTest(unittest.TestCase):
    def test_reverse_before_line_revokes_crossing(self):
        r=route();line=r.project(3.11,3.19)[0]
        d=Driver(r,[{'id':'test','line_s':line,'stop_s':line-.1382}])
        d.progress=line-.0982
        p=(*r.target(d.progress),math.pi/2)
        for i in range(3):d.step(p,Observation(light='green',light_id='test',frame=i),i*.05)
        self.assertTrue(d.committed)
        back=(*r.target(line-.20),math.pi/2)
        d.step(back,Observation(),.2,enabled=False)
        self.assertFalse(d.committed)
        v,_=d.step(back,Observation(light='red',light_id='test',frame=4),.25)
        self.assertLess(v,.05)
        self.assertNotEqual(d.state,'CROSSING')


class SweptBodyTest(unittest.TestCase):
    def test_turn_detects_rear_side_swing(self):
        angles=np.linspace(-math.pi,math.pi,720)
        ranges=np.full(720,math.inf)
        x,y=.000305,-.080284
        i=np.argmin(abs(angles-math.atan2(y,x-.075)))
        ranges[i]=math.hypot(x-.075,y)
        self.assertLess(obstacle_distance(ranges,angles,2.),.08)

    def test_clear_sweep_remains_clear(self):
        angles=np.linspace(-math.pi,math.pi,720)
        self.assertEqual(obstacle_distance(np.full(720,math.inf),angles,2.),math.inf)


class LongManualReverseTest(unittest.TestCase):
    def test_reverse_outside_progress_window_cannot_cross_on_red(self):
        r=route();line=r.project(3.11,3.19)[0]
        d=Driver(r,[{'id':'test','line_s':line,'stop_s':line-.1382}])
        d.progress=line+.1
        for i in range(3):d.step((*r.target(d.progress),math.pi/2),Observation(light='green',light_id='test',frame=i),i*.05)
        self.assertTrue(d.committed)
        back=(*r.target(line-.2),math.pi/2)
        d.step(back,Observation(),.2,enabled=False)
        self.assertFalse(d.committed)
        self.assertEqual(d.step(back,Observation(light='red',light_id='test',frame=4),.25),(0.,0.))
