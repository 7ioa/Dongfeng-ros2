"""These fixtures describe truth motion, never controller-reported progress."""
import math
import unittest
import numpy as np
from dongfeng_autonomy.route import Route
from dongfeng_autonomy.evaluation import evaluate_lap


class LapEvaluationTest(unittest.TestCase):
    def setUp(self):
        self.route = Route.perimeter(3.3, 5.4, .04, .8, .3)

    def sample(self, s, t):
        x,y = self.route.target(s)
        a,b = self.route.target(s-.001),self.route.target(s+.001)
        return dict(t=t,x=float(x),y=float(y),yaw=math.atan2(b[1]-a[1],b[0]-a[0]))

    def lap(self):
        samples=[self.sample(s,i*.1) for i,s in enumerate(np.linspace(0,self.route.length,1501))]
        samples += [self.sample(0,150+i*.1) for i in range(1,26)]
        return samples

    def test_spawn_only_false_complete_fails(self):
        result=evaluate_lap([self.sample(0,i*.1) for i in range(40)],True,self.route)
        self.assertFalse(result['lap_pass'])

    def test_ordered_full_lap_and_two_second_stop_passes(self):
        result=evaluate_lap(self.lap(),True,self.route)
        self.assertTrue(result['lap_pass'],result)
        self.assertEqual(result['checkpoints'],['B','C','D','A'])

    def test_controller_incomplete_cannot_pass(self):
        self.assertFalse(evaluate_lap(self.lap(),False,self.route)['lap_pass'])

    def test_teleporting_between_corners_fails(self):
        trace=[self.sample(s,i*.1) for i,s in enumerate(np.linspace(0,self.route.length,9))]
        trace += [self.sample(0,1+i*.1) for i in range(30)]
        self.assertFalse(evaluate_lap(trace,True,self.route)['lap_pass'])

    def test_reversed_lap_fails(self):
        trace=[self.sample(-s,i*.1) for i,s in enumerate(np.linspace(0,self.route.length,1501))]
        trace += [self.sample(0,150+i*.1) for i in range(1,26)]
        self.assertFalse(evaluate_lap(trace,True,self.route)['lap_pass'])

    def test_must_stop_for_two_seconds(self):
        self.assertFalse(evaluate_lap(self.lap()[:1501],True,self.route)['lap_pass'])

    def test_wrong_return_heading_fails(self):
        trace=self.lap()
        for sample in trace[-25:]:sample['yaw']=math.pi
        self.assertFalse(evaluate_lap(trace,True,self.route)['lap_pass'])

    def test_data_gaps_and_clock_rewind_fail(self):
        for dt in (-1.,3.):
            trace=self.lap();trace[700]['t']+=dt
            self.assertFalse(evaluate_lap(trace,True,self.route)['lap_pass'])

    def test_corner_of_footprint_off_road_fails(self):
        trace=self.lap()
        trace[20]['y']+=.10
        self.assertFalse(evaluate_lap(trace,True,self.route)['lap_pass'])


class SignalEvaluationTest(unittest.TestCase):
    def fixture(self):
        r=Route.perimeter(3.3,5.4,.04,.8,.3)
        signal={'id':'signal_7','line_s':r.project(3.11,3.19)[0]}
        trace=[]
        for i in range(71):
            t=i*.1
            y=2.95+.065*min(t,1)+.08*max(0,t-3)
            trace.append(dict(t=t,x=3.11,y=y,yaw=math.pi/2))
        lamps=[dict(t=i*.1,colors={'signal_7':'red' if i<30 else 'green'}) for i in range(71)]
        return r,signal,trace,lamps

    def test_red_stop_and_green_crossing_pass(self):
        from dongfeng_autonomy.evaluation import evaluate_signals
        r,s,trace,lamps=self.fixture()
        result=evaluate_signals(trace,lamps,[s],r)
        self.assertTrue(result['traffic_pass'],result)

    def test_red_crossing_fails(self):
        from dongfeng_autonomy.evaluation import evaluate_signals
        r,s,trace,lamps=self.fixture()
        lamps=lamps[:1]
        self.assertFalse(evaluate_signals(trace,lamps,[s],r)['traffic_pass'])

    def test_stale_green_at_crossing_is_unknown(self):
        from dongfeng_autonomy.evaluation import evaluate_signals
        r,s,trace,lamps=self.fixture()
        result=evaluate_signals(trace,[m for m in lamps if m['t']<=3.],[s],r)
        self.assertEqual(result['signal_crossings'][0]['color'],'unknown')
        self.assertFalse(result['traffic_pass'])

    def test_no_lamp_truth_cannot_claim_traffic_pass(self):
        from dongfeng_autonomy.evaluation import evaluate_signals
        r,s,trace,lamps=self.fixture()
        self.assertFalse(evaluate_signals(trace,[],[s],r)['traffic_pass'])

    def test_crossing_green_without_red_stop_is_incomplete_acceptance(self):
        from dongfeng_autonomy.evaluation import evaluate_signals
        r,s,trace,lamps=self.fixture()
        lamps=[dict(t=0.,colors={'signal_7':'green'})]
        self.assertFalse(evaluate_signals(trace,lamps,[s],r)['traffic_pass'])


class CurvedFootprintTest(unittest.TestCase):
    def test_inner_side_midpoint_must_stay_on_road(self):
        fixture=LapEvaluationTest();fixture.setUp()
        r=fixture.route;trace=fixture.lap();gate=r.project(2.46+.65/math.sqrt(2),.84-.65/math.sqrt(2))[0]
        for i,p in enumerate(trace[:1501]):
            progress=i*r.length/1500
            offset=.079*math.exp(-((progress-gate)/.22)**2)
            p['x']-=offset*math.sin(p['yaw']);p['y']+=offset*math.cos(p['yaw'])
        self.assertFalse(evaluate_lap(trace,True,r)['road_containment_pass'])


class GuiSampleTimeTest(unittest.TestCase):
    def test_late_green_is_rejected_without_restamping(self):
        from dongfeng_autonomy.evaluation import gui_signal_sample
        sample={'sim_time':3.,'wall_time':100.,'bulbs':{'signal_7_green':[0.,1.,.03]}}
        self.assertIsNone(gui_signal_sample(sample,100.9))

    def test_fresh_sample_keeps_render_source_time(self):
        from dongfeng_autonomy.evaluation import gui_signal_sample
        sample={'sim_time':3.,'wall_time':100.,'bulbs':{'signal_7_green':[0.,1.,.03]}}
        result=gui_signal_sample(sample,100.1)
        self.assertEqual(result['t'],3.)
        self.assertEqual(result['colors']['signal_7'],'green')
