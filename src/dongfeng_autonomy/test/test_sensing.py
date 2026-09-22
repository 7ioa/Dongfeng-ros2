import unittest
from dongfeng_autonomy.sensing import Freshness


class FreshnessTest(unittest.TestCase):
    def test_repeated_stamp_does_not_refresh_source(self):
        f=Freshness(('image',),.5)
        self.assertTrue(f.update('image',1.,10.))
        self.assertFalse(f.update('image',1.,10.4))
        self.assertFalse(f.ready(1.1,10.6))

    def test_future_old_and_missing_samples_stop(self):
        f=Freshness(('image','scan'),.5)
        f.update('image',1.,10.)
        self.assertFalse(f.ready(1.1,10.1))
        f.update('scan',1.,10.)
        self.assertTrue(f.ready(1.1,10.1))
        self.assertFalse(f.ready(2.,10.1))
        self.assertFalse(f.ready(.1,10.1))
        self.assertFalse(f.update('scan',float('nan'),10.2))


class OrientationTest(unittest.TestCase):
    def test_invalid_and_unavailable_orientation_rejected(self):
        from dongfeng_autonomy.sensing import orientation_rpy
        for q in ([0,0,0,0],[0,0,10,1],[float('nan'),0,0,1]):
            self.assertIsNone(orientation_rpy(q))
        self.assertIsNone(orientation_rpy([0,0,0,1],available=False))

    def test_normalizes_roundoff_and_preserves_heading(self):
        import math
        from dongfeng_autonomy.sensing import orientation_rpy
        r,p,y=orientation_rpy([0,0,.707107,.707107])
        self.assertAlmostEqual(r,0.);self.assertAlmostEqual(p,0.)
        self.assertAlmostEqual(y,math.pi/2)
