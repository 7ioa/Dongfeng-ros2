import unittest
import numpy as np
from dongfeng_autonomy.localization import LandmarkMap


class LocalizationTest(unittest.TestCase):
    def test_recovers_translation_from_two_walls(self):
        y=np.linspace(-1,1,100)
        points=np.vstack([np.c_[np.ones(100),y,np.zeros(100)],np.c_[y,np.ones(100),np.zeros(100)]])
        normals=np.vstack([np.tile([1.,0,0],(100,1)),np.tile([0.,1,0],(100,1))])
        m=LandmarkMap(points,normals)
        delta,good=m.correction(points+np.array([.05,-.03,0.]))
        self.assertTrue(good)
        np.testing.assert_allclose(delta,[-.05,.03],atol=.005)

    def test_empty_scan_and_far_outliers_rejected(self):
        m=LandmarkMap(np.array([[0.,0.,0.],[1.,0,0]]),np.array([[1.,0,0],[1.,0,0]]))
        self.assertFalse(m.correction(np.empty((0,3)))[1])
        self.assertFalse(m.correction(np.full((30,3),100.))[1])

    def test_real_a_bend_scan_recovers_without_runaway_translation(self):
        from pathlib import Path
        root=Path(__file__).resolve().parents[1]
        data=np.load(root/'config/landmarks.npz')
        m=LandmarkMap(data['points'],data['normals'])
        endpoints=np.load(Path(__file__).parent/'fixtures/a_bend_scan.npz')['endpoints'].copy()
        offset=np.array([0.,-.027])
        for _ in range(8):
            moved=endpoints.copy();moved[:,:2]+=offset
            delta,good=m.correction(moved)
            self.assertTrue(good)
            offset+=delta
        self.assertLess(np.linalg.norm(offset),.01)
