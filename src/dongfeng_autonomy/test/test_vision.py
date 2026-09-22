import unittest
import cv2
import numpy as np
from dongfeng_autonomy.vision import detect_light, detect_lane


class VisionTest(unittest.TestCase):
    def test_light_colors_and_black_frame(self):
        for color,bgr in [('red',(0,0,255)),('yellow',(0,255,255)),('green',(0,255,0))]:
            frame=np.full((480,640,3),200,np.uint8)
            cv2.rectangle(frame,(270,100),(370,133),(20,20,20),-1)
            cv2.circle(frame,(320,115),8,bgr,-1)
            self.assertEqual(detect_light(frame,(260,90,120,60))[0],color)
        self.assertEqual(detect_light(np.zeros((480,640,3),np.uint8))[0],'unknown')

    def test_background_green_and_wrong_roi(self):
        frame=np.zeros((480,640,3),np.uint8)
        cv2.rectangle(frame,(0,0),(640,200),(0,180,0),-1)
        self.assertEqual(detect_light(frame)[0],'unknown')
        cv2.circle(frame,(500,300),8,(0,0,255),-1)
        self.assertEqual(detect_light(frame,(0,0,100,100))[0],'unknown')

    def test_blank_has_no_lane(self):
        result=detect_lane(np.zeros((480,640,3),np.uint8))
        self.assertFalse(result[1])

    def test_white_lane_edges(self):
        frame=np.full((480,640,3),45,np.uint8)
        cv2.line(frame,(30,479),(290,260),(240,240,240),5)
        cv2.line(frame,(610,479),(350,260),(240,240,240),5)
        error,valid=detect_lane(frame)
        self.assertTrue(valid)
        self.assertLess(abs(error),.02)

class SurfaceTest(unittest.TestCase):
    def test_road_without_painted_edges(self):
        frame=np.full((480,640,3),200,np.uint8)
        cv2.fillPoly(frame,[np.array([[280,250],[360,250],[639,400],[0,400]])],(110,105,100))
        self.assertTrue(detect_lane(frame)[1])

class CurveSurfaceTest(unittest.TestCase):
    def test_curve_with_one_visible_boundary(self):
        frame=np.full((480,640,3),210,np.uint8)
        cv2.fillPoly(frame,[np.array([[0,240],[320,230],[639,310],[639,479],[0,479]])],(110,105,100))
        self.assertTrue(detect_lane(frame)[1])
        self.assertFalse(detect_lane(np.full_like(frame,110))[1])

class StopLineTest(unittest.TestCase):
    def test_projected_stop_line_and_missing_line(self):
        from dongfeng_autonomy.vision import detect_stop_line
        frame=np.full((480,640,3),80,np.uint8)
        cv2.line(frame,(130,300),(510,300),(240,240,240),5)
        self.assertAlmostEqual(detect_stop_line(frame,(254,254,320,240),0.,.156),.156,delta=.015)
        self.assertIsNone(detect_stop_line(np.zeros_like(frame),(254,254,320,240),0.,.156))
