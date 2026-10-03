"""Keep autonomous spawning aligned with the actual mission geometry."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext


PACKAGE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('autonomy_launch',PACKAGE/'launch/autonomy.launch.py')
autonomy_launch=importlib.util.module_from_spec(spec)
spec.loader.exec_module(autonomy_launch)


class MissionSpawnTest(unittest.TestCase):
    def simulation_arguments(self,mission):
        setup=getattr(autonomy_launch,'_simulation_launch',None)
        self.assertTrue(callable(setup),'Spawn must be selected from the mission at launch time')
        context=LaunchContext()
        context.launch_configurations.update(dict(
            mission=mission,gui_config='',headless='true',headless_rendering='false',
            render_engine='ogre',sensor_render_engine='ogre2',
            sensor_software_rendering='true',driving_mode='fast'))
        with patch.object(autonomy_launch,'get_package_share_directory',
                          side_effect=lambda name:str(PACKAGE.parent/name)):
            include=setup(context)[0]
        return dict(include.launch_arguments)

    def test_full_demo_spawns_inside_parking_at_the_mission_start(self):
        args=self.simulation_arguments('full_demo')
        self.assertAlmostEqual(float(args['x']),1.65)
        self.assertAlmostEqual(float(args['y']),4.815)
        self.assertAlmostEqual(float(args['z']),.033)
        self.assertLess(abs(float(args['yaw'])),.01)
        self.assertEqual(args['auto_mode'],'true')

    def test_perimeter_keeps_the_simulations_original_spawn(self):
        args=self.simulation_arguments('perimeter')
        self.assertTrue({'x','y','z','yaw'}.isdisjoint(args))
        self.assertEqual(args['auto_mode'],'true')
