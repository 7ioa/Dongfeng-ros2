"""Behavior regressions for keyboard control, independent of ROS or Gazebo."""
from pathlib import Path
import math
import sys
import unittest
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
from command_logic import CommandLease, KeyboardCommands, TerminalKeys


class KeyboardTest(unittest.TestCase):
    def test_forward_reverse_left_right_and_diagonals(self):
        controller = KeyboardCommands()
        for key, expected in [('w', (.1, 0)), ('s', (-.1, 0)),
                              ('a', (0, .65)), ('d', (0, -.65)),
                              ('u', (.1, .65)), ('o', (.1, -.65))]:
            controller.key(key, 10)
            self.assertEqual(controller.sample(10.01), expected)

    def test_release_timeout_does_not_revive(self):
        controller = KeyboardCommands()
        controller.key('w', 10)
        self.assertEqual(controller.sample(10.5), (.1, 0))
        self.assertEqual(controller.sample(10.7), (0, 0))
        self.assertEqual(controller.sample(10.1), (0, 0))
        controller.key('w', 11)
        self.assertEqual(controller.sample(11.1), (.1, 0))

    def test_space_quit_unknown_key_and_speed_change_stop(self):
        for key in (' ', 'k', 'q', '\x03', '?', '+', '-'):
            controller = KeyboardCommands()
            controller.key('w', 0)
            running = controller.key(key, .1)
            self.assertEqual(controller.sample(.1), (0, 0))
            self.assertEqual(running, key not in ('q', '\x03'))

    def test_speed_limits_cannot_be_exceeded(self):
        controller = KeyboardCommands()
        for _ in range(100):
            controller.key('+', 0)
        controller.key('u', 0)
        self.assertEqual(controller.sample(.1), (.25, 1.2))
        for _ in range(100):
            controller.key('-', 0)
        controller.key('o', 0)
        self.assertEqual(controller.sample(.1), (.02, -.2))

    def test_split_arrow_sequence_never_becomes_wasd(self):
        decoder = TerminalKeys()
        self.assertEqual(decoder.feed(b'\x1b'), [' '])
        self.assertEqual(decoder.feed(b'['), [])
        self.assertEqual(decoder.feed(b'A'), [])
        self.assertEqual(decoder.feed(b'\x1bODw'), [' ', 'w'])
        self.assertEqual(decoder.feed(b'\x1b[1;5D'), [' '])


class GuardTest(unittest.TestCase):
    def test_no_command_or_lost_publisher_means_zero(self):
        guard = CommandLease(.4)
        self.assertEqual(guard.sample(10), (0, 0))
        guard.update(.1, .3, 10)
        self.assertEqual(guard.sample(10.39), (.1, .3))
        self.assertEqual(guard.sample(10.41), (0, 0))

    def test_key_release_and_disconnection_whole_chain(self):
        keyboard, guard = KeyboardCommands(), CommandLease()
        keyboard.key('w', 100)
        for t in (100.05, 100.2, 100.4, 100.6):
            guard.update(*keyboard.sample(t), t)
            self.assertEqual(guard.sample(t), (.1, 0))
        guard.update(*keyboard.sample(100.7), 100.7)
        self.assertEqual(guard.sample(100.7), (0, 0))
        keyboard.key('w', 101)
        guard.update(*keyboard.sample(101), 101)
        self.assertEqual(guard.sample(101.41), (0, 0))

    def test_forward_reverse_and_turn_limits_symmetric(self):
        guard = CommandLease()
        for sign in (-1, 1):
            guard.update(sign * 100, sign * 100, 0)
            self.assertEqual(guard.sample(.1), (sign * .25, sign * 1.2))

    def test_nan_and_infinity_stop_existing_motion(self):
        for bad in (math.nan, math.inf, -math.inf):
            guard = CommandLease()
            guard.update(.1, 0, 0)
            self.assertFalse(guard.update(bad, 0, .1))
            self.assertEqual(guard.sample(.11), (0, 0))

    def test_time_rewind_invalidates_command(self):
        guard = CommandLease()
        guard.update(.1, 0, 10)
        self.assertEqual(guard.sample(9), (0, 0))


class IntegrationConfigTest(unittest.TestCase):
    def test_all_four_wheels_receive_correct_turn_direction(self):
        robot = ET.parse(PACKAGE.parent / 'dongfeng_description/urdf/dongfeng_car.urdf.xacro').getroot()
        drive = robot.find("gazebo/plugin[@name='gz::sim::systems::DiffDrive']")
        track, radius = float(drive.findtext('wheel_separation')), float(drive.findtext('wheel_radius'))
        joints = {j.get('name'): j for j in robot.findall('joint')}
        for side, sign in [('left', -1), ('right', 1)]:
            names = [e.text for e in drive.findall(side + '_joint')]
            self.assertEqual(len(names), 2)
            # Positive yaw: left wheels reverse; right wheels move forward.
            omega = sign * .65 * track / (2 * radius)
            self.assertEqual(math.copysign(1, omega), sign)
            for name in names:
                self.assertEqual(joints[name].find('axis').get('xyz'), '0 1 0')
                y = float(joints[name].find('origin').get('xyz').split()[1])
                self.assertAlmostEqual(y, -sign * track / 2)
        self.assertEqual(drive.findtext('topic'), '/cmd_vel')
        self.assertEqual(float(drive.findtext('min_linear_velocity')), -.25)
        self.assertEqual(float(drive.findtext('max_linear_velocity')), .25)
        self.assertEqual(float(drive.findtext('min_angular_acceleration')), -2.4)
        bridge = (PACKAGE / 'config/bridge.yaml').read_text(encoding='utf-8')
        self.assertIn('ros_topic_name: "/cmd_vel_safe"', bridge)
        self.assertNotIn('ros_topic_name: "/cmd_vel"', bridge)
        self.assertIn('gz_topic_name: "/cmd_vel"', bridge)


if __name__ == '__main__':
    unittest.main(verbosity=2)

class ManualStreamTest(unittest.TestCase):
    def test_idle_does_not_revoke_auto_but_stop_key_does(self):
        from command_logic import manual_publish_needed
        self.assertFalse(manual_publish_needed((0.,0.),(0.,0.),False))
        self.assertTrue(manual_publish_needed((0.,0.),(0.,0.),True))
        self.assertTrue(manual_publish_needed((.1,0.),(.1,0.),False))
        self.assertTrue(manual_publish_needed((0.,0.),(.1,0.),False))
