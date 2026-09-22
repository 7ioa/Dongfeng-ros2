#!/usr/bin/env python3
"""Terminal teleoperation; release detection uses keyboard-repeat expiry."""
import os
import select
import signal
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist

from command_logic import KeyboardCommands, TerminalKeys, manual_publish_needed

HELP = '''
东风沙盘小车 · 键盘控制（请保持本终端焦点，英文输入法）
  W / I：前进       S / ,：后退
  A / J：原地左转   D / L：原地右转
  U / O：前进左转 / 前进右转
  空格 / K：停车    + / -：调速    Q / Ctrl+C：退出
长按移动键持续行驶；松键后约 0.65 秒发出停车指令。
终端没有按键释放事件，停车请优先按空格。方向键不支持。
'''


def main():
    if not sys.stdin.isatty():
        raise SystemExit('请在 Ubuntu 的交互终端运行 bash keyboard_control.sh')
    rclpy.init()
    node = Node('dongfeng_keyboard')
    control = KeyboardCommands(
        float(node.declare_parameter('speed', 0.10).value),
        float(node.declare_parameter('turn', 0.65).value),
        float(node.declare_parameter('key_timeout', 0.65).value))
    pub = node.create_publisher(Twist, '/cmd_vel_manual', 1)
    decoder = TerminalKeys()
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    running = True

    def stop_signal(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop_signal)
    signal.signal(signal.SIGHUP, stop_signal)

    def publish(v=0.0, w=0.0):
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        pub.publish(msg)

    try:
        print(HELP, flush=True)
        # Wait for the guard; never queue a movement before control is connected.
        deadline = time.monotonic() + 20
        while rclpy.ok() and running and pub.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError('未连接到控制节点。请先在另一个终端运行 bash launch_car.sh')
            rclpy.spin_once(node, timeout_sec=0.1)
        if not running or not rclpy.ok():
            return
        termios.tcflush(fd, termios.TCIFLUSH)
        tty.setcbreak(fd)
        last_status = 0.0
        previous_velocity = (0.0, 0.0)
        while running and rclpy.ok():
            key_received = False
            if select.select([sys.stdin], [], [], 0.05)[0]:
                raw = os.read(fd, 64)
                if not raw:
                    break
                for key in decoder.feed(raw):
                    key_received = True
                    if not control.key(key, time.monotonic()):
                        running = False
                        break
            rclpy.spin_once(node, timeout_sec=0)
            now = time.monotonic()
            velocity = control.sample(now) if running else (0.0, 0.0)
            if manual_publish_needed(velocity, previous_velocity, key_received):
                publish(*velocity)
            previous_velocity = velocity
            if now - last_status > 0.25:
                print('\r线速度 %+.2f m/s  转速 %+.2f rad/s  档位 %.2f m/s   ' % (*velocity, control.speed), end='', flush=True)
                last_status = now
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)
        if rclpy.ok():
            for _ in range(3):
                publish()
                rclpy.spin_once(node, timeout_sec=0.03)
        print('\n键盘控制已结束。', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
