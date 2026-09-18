#!/usr/bin/env python3
"""Bridge-facing command limiter and timeout, including while Gazebo is paused."""
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist

from command_logic import CommandLease


class CommandGuard(Node):
    def __init__(self):
        super().__init__('command_guard')
        timeout = float(self.declare_parameter('command_timeout', 0.4).value)
        self.lease = CommandLease(timeout)
        self.pub = self.create_publisher(Twist, '/cmd_vel_safe', 1)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.receive, 1)
        # A ROS-time timer would stop on pause and retain a stale command.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.02, self.publish, clock=self.wall_clock)
        self.get_logger().info('Command guard: /cmd_vel -> /cmd_vel_safe; timeout %.2f s' % timeout)

    def receive(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            self.lease.stop()
        else:
            self.lease.update(msg.linear.x, msg.angular.z, time.monotonic())
        self.publish()

    def publish(self):
        msg = Twist()
        msg.linear.x, msg.angular.z = self.lease.sample(time.monotonic())
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CommandGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.lease.stop()
            node.publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
