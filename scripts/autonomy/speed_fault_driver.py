#!/usr/bin/env python3
"""Test-only sensor fault injection; never used by normal launch files.

Gate the real driver's image callback without adding a second DDS image hop.
No truth or evaluator data is available to this driver.
"""
import rclpy
from dongfeng_autonomy.autonomy_node import Autonomy


class FaultProbeDriver(Autonomy):
    def __init__(self):
        super().__init__()
        self.declare_parameter('probe_image_gate','live')
        self.probe_last_image=None

    def image(self,message):
        gate=self.get_parameter('probe_image_gate').value
        if gate=='live':
            self.probe_last_image=message
            super().image(message)
        elif gate=='stale' and self.probe_last_image is not None:
            super().image(self.probe_last_image)


def main():
    rclpy.init();node=FaultProbeDriver()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
