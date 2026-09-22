"""Manual commands latch automatic control off until explicitly enabled."""
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from .control import CommandMux


class Arbiter(Node):
    def __init__(self):
        super().__init__('command_arbiter')
        self.mux=CommandMux();self.mux.enable(self.declare_parameter('start_enabled',True).value)
        self.auto=(0.,0.);self.stamp=-float('inf')
        self.pub=self.create_publisher(Twist,'/cmd_vel',1)
        self.mode_pub=self.create_publisher(Bool,'/autonomy/enabled',1)
        self.create_subscription(Twist,'/cmd_vel_auto',self.automatic,1)
        self.create_subscription(Twist,'/cmd_vel_manual',self.manual,1)
        self.create_service(SetBool,'/autonomy/enable',self.enable)
        self.create_timer(.02,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def automatic(self,m):self.auto=(m.linear.x,m.angular.z);self.stamp=time.monotonic()
    def manual(self,m):self.mux.manual((m.linear.x,m.angular.z),time.monotonic());self.tick()
    def enable(self,req,res):
        self.mux.enable(req.data);self.stamp=-float('inf');self.tick()
        res.success=True;res.message='Automatic enabled' if req.data else 'Stopped; automatic disabled'
        return res
    def tick(self):
        m=Twist();m.linear.x,m.angular.z=self.mux.sample(time.monotonic(),self.auto,self.stamp);self.pub.publish(m)
        self.mode_pub.publish(Bool(data=self.mux.enabled))


def main():
    rclpy.init();n=Arbiter()
    try:rclpy.spin(n)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        if rclpy.ok():n.pub.publish(Twist())
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
