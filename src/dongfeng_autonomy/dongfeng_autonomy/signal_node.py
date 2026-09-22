"""Change actual Gazebo bulb materials. Driving never subscribes to this node."""
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from gz.transport13 import Node as GzNode
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.visual_pb2 import Visual
from gz.msgs10.boolean_pb2 import Boolean


class Signals(Node):
    def __init__(self):
        super().__init__('traffic_signals')
        root=Path(get_package_share_directory('dongfeng_autonomy'))
        self.config=json.loads((root/'config/signals.json').read_text())
        self.gz=GzNode();self.ids={};self.last={}
        self.offset=float(self.declare_parameter('phase_offset',0.).value)
        self.forced=str(self.declare_parameter('force_color','cycle').value)
        self.prefix='/world/'+self.declare_parameter('world_name','dongfeng_world').value
        self.state_pub=self.create_publisher(String,'/evaluation/signal_state',10)
        self.create_timer(.25,self.tick)

    def tick(self):
        if not self.ids:
            ok,scene=self.gz.request(self.prefix+'/scene/info',Empty(),Empty,Scene,500)
            if not ok:return
            for model in scene.model:
                for link in model.link:
                    for visual in link.visual:
                        if visual.name.startswith('signal_'):self.ids[visual.name]=visual.id
            if not self.ids:return
        self.forced=str(self.get_parameter('force_color').value)
        t=self.get_clock().now().nanoseconds/1e9+float(self.get_parameter('phase_offset').value)
        for signal in self.config:
            phase=(t+signal.get('phase',0))%26
            color='green' if phase<10 else ('yellow' if phase<12 else 'red')
            if self.forced in ('red','green','yellow'):color=self.forced
            if self.last.get(signal['id'])==color:continue
            success=True
            for lamp,rgb in [('red',(1.,0.,0.)),('yellow',(1.,.75,0.)),('green',(0.,1.,.03))]:
                name=signal['id']+'_'+lamp
                if name not in self.ids:success=False;continue
                msg=Visual();msg.id=self.ids[name];msg.name=name
                active=lamp==color
                for prop in ('ambient','diffuse','emissive'):
                    c=getattr(msg.material,prop)
                    c.r,c.g,c.b=rgb if active else (.025,.025,.025)
                    c.a=1.
                ok,response=self.gz.request(self.prefix+'/visual_config',msg,Visual,Boolean,200)
                success=success and ok and response.data
            self.last[signal['id']]=color if success else 'unknown'
        # Evaluation only: the driving node has no subscription to this topic.
        self.state_pub.publish(String(data=json.dumps(dict(t=self.get_clock().now().nanoseconds/1e9,colors=self.last))))


def main():
    rclpy.init();n=Signals()
    try:rclpy.spin(n)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
