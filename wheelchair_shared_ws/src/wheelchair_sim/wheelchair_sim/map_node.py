"""Simulation oracle. No real robot sensor interpretation is claimed."""
import math
import subprocess
import uuid
import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon
from wheelchair_interfaces.msg import ObstacleMap
from .scenes import boundary, map_obstacles, MESH_SCENES

def polygon(points): return Polygon(points=[Point32(x=float(x),y=float(y),z=0.) for x,y in points])
class MapNode(Node):
    def __init__(self):
        super().__init__('simulation_obstacle_map')
        self.declare_parameter('scene','straight_wall')
        self.scene=self.get_parameter('scene').value
        self.boxes=map_obstacles(self.scene)
        self.boundary=boundary(self.scene)
        self.session=uuid.uuid4().hex
        self.sequence=0
        self.pub=self.create_publisher(ObstacleMap,'/wheelchair/obstacle_map',1)
        self.message=ObstacleMap()
        self.message.header.frame_id='odom';self.message.session_id=self.session
        self.message.complete=True;self.message.known_free_boundary=polygon(self.boundary)
        self.message.obstacles=[polygon(b.polygon()) for b in self.boxes]
        self.create_timer(.1 if self.scene in MESH_SCENES else .05,self.publish)
        self.start=None
        self.process=None
        if self.scene=='dynamic': self.create_timer(.25,self.move_obstacle)
    def publish(self):
        now=self.get_clock().now()
        if now.nanoseconds==0: return
        self.sequence+=1
        msg=self.message
        msg.header.stamp=now.to_msg();msg.sequence=self.sequence
        self.pub.publish(msg)
    def move_obstacle(self):
        now=self.get_clock().now().nanoseconds*1e-9
        if now<=0: return
        if self.start is None: self.start=now
        if self.process is not None and self.process.poll() is None: return
        y=2*math.cos((now-self.start)*.4)
        request=f'name: "moving_obstacle", position: {{x: 2, y: {y}, z: 0.75}}'
        self.process=subprocess.Popen(['ign','service','-s','/world/wheelchair_arena/set_pose','--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','500','--req',request],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    def destroy_node(self):
        if self.process is not None and self.process.poll() is None: self.process.terminate()
        super().destroy_node()
def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO); node=MapNode()
    try:
        while rclpy.ok():rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()
