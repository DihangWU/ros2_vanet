"""Poll Gazebo existence using wall time, including when simulation time is paused."""
import json
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from gz.transport13 import Node as TransportNode
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene
from .preview_scene import world_vehicles
from .vehicle_lifecycle import RemovalTracker, SCENE_QOS


class GazeboScene(Node):
    def __init__(self):
        super().__init__('gazebo_scene')
        self.declare_parameter('project_root', '')
        self.declare_parameter('scenario', 'traffic')
        expected = [v['id'] for v in world_vehicles(self.get_parameter('project_root').value,
                                                   self.get_parameter('scenario').value)]
        self.tracker = RemovalTracker(expected)
        self.transport = TransportNode()
        self.publisher = self.create_publisher(String, '/gazebo/vehicle_lifecycle', SCENE_QOS)
        self.timer = self.create_timer(.25, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def poll(self):
        success, scene = self.transport.request('/world/cosim_demo/scene/info', Empty(), Empty, Scene, 150)
        if not success or not any(m.name == 'road' for m in scene.model):
            return  # No scene is not evidence that every vehicle was deleted.
        before = set(self.tracker.deleted)
        self.tracker.observe(m.name for m in scene.model)
        for vehicle in self.tracker.deleted-before:
            self.get_logger().info(f'Gazebo vehicle removed: {vehicle}')
        self.publisher.publish(String(data=json.dumps({'deleted': sorted(self.tracker.deleted)})))


def main(args=None):
    rclpy.init(args=args)
    node = GazeboScene()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
