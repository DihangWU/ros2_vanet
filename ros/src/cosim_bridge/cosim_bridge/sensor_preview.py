"""Static ROS vehicle display for Gazebo sensor inspection; no traffic/control backend."""
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from .preview_scene import world_scene, world_vehicles
from .car_visuals import car_markers
from .sensor_visuals import sensor_markers


class SensorPreview(Node):
    def __init__(self):
        super().__init__('sensor_preview')
        self.declare_parameter('project_root', '')
        self.declare_parameter('scenario', 'traffic')
        self.root = self.get_parameter('project_root').value
        scenario = self.get_parameter('scenario').value
        self.vehicles = world_vehicles(self.root, scenario)
        self.road = world_scene(self.root, scenario).find('world/model[@name="road"]')
        self.publisher = self.create_publisher(MarkerArray, '/demo/markers', 10)
        self.tf = StaticTransformBroadcaster(self)
        transforms = []
        for vehicle in self.vehicles:
            x, y, z, _, _, yaw = vehicle['pose']
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = vehicle['id']+'/base_link'
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation.z = math.sin(yaw/2)
            transform.transform.rotation.w = math.cos(yaw/2)
            transforms.append(transform)
        self.tf.sendTransform(transforms)
        self.timer = self.create_timer(.1, self.publish_scene)
        self.publish_scene()

    def publish_scene(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for vehicle in self.vehicles:
            x, y, _, _, _, yaw = vehicle['pose']
            markers.markers.extend(car_markers(vehicle['id'], stamp, x, y, yaw))
            markers.markers.extend(sensor_markers(vehicle['id'], stamp, x, y, yaw, self.root))
        # Read the actual asphalt, grass and lane markings; no duplicated road constants.
        for index, visual in enumerate(self.road.findall('.//visual')):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = 'map', stamp
            marker.ns, marker.id = 'preview/road', index
            marker.type, marker.action = Marker.CUBE, Marker.ADD
            x, y, z, roll, pitch, yaw = map(float, visual.findtext('pose').split())
            if abs(roll)+abs(pitch)>1e-9:
                raise ValueError('Preview road visuals must be level')
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
            marker.pose.orientation.z, marker.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            marker.scale.x, marker.scale.y, marker.scale.z = map(float, visual.findtext('geometry/box/size').split())
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = map(float, visual.findtext('material/diffuse').split())
            markers.markers.append(marker)
        self.publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SensorPreview()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
