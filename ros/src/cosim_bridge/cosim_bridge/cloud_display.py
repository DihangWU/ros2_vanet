"""Transform display-only clouds at their measurement time before publishing to RViz."""
import copy
from collections import deque
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from .vehicle_lifecycle import empty_cloud
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy


def obstacle_cloud(cloud):
    """Display-only height mask for the current flat map; retain all fields/stamps."""
    points = point_cloud2.read_points(cloud)
    return point_cloud2.create_cloud(copy.deepcopy(cloud.header), cloud.fields,
                                    points[points['z'] > .15], point_step=cloud.point_step)


class CloudDisplay:
    def __init__(self, node):
        self.buffer = Buffer(node=node)
        self.listener = TransformListener(self.buffer, node)
        self.pending = {}
        self.latest = {}
        self.show_ground = True
        self.ground_sub = node.create_subscription(Bool, '/display/show_ground_returns',
            self.set_ground_visibility, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = node.create_timer(.02, self.flush,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))

    def submit(self, publisher, cloud):
        # Bounded per-channel queues also cover startup and paused simulation.
        queue = self.pending.setdefault(publisher, deque(maxlen=20))
        queue.append(cloud)
        self.flush()

    def flush(self):
        for publisher, queue in self.pending.items():
            while queue:
                cloud = queue[0]
                try:
                    transform = self.buffer.lookup_transform(
                        'map', cloud.header.frame_id, Time.from_msg(cloud.header.stamp))
                except TransformException:
                    # Never substitute the latest TF or change the measurement stamp.
                    break
                result = do_transform_cloud(cloud, transform)
                result.header = copy.deepcopy(cloud.header)
                result.header.frame_id = 'map'
                queue.popleft()
                self.latest[publisher] = result
                publisher.publish(result if self.show_ground else obstacle_cloud(result))

    def set_ground_visibility(self, message):
        self.show_ground = message.data
        # The switch also works while the simulation is paused.
        for publisher, cloud in self.latest.items():
            publisher.publish(cloud if self.show_ground else obstacle_cloud(cloud))

    def clear(self, publisher, cloud):
        self.pending.pop(publisher, None)
        result = empty_cloud(cloud)
        result.header.frame_id = 'map'
        self.latest[publisher] = result
        publisher.publish(result)
