"""Transform display-only clouds at their measurement time before publishing to RViz."""
import copy
from collections import deque
from rclpy.clock import Clock, ClockType
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from .vehicle_lifecycle import empty_cloud


class CloudDisplay:
    def __init__(self, node):
        self.buffer = Buffer(node=node)
        self.listener = TransformListener(self.buffer, node)
        self.pending = {}
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
                publisher.publish(result)

    def clear(self, publisher, cloud):
        self.pending.pop(publisher, None)
        result = empty_cloud(cloud)
        result.header.frame_id = 'map'
        publisher.publish(result)
