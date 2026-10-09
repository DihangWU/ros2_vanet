"""Confirmed Gazebo removals and explicit cleanup of persistent RViz markers."""
import json
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker

SCENE_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def deleted_ids(message):
    return set(json.loads(message.data)['deleted'])


class RemovalTracker:
    """Require prior presence and two successful missing snapshots; failures do not count."""
    def __init__(self, expected):
        self.expected = set(expected)
        self.seen, self.deleted, self.missing = set(), set(), {}

    def observe(self, present):
        present = set(present) & self.expected
        self.seen |= present
        for name in self.seen-self.deleted:
            self.missing[name] = 0 if name in present else self.missing.get(name, 0)+1
            if self.missing[name] >= 2:
                self.deleted.add(name)
        return self.deleted


class MarkerHistory:
    """A missing ADD does not erase a Marker in RViz; send DELETE for its exact key."""
    def __init__(self):
        self.keys = set()
        self.retired = set()

    def reconcile(self, array, stamp):
        supplied = {(m.ns, m.id) for m in array.markers}
        active = {(m.ns, m.id) for m in array.markers if m.action == Marker.ADD}
        self.retired |= self.keys-active
        self.retired -= active
        self.keys = active
        # Publishers may already include DELETE (for example an expired packet replay).
        # RViz rejects duplicate keys even when both entries request deletion.
        for namespace, identifier in sorted(self.retired-supplied):
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = 'map', stamp
            marker.ns, marker.id, marker.action = namespace, identifier, Marker.DELETE
            array.markers.append(marker)
        return array


def empty_cloud(message):
    """Keep the schema/frame but clear the last displayed point cloud."""
    import copy
    result = copy.deepcopy(message)
    result.height, result.width, result.row_step = 1, 0, 0
    result.data = b''
    return result


def black_image(width, height, frame, stamp):
    from sensor_msgs.msg import Image
    result = Image(width=width, height=height, encoding='rgb8', step=width*3,
                   data=bytes(width*height*3))
    result.header.frame_id, result.header.stamp = frame, stamp
    return result
