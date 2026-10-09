"""Display clouds wait for exact-time TF; algorithm clouds and timestamps stay intact."""
import unittest
from collections import deque
from types import SimpleNamespace
import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import ExtrapolationException
from cosim_bridge.cloud_display import CloudDisplay


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(message)


class Buffer:
    ready = False
    def lookup_transform(self, target, source, stamp):
        if not self.ready: raise ExtrapolationException('TF has not arrived yet')
        self.query = (target, source, stamp.nanoseconds)
        result = TransformStamped()
        result.header.frame_id = target
        result.transform.translation.x = 10.
        # 90-degree yaw tests true coordinate rotation rather than relabeling.
        result.transform.rotation.z = result.transform.rotation.w = 2**-.5
        return result


class CloudDisplayTests(unittest.TestCase):
    def setup_display(self):
        display = object.__new__(CloudDisplay)
        display.pending, display.buffer = {}, Buffer()
        fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z'))]
        fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
        points = np.array([(1., 2., 3., 0x00FFFF)],
                          dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
        cloud = point_cloud2.create_cloud(Header(frame_id='car_b/radar/side_left_link',
                          stamp=Time(sec=8, nanosec=50000000)), fields, points)
        return display, Publisher(), cloud

    def test_wait_then_rotate_preserving_stamp_color_and_source(self):
        display, publisher, cloud = self.setup_display()
        data, frame = bytes(cloud.data), cloud.header.frame_id
        display.submit(publisher, cloud)
        self.assertEqual(publisher.messages, [])
        display.buffer.ready = True
        display.flush()
        result = publisher.messages[0]
        self.assertEqual(display.buffer.query, ('map', frame, 8050000000))
        self.assertEqual(result.header.frame_id, 'map')
        self.assertEqual(result.header.stamp, cloud.header.stamp)
        point = point_cloud2.read_points(result)[0]
        np.testing.assert_allclose([point[n] for n in ('x','y','z')], [8.,1.,3.], atol=1e-6)
        self.assertEqual(point['rgb'], 0x00FFFF)
        self.assertEqual(bytes(cloud.data), data)
        self.assertEqual(cloud.header.frame_id, frame)

    def test_deletion_cancels_queued_clouds_and_clears_map_display(self):
        display, publisher, cloud = self.setup_display()
        for _ in range(30): display.submit(publisher, cloud)
        self.assertEqual(len(display.pending[publisher]),20)
        display.clear(publisher, cloud)
        display.buffer.ready = True
        display.flush()
        self.assertEqual(len(publisher.messages),1)
        self.assertEqual(publisher.messages[0].width,0)
        self.assertEqual(publisher.messages[0].header.frame_id,'map')


if __name__ == '__main__': unittest.main()
