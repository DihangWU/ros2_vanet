"""Check physical mounting continuity and shared-model RViz placement."""
import math
from pathlib import Path
import unittest
import numpy as np
from builtin_interfaces.msg import Time
from cosim_bridge.sensor_visuals import rig_visuals, sensor_markers

ROOT = Path(__file__).resolve().parents[2]


class SensorVisualTests(unittest.TestCase):
    def test_every_component_is_connected_to_roof(self):
        parts = rig_visuals(str(ROOT), 'car_b')
        self.assertEqual(len(parts), 9)
        bounds = [(np.array(pose)-np.array(size)/2, np.array(pose)+np.array(size)/2)
                  for pose, _, size, _ in parts]
        # The first part is the mounting plate; roof front=0.95, top=1.55.
        self.assertAlmostEqual(bounds[0][0][2], 1.55)
        self.assertLessEqual(bounds[0][1][0], .95)
        connected = {0}
        while True:
            added = {i for i, (lo, hi) in enumerate(bounds) if any(
                np.all(lo <= bounds[j][1]+1e-9) and np.all(hi >= bounds[j][0]-1e-9)
                for j in connected)}
            if added <= connected:
                break
            connected |= added
        self.assertEqual(len(connected), len(parts), 'A housing, support or lens is floating')

    def test_markers_follow_vehicle_and_only_equipped_cars(self):
        self.assertEqual(sensor_markers('car_a', Time(), 10., 20., 0., ROOT), [])
        parts = rig_visuals(str(ROOT), 'car_b')
        markers = sensor_markers('car_b', Time(), 10., 20., math.pi/2, ROOT)
        for marker, (offset, shape, size, _) in zip(markers, parts):
            self.assertEqual(marker.ns, 'car_b/sensors')
            self.assertEqual(marker.type, shape)
            self.assertAlmostEqual(marker.pose.position.x, 10.-offset[1])
            self.assertAlmostEqual(marker.pose.position.y, 20.+offset[0])
            self.assertAlmostEqual(marker.pose.position.z, offset[2])
            self.assertEqual((marker.scale.x, marker.scale.y, marker.scale.z), size)


if __name__ == '__main__':
    unittest.main()
