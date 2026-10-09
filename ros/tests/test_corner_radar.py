"""Check side/rear coverage, near-field gaps and independent model reuse."""
from pathlib import Path
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from builtin_interfaces.msg import Time
from cosim_bridge.corner_radar_display import radar_specs
from cosim_bridge.sensor_visuals import sensor_markers

ROOT = Path(__file__).resolve().parents[2]


class CornerRadarTests(unittest.TestCase):
    def test_side_and_rear_targets_are_in_at_least_one_scan(self):
        rig = ET.parse(ROOT/'gazebo/models/corner_radar_rig/model.sdf')
        specs = radar_specs(ROOT)
        self.assertEqual(set(specs), {'front_left', 'front_right', 'rear_left', 'rear_right'})
        # Diagonal front mounts create front overlap but retain side near-field gaps.
        targets = [(x, y, .65) for x in np.linspace(-2.5, 2.5, 11) for y in (-12., 12.)]
        targets += [(-x, y, .65) for x in (12., 25.) for y in (-5., 0., 5.)]
        front_overlap = [(x, 0., .65) for x in (6.3, 10., 25.)]
        gaps = [(0., 1.4, .65), (0., -1.4, .65), (0., 8., .65),
                (0., -8., .65), (-4., 0., .65), (6., 0., .65)]
        for target in targets + front_overlap + gaps:
            covered = []
            for name, spec in specs.items():
                x, y, z, _, _, yaw = spec['pose']
                dx, dy, dz = np.array(target)-np.array([x, y, z])
                distance = math.sqrt(dx*dx+dy*dy+dz*dz)
                bearing = math.atan2(math.sin(math.atan2(dy, dx)-yaw), math.cos(math.atan2(dy, dx)-yaw))
                sensor = rig.find(f".//sensor[@name='{name}']")
                scan = sensor.find('lidar/scan/horizontal')
                if (.3 <= distance <= 30 and float(scan.findtext('min_angle')) <= bearing <= float(scan.findtext('max_angle'))):
                    covered.append(name)
            if target in gaps:
                self.assertFalse(covered, f'Expected near-field gap at {target}')
            else:
                self.assertTrue(covered, f'Uncovered target {target}')
            if target in front_overlap:
                self.assertTrue({'front_left', 'front_right'} <= set(covered),
                                f'Missing two-front-radar overlap at {target}')

    def test_one_line_include_on_custom_vehicle(self):
        with tempfile.TemporaryDirectory(prefix='cosim-radar-') as directory:
            root = Path(directory)
            rig = root/'gazebo/models/corner_radar_rig'
            car = root/'gazebo/models/test_car'
            rig.mkdir(parents=True)
            car.mkdir(parents=True)
            (rig/'model.sdf').write_text((ROOT/'gazebo/models/corner_radar_rig/model.sdf').read_text())
            (car/'model.sdf').write_text('<sdf version="1.9"><model name="test_car"><include merge="true"><uri>model://corner_radar_rig</uri></include></model></sdf>')
            markers = sensor_markers('test_car', Time(), 10., 20., math.pi/2, root)
            self.assertEqual(len(markers), 4)
            self.assertTrue(all(m.ns == 'test_car/radars' for m in markers))
            self.assertAlmostEqual(markers[0].pose.position.x, 10.-.9)
            self.assertAlmostEqual(markers[0].pose.position.y, 20.+2.45)


if __name__ == '__main__':
    unittest.main()
