"""Check side radar angles, wheel clearance and reusable mounting geometry."""
from pathlib import Path
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from builtin_interfaces.msg import Time
from cosim_bridge.corner_radar_display import radar_specs
from cosim_bridge.sensor_visuals import sensor_markers

ROOT = Path(__file__).resolve().parents[2]


class SideRadarTests(unittest.TestCase):
    def test_angles_clearance_and_close_side_coverage(self):
        rig = ET.parse(ROOT/'gazebo/models/side_radar_rig/model.sdf')
        car = ET.parse(ROOT/'gazebo/models/car_b/model.sdf')
        specs = radar_specs(ROOT, 'side_radar_rig')
        self.assertEqual(set(specs), {'side_left', 'side_right'})
        for name, sign in [('side_left', 1), ('side_right', -1)]:
            sensor = rig.find(f".//sensor[@name='{name}']")
            scan = sensor.find('lidar/scan/horizontal')
            yaw = specs[name]['pose'][5]
            limits = sorted(sign*math.degrees(yaw+float(scan.findtext(edge)))
                            for edge in ('min_angle', 'max_angle'))
            self.assertAlmostEqual(90-limits[0], 40, places=7)
            self.assertAlmostEqual(limits[1]-90, 80, places=7)
            visual = rig.find(f".//visual[@name='{name}_housing']")
            pose = list(map(float, visual.findtext('pose').split()))
            size = list(map(float, visual.findtext('geometry/box/size').split()))
            wheel = car.find(f".//visual[@name='wheel1.65_{sign}']")
            wheel_x = float(wheel.findtext('pose').split()[0])
            radius = float(wheel.findtext('geometry/cylinder/radius'))
            self.assertLess(pose[0]+size[0]/2, wheel_x-radius)
            self.assertLessEqual(abs(pose[1])-size[1]/2, .9)
            # Close side objects previously outside the four-corner views.
            for x in (-1., 0., 1.):
                dx, dy = x-specs[name]['pose'][0], sign*1.4-specs[name]['pose'][1]
                bearing = math.atan2(math.sin(math.atan2(dy, dx)-yaw), math.cos(math.atan2(dy, dx)-yaw))
                self.assertTrue(float(scan.findtext('min_angle')) <= bearing <= float(scan.findtext('max_angle')))

    def test_single_include_generates_two_independent_markers(self):
        with tempfile.TemporaryDirectory(prefix='cosim-side-radar-') as directory:
            root = Path(directory)
            rig = root/'gazebo/models/side_radar_rig'
            car = root/'gazebo/models/test_car'
            rig.mkdir(parents=True); car.mkdir(parents=True)
            (rig/'model.sdf').write_text((ROOT/'gazebo/models/side_radar_rig/model.sdf').read_text())
            (car/'model.sdf').write_text('<sdf version="1.9"><model name="test_car"><include merge="true"><uri>model://side_radar_rig</uri></include></model></sdf>')
            markers = sensor_markers('test_car', Time(), 10., 20., math.pi/2, root)
            self.assertEqual(len(markers), 2)
            self.assertTrue(all(m.ns == 'test_car/side_radars' for m in markers))
            self.assertAlmostEqual(markers[0].pose.position.x, 10.-.88)
            self.assertAlmostEqual(markers[0].pose.position.y, 20.+1.05)


if __name__ == '__main__':
    unittest.main()
