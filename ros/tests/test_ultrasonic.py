"""Near-field mounting and valid independent range/color semantics."""
import math
from pathlib import Path
import unittest
import numpy as np
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from cosim_bridge.ultrasonic_display import ultrasonic_specs, range_message
from cosim_bridge.perception_display import colored_cloud

ROOT=Path(__file__).resolve().parents[2]
class UltrasonicTests(unittest.TestCase):
    def test_mounting_and_coverage(self):
        specs=ultrasonic_specs(ROOT)
        self.assertEqual(len(specs),16)
        for name,s in specs.items():
            x,y,z,r,p,yaw=s['pose']
            if name.startswith('side_'):
                self.assertAlmostEqual(abs(y),.94)
                self.assertIn(x, (2.25,.75,-.75,-2.25))
                # Rays begin outside the body. No housing overlaps the wheel x extent.
                self.assertTrue(all(abs(x-wheel_x)>.38+.03 for wheel_x in (-1.65,1.65)))
                self.assertGreater(abs(math.sin(yaw)),.96)
            else:
                self.assertAlmostEqual(abs(x),2.54)
                self.assertLess(abs(y),.9)
            self.assertEqual(z,.45)
            self.assertEqual((r,p),(0,0))
            self.assertAlmostEqual(math.degrees(s['fov']),100,places=7)
            self.assertEqual((s['minimum'],s['maximum']),(.15,5))
            # Forward/rear projection and side orientation follow the mounting location.
            if not name.startswith('side_'):
                self.assertEqual(math.cos(yaw)>0,name.startswith('front'))
            self.assertEqual(math.sin(yaw)>0,'left' in name)

    def test_side_strip_has_overlapping_horizontal_coverage(self):
        specs=ultrasonic_specs(ROOT)
        for side,sign in [('left',1),('right',-1)]:
            sensors=[s for n,s in specs.items() if n.startswith(f'side_{side}_')]
            # A near-side strip 0.85 m outside the optical origins, along the full body.
            for target_x in np.linspace(-2.5,2.5,101):
                covered=False
                for s in sensors:
                    x,y,z,_,_,yaw=s['pose']
                    dx,dy=target_x-x,sign*.85
                    delta=math.atan2(math.sin(math.atan2(dy,dx)-yaw),
                                     math.cos(math.atan2(dy,dx)-yaw))
                    covered |= abs(delta)<=s['fov']/2 and math.hypot(dx,dy)<=s['maximum']
                self.assertTrue(covered, (side,target_x))

    def test_nearest_range_and_distinct_colors(self):
        spec=ultrasonic_specs(ROOT)['front_left_inner']
        cloud=point_cloud2.create_cloud_xyz32(Header(frame_id='sensor'),
            [(1,0,0),(2,0,-.45),(float('inf'),0,0),(6,0,0),(.1,0,0)])
        message=range_message(cloud,spec)
        self.assertEqual(message.range,1)
        self.assertEqual(message.radiation_type,message.ULTRASOUND)
        self.assertEqual(message.header,cloud.header)
        rgb=point_cloud2.read_points(colored_cloud(cloud,.45,0xFFFF00,0xFFFFFF))['rgb']
        self.assertEqual(set(rgb),{0xFFFF00,0xFFFFFF})
        empty=point_cloud2.create_cloud_xyz32(Header(),[])
        self.assertTrue(math.isinf(range_message(empty,spec).range))

if __name__=='__main__':unittest.main()
