"""Deterministic pure-algorithm tests, runnable without ROS2/Gazebo/SUMO."""
import unittest
import numpy as np
from Algorithm.LidarBrake import Config, LidarBrake
from Algorithm.LidarBrake.front_obstacle import front_gap
from Algorithm.LidarBrake.distance_tracker import DistanceTracker


def rear_surface(gap):
    return np.array([[gap+2.5, y, z] for y in np.linspace(-.8,.8,13)
                     for z in np.linspace(.35,1.4,10)])


class LidarBrakeTests(unittest.TestCase):
    def test_ground_self_and_adjacent_lane_are_not_front_target(self):
        cfg = Config()
        target = rear_surface(12.)
        ground = np.array([[3.,0.,0.], [8.,0.,.01]])
        self_points = np.tile([1.,0.,1.], (20,1))
        adjacent = rear_surface(3.)+np.array([0.,3.2,0.])
        noise = np.array([[4.,0.,1.], [np.nan,0.,1.]])
        self.assertAlmostEqual(front_gap(np.vstack([target,ground,self_points,adjacent,noise]),cfg,2.5),12.)
        self.assertIsNone(front_gap(np.vstack([ground,self_points,adjacent,noise]),cfg,2.5))

    def test_tracker_closing_duplicate_and_reacquisition(self):
        tracker = DistanceTracker(Config())
        tracker.update(20.,1.)
        value = tracker.update(19.,1.1)
        self.assertLess(value.relative_speed,0.)
        self.assertEqual(tracker.update(100.,1.0),value)
        self.assertIsNone(tracker.update(None,1.2))
        self.assertEqual(tracker.update(50.,1.3).relative_speed,0.)

    def test_stale_missing_cloud_never_accelerates(self):
        algorithm = LidarBrake(Config(),2.5)
        algorithm.observe(rear_surface(40.),0.)
        decision = algorithm.step(10.,.5)
        self.assertEqual(decision.state,'SENSOR_LOST_BRAKE')
        self.assertLess(decision.target_speed,10.)
        algorithm.observe(np.empty((0,3)),.6)
        self.assertEqual(algorithm.step(10.,.6).state,'SENSOR_LOST_BRAKE')

    def test_warning_and_stop_hold(self):
        algorithm = LidarBrake(Config(),2.5)
        algorithm.observe(rear_surface(40.),0.)
        algorithm.warn(0.)
        self.assertLess(algorithm.step(15.,.05).acceleration,0.)
        parked = LidarBrake(Config(),2.5)
        parked.observe(rear_surface(2.53),0.)
        self.assertEqual(parked.step(.05,0.).target_speed,0.)
        parked.observe(rear_surface(20.),.1)
        self.assertEqual(parked.step(0.,.1).state,'STOPPED_HOLD')

    def test_emergency_guard_preempts_comfort_jerk(self):
        algorithm = LidarBrake(Config(max_jerk=.1), 2.5)
        algorithm.observe(rear_surface(20.), 0.)
        algorithm.step(15., 0.)
        algorithm.observe(rear_surface(6.), .1)
        decision = algorithm.step(15., .1)
        self.assertEqual(decision.state, 'EMERGENCY_BRAKE')
        self.assertEqual(decision.acceleration, -8.)

    def test_two_car_braking_from_lidar_samples_only(self):
        algorithm = LidarBrake(Config(),2.5)
        a,b,va,vb = 100.,55.,15.,15.
        minimum = float('inf')
        for step in range(600):
            now = step*.05
            gap = a-5.-b
            minimum = min(minimum,gap)
            if step%2 == 0:
                algorithm.observe(rear_surface(gap),now)
            if step == 101:
                algorithm.warn(now)
            decision = algorithm.step(vb,now)
            vb = decision.target_speed
            if now >= 5.:
                va = max(0.,va-7.5*.05)
            a += va*.05
            b += vb*.05
        self.assertGreater(minimum,2.3)
        self.assertLess(a-5.-b,2.8)
        self.assertEqual(vb,0.)


if __name__ == '__main__':
    unittest.main()
