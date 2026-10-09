"""Validate status semantics and animation lifecycle without a running simulator."""
import unittest
from unittest.mock import patch

from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker
from cosim_bridge.communication_visuals import CommunicationVisuals
from cosim_bridge.sumo_bridge import SumoBridge
from cosim_bridge.vehicle_lifecycle import MarkerHistory


class CommunicationTests(unittest.TestCase):
    def setUp(self):
        self.visuals = CommunicationVisuals(100.0)
        self.states = {'car_a': (172.35, -1.6, 0.0, 15.0),
                       'car_b': (127.35, -1.6, 0.0, 15.0)}
        self.sent = dict(event='packet_sent', event_id='front_brake_1',
                         send_time_s=5.001, x=172.35, y=-1.6)

    def markers(self):
        return {m.id: m for m in self.visuals.markers(
            self.states, 5.05, Time(), SumoBridge.marker).markers}

    def test_missing_packet_and_disconnect(self):
        self.visuals.connected = True
        self.assertIn('unverified', self.visuals.status(0)[0])
        self.visuals.event(self.sent)
        self.assertEqual(self.visuals.status(5.05)[0], 'WAITING FOR PACKET')
        self.assertIn('NOT RECEIVED', self.visuals.status(5.55)[0])
        self.visuals.failed = True
        self.assertEqual(self.visuals.status(5.55)[0], 'DISCONNECTED')

    def test_real_delay_positions_and_replay_deletion(self):
        event = dict(self.sent, event='packet_received', receive_time_s=5.001152150104,
                     receive_x=127.35, receive_y=-1.6)
        with patch('cosim_bridge.communication_visuals.time.monotonic', return_value=100):
            self.visuals.event(event)
        self.visuals.published = self.visuals.applied = 5.05
        with patch('cosim_bridge.communication_visuals.time.monotonic', return_value=100.75):
            markers = self.markers()
        self.assertIn('NETWORK DELAY: 0.152 ms', markers[0].text)
        self.assertIn('ROS CMD PUBLISHED: 5.050000 s', markers[0].text)
        self.assertAlmostEqual(markers[10].pose.position.x, 149.85)
        self.assertEqual(markers[8].pose.position.x, 127.35)
        self.assertEqual(len(markers[3].points), 97)
        with patch('cosim_bridge.communication_visuals.time.monotonic', return_value=102):
            self.assertEqual(self.markers()[10].action, Marker.DELETE)

    def test_test_input_does_not_claim_wireless_delay(self):
        self.visuals.simulated = True
        self.visuals.sent, self.visuals.received = 5.0, 5.1
        self.assertIn('NETWORK DELAY: --', self.markers()[0].text)

    def test_replay_expiry_cleanup_has_no_duplicate_marker_keys(self):
        history = MarkerHistory()
        event = dict(self.sent, event='packet_received', receive_time_s=5.001152,
                     receive_x=127.35, receive_y=-1.6)
        with patch('cosim_bridge.communication_visuals.time.monotonic', return_value=100):
            self.visuals.event(event)
        for wall_time in (100.75, 102, 103):
            with patch('cosim_bridge.communication_visuals.time.monotonic', return_value=wall_time):
                array = history.reconcile(self.visuals.markers(
                    self.states, 5.05, Time(), SumoBridge.marker), Time())
            keys = [(m.ns, m.id) for m in array.markers]
            self.assertEqual(len(keys), len(set(keys)))
            if wall_time >= 102:
                self.assertEqual({m.id for m in array.markers if m.action == Marker.DELETE}, {10, 11})
        # Removing a core vehicle still cleans up all previously active markers.
        array = history.reconcile(self.visuals.markers({}, 5.05, Time(), SumoBridge.marker), Time())
        keys = [(m.ns, m.id) for m in array.markers]
        self.assertEqual(len(keys), len(set(keys)))
        self.assertTrue(all(m.action == Marker.DELETE for m in array.markers))

    def test_background_traffic_does_not_hide_pair_markers(self):
        self.states['car_a'] = (172.35, -4.8, 0.0, 15.0)
        self.states['car_b'] = (127.35, -4.8, 0.0, 15.0)
        self.states.update({f'bg_{i:02d}': (float(i*60), -1.6, 0.0, 10.0) for i in range(1, 7)})
        markers = self.markers()
        self.assertEqual(len(markers), 12)
        self.assertEqual(markers[1].points[0].y, -4.8)
        self.assertEqual(markers[2].pose.position.y, -4.8)


if __name__ == '__main__':
    unittest.main()
