"""Validate status semantics and animation lifecycle without a running simulator."""
import unittest
from unittest.mock import patch

from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker
from cosim_bridge.communication_visuals import CommunicationVisuals
from cosim_bridge.sumo_bridge import SumoBridge


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


if __name__ == '__main__':
    unittest.main()
