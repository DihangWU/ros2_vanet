"""Deletion must survive stale messages without clearing unrelated scene objects."""
import json
import tempfile
import socket
from pathlib import Path as FilePath
from types import SimpleNamespace
import unittest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from cosim_bridge.vehicle_lifecycle import RemovalTracker, MarkerHistory
from cosim_bridge.gazebo_sync import GazeboSync
from cosim_bridge.network_backend import NetworkBackend
from cosim_bridge.sumo_bridge import SumoBridge


class LifecycleTests(unittest.TestCase):
    def test_startup_and_transient_absence_are_not_deletions(self):
        tracker = RemovalTracker(['car_b', 'bg_01'])
        self.assertEqual(tracker.observe([]), set())
        tracker.observe(['car_b', 'bg_01'])
        self.assertEqual(tracker.observe(['car_b']), set())
        tracker.observe(['car_b', 'bg_01'])
        self.assertEqual(tracker.observe(['car_b']), set())
        self.assertEqual(tracker.observe(['car_b']), {'bg_01'})
        self.assertEqual(tracker.observe(['car_b', 'bg_01']), {'bg_01'})

    def test_exact_marker_deletes_keep_road_and_survivors(self):
        history = MarkerHistory()
        def scene(*names):
            return MarkerArray(markers=[Marker(ns=name, id=0, action=Marker.ADD) for name in names])
        history.reconcile(scene('road', 'bg_01/model', 'car_b/model'), Time())
        result = history.reconcile(scene('road', 'car_b/model'), Time())
        self.assertEqual([(m.ns,m.id) for m in result.markers if m.action==Marker.DELETE], [('bg_01/model',0)])
        again = history.reconcile(scene('road', 'car_b/model'), Time())
        self.assertTrue(any(m.ns=='bg_01/model' and m.action==Marker.DELETE for m in again.markers))

    def test_sync_ignores_queued_odom_from_removed_car(self):
        node = SimpleNamespace(vehicle_ids={'car_b','bg_01'}, latest={'bg_01':Pose()}, latest_steps={'bg_01':10})
        GazeboSync.on_lifecycle(node, String(data=json.dumps({'deleted':['bg_01']})))
        self.assertEqual(node.vehicle_ids, {'car_b'})
        self.assertEqual(node.latest_steps,{})
        GazeboSync.receive(node, 'bg_01', None)
        self.assertEqual(node.latest,{})

    def test_network_ack_delegates_remove_to_only_traci_owner(self):
        sent=[]
        owner=SimpleNamespace(pending_command=None, longitudinal=None, lidar_mode=False,
                              pending_removals={'bg_01'}, scene_stopped=False)
        backend=SimpleNamespace(owner=owner, connection=SimpleNamespace(sendall=sent.append), stop_ack_sent=False)
        NetworkBackend.ack(backend)
        reply=json.loads(sent[0])
        self.assertEqual(reply['remove_vehicles'],['bg_01'])
        self.assertFalse(reply['stop'])
        self.assertFalse(owner.pending_removals)

    def test_network_gui_pause_and_partial_frames_do_not_block_callbacks(self):
        left, right = socket.socketpair()
        try:
            backend=SimpleNamespace(connection=left, receive_buffer=b'')
            self.assertIsNone(NetworkBackend.read_frame(backend))
            right.sendall(b'{"time":')
            self.assertIsNone(NetworkBackend.read_frame(backend))
            right.sendall(b'1}\n')
            self.assertEqual(NetworkBackend.read_frame(backend), '{"time":1}')
        finally:
            left.close();right.close()

    def test_removal_clears_path_and_core_removal_stops_demo(self):
        for vehicle in ('bg_01','car_b'):
            paths=[]
            node=SimpleNamespace(vehicle_ids=['bg_01','car_a','car_b'], removed_vehicles=set(),pending_removals=set(),
                                 states={vehicle:(1,2,0,0)},now=2.,paths={},path_pub={vehicle:SimpleNamespace(publish=paths.append)},
                                 log=SimpleNamespace(write=lambda *a,**kw:None),network_mode=True,finished=False,scene_stopped=False,
                                 get_logger=lambda:SimpleNamespace(warning=lambda *a:None),publish_state=lambda *a:None)
            with tempfile.TemporaryDirectory() as directory:
                node.trajectory_dir=FilePath(directory)
                SumoBridge.on_lifecycle(node,String(data=json.dumps({'deleted':[vehicle]})))
            self.assertNotIn(vehicle,node.states)
            self.assertEqual(paths[0].poses,[])
            self.assertEqual(node.finished,vehicle=='car_b')


if __name__=='__main__':
    unittest.main()
