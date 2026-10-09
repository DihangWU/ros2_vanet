"""Toggle display filtering in an isolated running demo/preview; originals stay unchanged."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from cosim_bridge.ultrasonic_display import ultrasonic_specs
from pathlib import Path
rclpy.init();node=Node('check_ground_toggle')
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
publisher=node.create_publisher(Bool,'/display/show_ground_returns',qos)
names=['lidar']+['radar/'+n for n in ['front_left','front_right','rear_left','rear_right','side_left','side_right']]
names+=['ultrasonic/'+n for n in ultrasonic_specs(Path(__file__).resolve().parents[2])]
received={};raw={};subscriptions=[]
for name in names:
 subscriptions.append(node.create_subscription(PointCloud2,f'/car_b/{name}/points_colored',lambda m,k=name:received.update({k:m}),qos_profile_sensor_data))
 subscriptions.append(node.create_subscription(PointCloud2,f'/car_b/{name}/points',lambda m,k=name:raw.update({k:m}),qos_profile_sensor_data))
def wait(predicate,seconds=25):
 deadline=time.monotonic()+seconds
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.1)
  if predicate():return
 raise AssertionError('Timed out waiting for display state')
def hidden():
 return len(received)==len(names) and all(m.header.frame_id=='map' and (point_cloud2.read_points(m)['z']>.15).all() for m in received.values())
try:
 publisher.publish(Bool(data=True))
 wait(lambda:len(received)==len(raw)==23 and received['lidar'].width>0)
 publisher.publish(Bool(data=False));received.clear()
 wait(hidden)
 assert received['lidar'].width>0,'Front obstacle points must remain visible'
 assert any((point_cloud2.read_points(m,skip_nans=True)['z']<0).any() for m in raw.values()),'Raw sensor ground was altered'
 publisher.publish(Bool(data=True));received.clear()
 wait(lambda:len(received)==23 and any((point_cloud2.read_points(m)['z']<=.15).any() for m in received.values()))
 print('PASS: 23 display channels hide/restore ground, obstacle lidar returns retained, raw clouds unaffected',flush=True)
finally:
 publisher.publish(Bool(data=True));node.destroy_node();rclpy.shutdown()
