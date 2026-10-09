"""Delete a background car in an isolated running preview/demo and verify cleanup."""
import argparse
import json
import time
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import String
from gz.transport13 import Node as TransportNode
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.boolean_pb2 import Boolean
from cosim_bridge.vehicle_lifecycle import SCENE_QOS
from rclpy.qos import qos_profile_sensor_data

parser=argparse.ArgumentParser()
parser.add_argument('--mode', choices=['preview','demo'],required=True)
parser.add_argument('--core',action='store_true',help='Also delete car_a and check that the demo stops')
parser.add_argument('--core-vehicle', choices=['car_a','car_b'],default='car_a')
args=parser.parse_args()
rclpy.init();node=Node('check_vehicle_removal_run');transport=TransportNode()
state={'markers':[], 'deleted':set(), 'clock':0., 'path':None, 'array':None, 'clouds':{}, 'image':None}
subscriptions=[
 node.create_subscription(MarkerArray,'/demo/markers',lambda m:state.update(markers=m.markers,array=m),10),
 node.create_subscription(String,'/gazebo/vehicle_lifecycle',lambda m:state.update(deleted=set(json.loads(m.data)['deleted'])),SCENE_QOS),
 node.create_subscription(Clock,'/clock',lambda m:state.update(clock=m.clock.sec+m.clock.nanosec/1e9),10),
 node.create_subscription(Path,'/bg_01/path',lambda m:state.update(path=m),10)]

cloud_names = ['lidar', 'radar/front_left', 'radar/front_right', 'radar/rear_left', 'radar/rear_right', 'radar/side_left', 'radar/side_right']
cloud_names += [f'ultrasonic/{end}_{location}' for end in ('front','rear')
                for location in ('left_outer','left_inner','right_inner','right_outer')]
cloud_names += [f'ultrasonic/side_{side}_{location}' for side in ('left','right')
                for location in ('front','mid_front','mid_rear','rear')]
for name in cloud_names:
 subscriptions.append(node.create_subscription(PointCloud2, f'/car_b/{name}/points_colored', lambda m,key=name:state['clouds'].update({key:m}),qos_profile_sensor_data))
subscriptions.append(node.create_subscription(Image,'/car_b/camera/surround/image_raw',lambda m:state.update(image=m),10))

def wait_for(predicate,timeout=20):
 deadline=time.monotonic()+timeout
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.1)
  if predicate():return
 raise AssertionError(f'Timed out: deleted={state["deleted"]}, clock={state["clock"]}')

def remove(name):
 ok,response=transport.request('/world/cosim_demo/remove',Entity(name=name,type=Entity.MODEL),Entity,Boolean,2000)
 assert ok and response.data,'Gazebo did not accept deletion'
 wait_for(lambda:name in state['deleted'] and any(m.action==Marker.DELETE and m.ns==name+'/model' for m in state['markers']))
 end=time.monotonic()+1.
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.1)
  assert not any(m.action==Marker.ADD and (m.ns==name or m.ns.startswith(name+'/')) for m in state['markers']),'Removed vehicle reappeared'

try:
 wait_for(lambda:state['clock']>.5 and any(m.ns=='bg_01/model' and m.action==Marker.ADD for m in state['markers']))
 remove('bg_01')
 assert any(m.ns.endswith('road') and m.action==Marker.ADD for m in state['markers'])
 assert any(m.ns=='car_b/model' and m.action==Marker.ADD for m in state['markers'])
 if args.mode=='demo':
  wait_for(lambda:state['path'] is not None and not state['path'].poses)
 before=state['clock'];wait_for(lambda:state['clock']>before+.2)
 if args.core:
  remove(args.core_vehicle);before=state['clock']
  if args.core_vehicle=='car_b':
   wait_for(lambda:len(state['clouds'])==len(cloud_names) and all(m.width==0 for m in state['clouds'].values()) and state['image'] is not None and not any(state['image'].data))
  end=time.monotonic()+1.
  while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
  if args.mode=='demo':assert abs(state['clock']-before)<.051,'Core removal did not stop demo'
 print(f'PASS: {args.mode} deletion clears vehicle markers, preserves road/car_b and keeps background-deletion simulation advancing'+('; core removal handled' if args.core else ''))
finally:
 node.destroy_node();rclpy.shutdown()
