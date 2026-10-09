"""Run beside an isolated preview/demo; --obstacle adds a near-front test cube."""
import argparse
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import MarkerArray
from cosim_bridge.ultrasonic_display import ultrasonic_specs

parser=argparse.ArgumentParser()
parser.add_argument('--obstacle',action='store_true',help='Create a temporary test cube in the running Gazebo scene')
args=parser.parse_args()
root=Path(__file__).resolve().parents[2];specs=ultrasonic_specs(root)
rclpy.init();node=Node('check_ultrasonic_run')
clouds,ranges,raw,stamps={},{},{},{};markers=set()
def record(message,name):
 clouds[name]=message
 stamps.setdefault(name,set()).add((message.header.stamp.sec,message.header.stamp.nanosec))
def scene(message):
 markers.update(m.ns for m in message.markers)
subscriptions=[node.create_subscription(MarkerArray,'/demo/markers',scene,10)]
for name in specs:
 base=f'/car_b/ultrasonic/{name}'
 subscriptions.extend([
  node.create_subscription(PointCloud2,base+'/points_colored',lambda m,k=name:record(m,k),qos_profile_sensor_data),
  node.create_subscription(PointCloud2,base+'/points',lambda m,k=name:raw.update({k:m}),qos_profile_sensor_data),
  node.create_subscription(Range,base+'/range',lambda m,k=name:ranges.update({k:m}),qos_profile_sensor_data)])
def wait(predicate,timeout=25):
 end=time.monotonic()+timeout
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.1)
  if predicate():return
 raise AssertionError(f'Missing streams: clouds={list(clouds)}, ranges={list(ranges)}')
transport=None
created=[]
try:
 wait(lambda:len(clouds)==len(ranges)==len(raw)==len(specs) and all(len(s)>3 for s in stamps.values()) and 'car_b/ultrasonics' in markers)
 for name in specs:
  assert clouds[name].header.frame_id=='map'
  assert raw[name].header.frame_id==ranges[name].header.frame_id==f'car_b/ultrasonic/{name}_link'
  assert math.isinf(ranges[name].range) or .15<=ranges[name].range<=5
  assert ranges[name].radiation_type==Range.ULTRASOUND
  colors=set(point_cloud2.read_points(clouds[name])['rgb'])
  assert colors and colors<={0xFFFF00,0xFFFFFF}
 if args.obstacle:
  from gz.transport13 import Node as TransportNode
  from gz.msgs10.entity_factory_pb2 import EntityFactory
  from gz.msgs10.entity_pb2 import Entity
  from gz.msgs10.boolean_pb2 import Boolean
  from cosim_bridge.preview_scene import world_vehicles
  x,y,z,*_=next(v['pose'] for v in world_vehicles(root,'traffic') if v['id']=='car_b')
  transport=TransportNode()
  cube=f'<sdf version="1.9"><model name="ultrasonic_test_cube"><static>true</static><pose>{x+4} {y} 0.6 0 0 0</pose><link name="body"><visual name="cube"><geometry><box><size>0.5 0.8 0.8</size></box></geometry></visual></link></model></sdf>'
  ok,response=transport.request('/world/cosim_demo/create',EntityFactory(sdf=cube),EntityFactory,Boolean,2000)
  assert ok and response.data
  created.append('ultrasonic_test_cube')
  wait(lambda:any(0xFFFF00 in point_cloud2.read_points(m)['rgb'] for n,m in clouds.items() if n.startswith('front')))
  assert any(.15<ranges[n].range<2 for n in specs if n.startswith('front'))
  for side,sign in [('left',1),('right',-1)]:
   name='ultrasonic_test_'+side
   cube=f'<sdf version="1.9"><model name="{name}"><static>true</static><pose>{x-.75} {y+sign*2.2} 0.6 0 0 0</pose><link name="body"><visual name="cube"><geometry><box><size>0.6 0.5 0.8</size></box></geometry></visual></link></model></sdf>'
   ok,response=transport.request('/world/cosim_demo/create',EntityFactory(sdf=cube),EntityFactory,Boolean,2000)
   assert ok and response.data
   created.append(name)
   wait(lambda:any(0xFFFF00 in point_cloud2.read_points(m)['rgb'] for n,m in clouds.items() if n.startswith(f'side_{side}_')))
 print(f'PASS: {len(specs)} live range/raw/map-cloud channels, distinct yellow/white colors, dynamic timestamps, sensor models'+('; front and both-side test cubes detected' if args.obstacle else ''),flush=True)
finally:
 if transport is not None:
  for name in created:
   transport.request('/world/cosim_demo/remove',Entity(name=name,type=Entity.MODEL),Entity,Boolean,2000)
 node.destroy_node();rclpy.shutdown()
