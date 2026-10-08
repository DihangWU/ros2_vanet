"""用 RViz 原生几何 Marker 组成三维小车，局部坐标 x 朝前。"""
import math
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def car_markers(vehicle, stamp, x, y, yaw):
    color = ((1.0, 0.15, 0.12) if vehicle == 'car_a' else
             ((0.12, 0.35, 1.0) if vehicle == 'car_b' else (0.6, 0.65, 0.7)))
    parts = [
        ('body', (0, 0, .65), (5, 1.8, .65), color, False),
        ('cabin', (-.25, 0, 1.2), (2.4, 1.6, .7), color, False),
        ('windshield', (.96, 0, 1.3), (.03, 1.35, .45), (.08, .18, .25), False),
    ]
    for side in (-1, 1):
        parts.append(('window', (-.25, side*.81, 1.3), (2, .03, .45), (.08, .18, .25), False))
        for axle in (-1.65, 1.65):
            parts.append(('wheel', (axle, side*.95, .38), (.76, .76, .25), (.04, .04, .04), True))
        parts.append(('headlight', (2.51, side*.6, .7), (.04, .35, .25), (1., 1., .8), False))
        parts.append(('taillight', (-2.51, side*.6, .7), (.04, .35, .25), (1., .05, .05), False))
    result = []
    for i, (_, offset, scale, rgb, wheel) in enumerate(parts):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = vehicle + '/model'
        marker.id = i
        marker.type = Marker.CYLINDER if wheel else Marker.CUBE
        marker.action = Marker.ADD
        dx, dy, dz = offset
        marker.pose.position = Point(x=x+dx*math.cos(yaw)-dy*math.sin(yaw),
                                     y=y+dx*math.sin(yaw)+dy*math.cos(yaw), z=dz)
        # yaw * roll，车轮圆柱轴由 z 转到 y。
        roll = math.pi/2 if wheel else 0.0
        marker.pose.orientation.x = math.cos(yaw/2)*math.sin(roll/2)
        marker.pose.orientation.y = math.sin(yaw/2)*math.sin(roll/2)
        marker.pose.orientation.z = math.sin(yaw/2)*math.cos(roll/2)
        marker.pose.orientation.w = math.cos(yaw/2)*math.cos(roll/2)
        marker.scale.x, marker.scale.y, marker.scale.z = map(float, scale)
        marker.color.r, marker.color.g, marker.color.b = rgb
        marker.color.a = 1.0
        result.append(marker)
    return result
