"""RViz communication markers, driven by actual events rather than log polling."""
import math
import time

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class CommunicationVisuals:
    def __init__(self, radius):
        self.radius = radius
        self.connected = False
        self.failed = False
        self.simulated = False
        self.event_id = None
        self.sent = None
        self.received = None
        self.send_position = None
        self.receive_position = None
        self.published = None
        self.applied = None
        self.animation_started = None

    def event(self, event):
        if event['event'] == 'packet_sent':
            self.event_id = event['event_id']
            self.sent = float(event['send_time_s'])
            self.send_position = (float(event['x']), float(event['y']))
        elif event['event'] == 'packet_received':
            self.event_id = event['event_id']
            self.sent = float(event['send_time_s'])
            self.received = float(event['receive_time_s'])
            self.send_position = (float(event['x']), float(event['y']))
            self.receive_position = (float(event['receive_x']), float(event['receive_y']))
            # Slow replay makes a sub-millisecond packet visible to a human.
            self.animation_started = time.monotonic()

    def status(self, now):
        if self.failed:
            return 'DISCONNECTED', (1.0, 0.15, 0.15)
        if self.received is not None:
            return ('TEST WARNING' if self.simulated else 'WARNING RECEIVED'), (1.0, 0.6, 0.0)
        if self.sent is not None and now - self.sent >= 0.5:
            return 'NOT RECEIVED (timeout 0.5 s)', (1.0, 0.15, 0.15)
        if self.sent is not None:
            return 'WAITING FOR PACKET', (0.1, 0.8, 0.2)
        return ('CONNECTED (reception unverified)' if self.connected else 'WAITING'), (0.1, 0.8, 0.2)

    def markers(self, states, now, stamp, make_marker):
        result = MarkerArray()
        if len(states) != 2:
            return result
        a, b = states['car_a'], states['car_b']
        status, color = self.status(now)

        def marker(identifier, kind):
            item = make_marker('v2v', identifier, kind, stamp)
            item.color.r, item.color.g, item.color.b = color
            result.markers.append(item)
            return item

        def text(identifier, position, content, size=0.8):
            item = marker(identifier, Marker.TEXT_VIEW_FACING)
            item.pose.position = Point(x=position[0], y=position[1], z=position[2])
            item.scale.z = size
            item.text = content
            return item

        def seconds(value):
            return '--' if value is None else f'{value:.6f} s'

        delay = '--' if self.received is None or self.simulated else f'{(self.received-self.sent)*1000:.3f} ms'
        content = '\n'.join([
            f'V2V STATUS: {status}',
            f'PACKET: {self.event_id or "--"}',
            f'SENT: {seconds(self.sent)}', f'RECEIVED: {seconds(self.received)}',
            f'NETWORK DELAY: {delay}', 'SOURCE: car_a   TARGET: car_b',
            f'ROS CMD PUBLISHED: {seconds(self.published)}',
            f'SUMO CMD APPLIED: {seconds(self.applied)}',
            'MODE: TEST INPUT' if self.simulated else 'MODE: INET 802.11p / UDP',
        ])
        # World-space text leaves the user's freely movable camera untouched.
        text(0, (125.0, 19.0, 5.0), content, 1.1)
        arrow = marker(1, Marker.ARROW)
        arrow.points = [Point(x=a[0], y=a[1], z=2.1), Point(x=b[0], y=b[1], z=2.1)]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.15, 0.5, 0.8
        text(2, ((a[0]+b[0])/2, -1.6, 4.5), status)

        for identifier, car in [(3, a), (4, b)]:
            circle = marker(identifier, Marker.LINE_STRIP)
            circle.scale.x = 0.1
            circle.color.a = 0.35
            circle.points = [Point(x=car[0]+self.radius*math.cos(i*math.tau/96),
                                   y=car[1]+self.radius*math.sin(i*math.tau/96), z=0.15)
                             for i in range(97)]
        text(5, (125.0, 8.0, 1.0), f'RANGE GUIDE: {self.radius:g} m (illustrative)', 0.8)

        for identifier, position, label in [
                (6, self.send_position, 'TX'), (8, self.receive_position, 'RX')]:
            dot = marker(identifier, Marker.SPHERE)
            caption = marker(identifier+1, Marker.TEXT_VIEW_FACING)
            if position is None:
                dot.action = caption.action = Marker.DELETE
                continue
            dot.pose.position = Point(x=position[0], y=position[1], z=2.8)
            dot.scale.x = dot.scale.y = dot.scale.z = 0.7
            caption.pose.position = Point(x=position[0], y=position[1]+2, z=3.3)
            caption.scale.z = 0.8
            caption.text = f'{label} position'

        packet = marker(10, Marker.SPHERE)
        replay = marker(11, Marker.TEXT_VIEW_FACING)
        elapsed = None if self.animation_started is None else time.monotonic()-self.animation_started
        if elapsed is None or elapsed > 1.5 or self.failed:
            packet.action = replay.action = Marker.DELETE
        else:
            fraction = min(1.0, elapsed/1.5)
            start, end = self.send_position, self.receive_position
            packet.pose.position = Point(x=start[0]+(end[0]-start[0])*fraction,
                                         y=start[1]+(end[1]-start[1])*fraction, z=3.0)
            packet.scale.x = packet.scale.y = packet.scale.z = 1.0
            replay.pose.position = Point(x=(start[0]+end[0])/2, y=-6.0, z=3.5)
            replay.scale.z = 0.8
            replay.text = 'PACKET REPLAY (1.5 s visual only)'
        return result
