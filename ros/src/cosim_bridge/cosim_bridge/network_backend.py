"""JSONL lockstep transport: Veins steps SUMO, ROS2 publishes state and returns commands."""
import json
import socket
import subprocess
import time
from pathlib import Path
import rclpy
from cosim_interfaces.msg import V2VWarning
from geometry_msgs.msg import Point


class NetworkBackend:
    def __init__(self, owner):
        self.owner = owner
        self.server = socket.socket()
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('127.0.0.1', 9998))
        self.server.listen(1)
        self.server.settimeout(60)
        command = ['python3', str(owner.root / 'omnet/scripts/run_network.py')]
        if owner.gui:
            command.append('--gui')
        if owner.get_parameter('network_gui').value:
            command.append('--qtenv')
        self.output = (owner.trajectory_dir / 'network_process.log').open('w')
        self.process = subprocess.Popen(command, stdout=self.output, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        self.server.settimeout(1.0)
        deadline = time.monotonic() + 60
        try:
            while True:
                if self.process.poll() is not None:
                    raise RuntimeError('网络启动失败，请查看 ros/log/events/network_process.log')
                if time.monotonic() > deadline:
                    raise RuntimeError('网络连接超时，请查看 network_process.log')
                try:
                    self.connection, _ = self.server.accept()
                    break
                except socket.timeout:
                    continue
        except BaseException:
            import os, signal
            if self.process.poll() is None:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=5)
            self.server.close()
            self.output.close()
            raise
        # SUMO GUI may wait indefinitely for Play; do not include that wait in elapsed time.
        self.stream = self.connection.makefile('r')
        self.needs_ack = False
        self.first_frame = True

    def tick(self):
        o = self.owner
        if o.finished:
            if self.needs_ack:
                self.ack()
            o.publish_state(False)
            return
        if self.needs_ack:
            self.ack()
        line = self.stream.readline()
        if not line:
            raise RuntimeError('网络进程断开，请查看 ros/log/events/network_process.log')
        frame = json.loads(line)
        self.needs_ack = True
        o.now = float(frame['time'])
        if self.first_frame:
            o.started = time.monotonic() - o.now / o.rate
            self.first_frame = False
        o.states = {v: (float(c['x']), float(c['y']), float(c['yaw']), float(c['speed']))
                    for v, c in frame['cars'].items()}
        if o.now >= 5 and not o.braked:
            o.braked = True
            o.log.write('front_brake', 5.0, event_id='front_brake_1', backend='veins_inet')
        if frame['command_active'] and o.applied_command is None:
            o.applied_command = self.sent_command
            o.response_start_speed = 15.0
            o.log.write('brake_command_applied', frame['applied_time'], event_id=o.applied_command.event_id,
                        simulated=False, backend='veins_inet')
        a, b = o.states.values()
        gap = a[0] - b[0] - 5.0
        o.minimum_gap = min(o.minimum_gap, gap)
        o.trajectory.write(f'{o.now:.2f},{a[0]:.4f},{a[3]:.4f},{b[0]:.4f},{b[3]:.4f},{gap:.4f}\n')
        o.trajectory.flush()
        if o.applied_command is not None and not o.response_recorded and b[3] < o.response_start_speed-.01:
            o.response_recorded = True
            o.log.write('rear_speed_response', o.now, event_id=o.applied_command.event_id, speed_mps=b[3])
        # Publish /clock first; callbacks execute before the next tick acknowledgement.
        o.publish_state(True)
        for event in frame['warnings']:
            warning = V2VWarning()
            warning.header.frame_id = 'map'
            warning.header.stamp = rclpy.time.Time(seconds=float(event['source_time_s'])).to_msg()
            warning.event_id = event['event_id']
            warning.source_vehicle, warning.target_vehicle = 'car_a', 'car_b'
            warning.event_type = 'EMERGENCY_BRAKE'
            warning.position = Point(x=float(event['x']), y=float(event['y']), z=0.0)
            warning.speed_mps = float(event['speed'])
            warning.simulated = False
            o.real_warning_pub.publish(warning)
            o.log.write('network_warning_received', o.now, event_id=warning.event_id,
                        send_time_s=event['send_time_s'], receive_time_s=event['receive_time_s'],
                        network_delay_s=event['receive_time_s']-event['send_time_s'], simulated=False)
        if o.now >= o.duration-1e-8:
            o.finished = True
            o.trajectory.close()
            summary = dict(end_time_s=o.now, minimum_gap_m=o.minimum_gap, final_gap_m=gap,
                           ros_brake_applied=o.applied_command is not None, warning_seen=o.warning_seen,
                           simulated_warning=False, backend='veins_inet',
                           final_speed_mps={v: s[3] for v,s in o.states.items()})
            (o.trajectory_dir/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
            o.log.write('simulation_finished', o.now, **summary)
            o.get_logger().info(f'Veins + INET 仿真结束，t={o.now:.2f}s；保留最终画面。')

    def ack(self):
        command = self.owner.pending_command
        payload = {'command': None}
        if command is not None:
            payload['command'] = {'event_id': command.event_id, 'desired_gap_m': command.desired_gap_m}
            self.sent_command = command
            self.owner.pending_command = None
        self.connection.sendall((json.dumps(payload)+'\n').encode())
        self.needs_ack = False

    def close(self):
        self.connection.close()
        self.stream.close()
        self.server.close()
        if self.process.poll() is None:
            import os, signal
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        self.output.close()
