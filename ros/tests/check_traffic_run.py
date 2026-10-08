"""Check actual default-scene output after a completed 30-second run."""
import csv
import json
from pathlib import Path

root = Path(__file__).resolve().parents[2]
output = root / 'ros/log/events'
summary = json.loads((output / 'summary.json').read_text())
assert summary['scenario'] == 'traffic' and summary['end_time_s'] == 30.0
assert summary['vehicle_count'] == 11
assert summary['ros_brake_applied'] and summary['warning_seen']
assert summary.get('collision_steps', 0) == 0
assert 2.5 <= summary['minimum_gap_m'] <= summary['final_gap_m'] <= 2.6
assert summary['final_speed_mps']['car_a'] == summary['final_speed_mps']['car_b'] == 0.0

rows = list(csv.DictReader((output / 'traffic_trajectory.csv').open()))
expected = {'car_a', 'car_b'} | {f'bg_{i:02d}' for i in range(1, 10)}
assert len(rows) == 6600
frames = {}
for row in rows:
    frames.setdefault(row['time_s'], set()).add(row['vehicle_id'])
    vehicle = row['vehicle_id']
    lane_y = -4.8 if vehicle.startswith('car_') else (-8.0 if vehicle in {'bg_07', 'bg_08', 'bg_09'} else -1.6)
    assert abs(float(row['y_m'])-lane_y) < 0.001
    assert 0 < float(row['x_m']) < 400
assert len(frames) == 600 and all(ids == expected for ids in frames.values())
for i in range(1, 7):
    vehicle = f'bg_{i:02d}'
    track = [r for r in rows if r['vehicle_id'] == vehicle]
    assert float(track[-1]['x_m']) > float(track[0]['x_m']) + 170
    assert summary['final_speed_mps'][vehicle] == 6.0 + (i-1)*0.5
for vehicle, speed in [('bg_07', 6.5), ('bg_08', 7.5), ('bg_09', 8.5)]:
    track = [r for r in rows if r['vehicle_id'] == vehicle]
    assert float(track[-1]['x_m']) > float(track[0]['x_m']) + 190
    assert summary['final_speed_mps'][vehicle] == speed
print('PASS: 600 frames / 11 cars, three correct lanes, moving background traffic, A/B close stop.')
