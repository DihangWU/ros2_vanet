"""Check a completed real 30-second traffic/lidar/network run, using truth only for evaluation."""
import csv
import json
from pathlib import Path
import numpy as np

root = Path(__file__).resolve().parents[2]
output = root/'ros/log/events'
summary = json.loads((output/'summary.json').read_text())
assert summary['control_mode'] == 'lidar'
assert summary['scenario'] == 'traffic' and summary['end_time_s'] == 30.
assert summary['vehicle_count'] == 11 and summary['collision_steps'] == 0
assert summary['ros_brake_applied'] and summary['warning_seen']
assert summary['final_speed_mps']['car_a'] == summary['final_speed_mps']['car_b'] == 0.
assert 2.3 < summary['minimum_gap_m'] <= summary['final_gap_m'] < 2.9

events = [json.loads(line) for line in (output/'lidar_brake.jsonl').read_text().splitlines()]
measurements = [e for e in events if e['event'] == 'lidar_measurement']
commands = [e for e in events if e['event'] == 'control_published']
assert len(measurements) >= 100 and len(commands) >= 500
assert any(e['event'] == 'v2v_trigger' and not e['simulated'] for e in events)
assert any(e['state'] == 'V2V_LIDAR_BRAKE' for e in commands)
assert commands[-1]['state'] == 'STOPPED_HOLD'
assert all(-8. <= e['acceleration_mps2'] <= 1.5 for e in commands)
assert all(0. <= e['target_speed_mps'] <= 15. for e in commands)
assert all(a['sequence'] < b['sequence'] for a, b in zip(commands, commands[1:]))

# Compare filtered measurements with recorded SUMO truth at measurement time.
# This CSV is never an input to the running controller.
with (output/'trajectory.csv').open() as stream:
    rows = list(csv.DictReader(stream))
assert len(rows) == 600
times = np.array([float(r['time_s']) for r in rows])
gaps = np.array([float(r['gap_m']) for r in rows])
valid = [m for m in measurements if m['measured_gap_m'] is not None]
assert len(valid) >= .95*len(measurements)
errors = np.array([abs(m['measured_gap_m']-np.interp(m['measurement_time_s'], times, gaps))
                   for m in valid])
assert np.median(errors) < .3 and np.quantile(errors, .95) < 1.
assert abs(valid[-1]['measured_gap_m']-summary['final_gap_m']) < .2

network = [json.loads(line) for line in (root/'omnet/results/network_events.jsonl').read_text().splitlines()]
accepted = [e for e in network if e['event'] == 'lidar_control_accepted']
assert len(accepted) >= 450
published = {e['sequence']: e for e in commands}
for e in accepted:
    assert e['sequence'] in published
    assert abs(e['target_speed_mps']-published[e['sequence']]['target_speed_mps']) < 1e-8
print(f"PASS: real lidar feedback, {len(measurements)} scans, {len(accepted)} executed commands, "
      f"gap={summary['final_gap_m']:.3f}m, median error={np.median(errors):.3f}m, no collision.")
