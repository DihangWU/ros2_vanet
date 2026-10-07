"""每个节点独立 JSONL 日志，sim_time_s 使用 SUMO 时钟。"""
import json
import time
from pathlib import Path


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class EventLog:
    def __init__(self, root, name):
        directory = Path(root) / 'ros/log/events'
        directory.mkdir(parents=True, exist_ok=True)
        self.path = directory / (name + '.jsonl')
        self.path.write_text('')

    def write(self, event, sim_time, **fields):
        record = dict(event=event, sim_time_s=round(sim_time, 6),
                      wall_monotonic_s=time.monotonic(), **fields)
        with self.path.open('a') as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + '\n')
