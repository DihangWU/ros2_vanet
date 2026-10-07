#!/usr/bin/env python3
"""SUMO 独立练习：只控制前车急刹，后车由 SUMO 跟车模型控制。"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DURATION = 15
BRAKE_TIME = 5.0
BRAKE_DURATION = 2.0
STOP_THRESHOLD = 0.05  # m/s：近似停车，原始速度仍完整记录


def load_traci(binary):
    # 优先使用已安装的 Python 包，否则从 SUMO 安装目录加载。
    try:
        import traci
    except ImportError:
        home = os.environ.get("SUMO_HOME")
        tools = Path(home) / "tools" if home else Path(binary).resolve().parents[1] / "tools"
        if not (tools / "traci").is_dir():
            raise RuntimeError("找不到 TraCI，请设置 SUMO_HOME 为 SUMO 安装目录")
        sys.path.insert(0, str(tools))
        import traci
    return traci


def build_network():
    binary = shutil.which("netconvert")
    if not binary:
        raise RuntimeError("找不到 netconvert，请把 SUMO/bin 加入 PATH")
    subprocess.run([
        binary, "--node-files", str(ROOT / "network/straight.nod.xml"),
        "--edge-files", str(ROOT / "network/straight.edg.xml"),
        "--output-file", str(ROOT / "network/straight.net.xml"),
    ], check=True)


def run(gui, duration, playback_rate, realtime):
    binary = shutil.which("sumo-gui" if gui else "sumo")
    if not binary:
        raise RuntimeError("找不到 SUMO 可执行文件，请检查 PATH")
    traci = load_traci(binary)
    build_network()
    output = ROOT / "output"
    output.mkdir(exist_ok=True)
    events = []
    minimum_gap = float("inf")
    collision_steps = 0
    stopped_at = {}
    final_speeds = {}
    brake_sent = False
    hold_sent = False
    command = [binary, "-c", str(ROOT / "config/demo.sumocfg"),
               "--no-step-log", "true", "--seed", "42", "--end", str(duration)]
    if gui:
        command += ["--start", "--delay", "0", "--quit-on-end"]
    traci.start(command)
    try:
        with (output / "trajectory.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_s", "vehicle_id", "x_m", "y_m", "speed_mps", "accel_mps2", "gap_m"])
            # 单调时钟不受系统时间调整影响；从仿真开始推进时计时。
            wall_start = time.monotonic()
            print(f"播放模式：{playback_rate:g} 倍实时" if realtime else "播放模式：无界面快速计算", flush=True)
            while traci.simulation.getTime() < duration - 1e-8:
                now = traci.simulation.getTime()
                # 在 t=5.00 的状态上发出命令，影响之后的仿真步。
                if now >= BRAKE_TIME - 1e-8 and not brake_sent:
                    traci.vehicle.slowDown("car_a", 0.0, BRAKE_DURATION)
                    events.append({"time_s": now, "event": "front_brake", "vehicle": "car_a",
                                   "wall_elapsed_s": time.monotonic() - wall_start})
                    brake_sent = True
                    print(f"[仿真 {now:.2f}s / 现实 {time.monotonic() - wall_start:.2f}s] 前车开始制动", flush=True)
                if now >= BRAKE_TIME + BRAKE_DURATION - 1e-8 and not hold_sent:
                    traci.vehicle.setSpeed("car_a", 0.0)
                    hold_sent = True
                # 此脚本是独立练习中唯一调用 simulationStep 的客户端。
                traci.simulationStep()
                now = traci.simulation.getTime()
                if traci.simulation.getCollidingVehiclesIDList():
                    collision_steps += 1
                vehicles = set(traci.vehicle.getIDList())
                if vehicles != {"car_a", "car_b"}:
                    raise RuntimeError(f"t={now}: 车辆未按预期出现：{vehicles}")
                if gui:
                    # 跟随两车中点，让两车保持在放大的画面内。
                    ax, ay = traci.vehicle.getPosition("car_a")
                    bx, by = traci.vehicle.getPosition("car_b")
                    traci.gui.setOffset("View #0", (ax + bx) / 2, (ay + by) / 2)
                gap = (traci.vehicle.getLanePosition("car_a")
                       - traci.vehicle.getLength("car_a")
                       - traci.vehicle.getLanePosition("car_b"))
                minimum_gap = min(minimum_gap, gap)
                for vehicle in ("car_a", "car_b"):
                    x, y = traci.vehicle.getPosition(vehicle)
                    speed = traci.vehicle.getSpeed(vehicle)
                    final_speeds[vehicle] = speed
                    if speed < STOP_THRESHOLD and vehicle not in stopped_at:
                        stopped_at[vehicle] = now
                    writer.writerow([f"{now:.2f}", vehicle, f"{x:.4f}", f"{y:.4f}",
                                     f"{speed:.4f}", f"{traci.vehicle.getAcceleration(vehicle):.4f}",
                                     f"{gap:.4f}" if vehicle == "car_b" else ""])
                if realtime:
                    # 使用绝对目标时间，避免每步 sleep 累积计算开销。
                    target_wall_time = wall_start + now / playback_rate
                    remaining = target_wall_time - time.monotonic()
                    if remaining > 0:
                        time.sleep(remaining)
            wall_elapsed = time.monotonic() - wall_start
    finally:
        traci.close()
    summary = {"mode": "sumo_following_baseline", "end_time_s": now,
               "realtime": realtime, "playback_rate": playback_rate,
               "wall_elapsed_s": wall_elapsed,
               "events": events, "minimum_gap_m": minimum_gap,
               "collision_steps": collision_steps, "near_stop_threshold_mps": STOP_THRESHOLD, "near_stopped_at_s": stopped_at,
               "final_speed_mps": final_speeds}
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    if collision_steps or minimum_gap < 2.5 - 1e-3 or any(v > STOP_THRESHOLD for v in final_speeds.values()):
        raise RuntimeError("场景检查失败：请查看 output 中的轨迹和摘要")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true", help="打开 SUMO GUI")
    parser.add_argument("--duration", type=int, default=DEFAULT_DURATION,
                        help="仿真时长（秒），默认 15，至少 15")
    parser.add_argument("--playback-rate", type=float, default=1.0,
                        help="实时播放倍率：默认 1，0.5 为半速，2 为两倍速")
    parser.add_argument("--realtime", action="store_true",
                        help="无界面模式也按现实时间推进；GUI 默认启用")
    args = parser.parse_args()
    if args.duration < 15:
        parser.error("--duration 至少为 15 秒，以便观察后车停车")
    if not math.isfinite(args.playback_rate) or args.playback_rate <= 0:
        parser.error("--playback-rate 必须为有限的正数")
    try:
        run(args.gui, args.duration, args.playback_rate, args.gui or args.realtime)
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print(f"错误：{error}", file=sys.stderr)
        sys.exit(1)
