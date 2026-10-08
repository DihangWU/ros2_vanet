#!/usr/bin/env python3
"""Start SUMO and OMNeT++; Veins owns the only TraCI connection."""
import argparse
import os
from pathlib import Path
import subprocess
import time


def stop(child):
    if child is None or child.poll() is not None:
        return
    child.terminate()
    try:
        child.wait(timeout=5)
    except subprocess.TimeoutExpired:
        child.kill()
        child.wait()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--qtenv', action='store_true')
    parser.add_argument('--scenario', choices=['traffic', 'two_cars'], default='traffic')
    parser.add_argument('--duration', type=float, default=30.0)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[2]
    home = Path('/home/maple')
    opp = Path(os.environ.get('OMNETPP_ROOT', home / 'omnetpp-6.1'))
    veins = Path(os.environ.get('VEINS_ROOT', home / 'veins-5.3.1'))
    inet = Path(os.environ.get('INET_ROOT', home / 'inet4.5'))
    veins_inet = veins / 'subprojects/veins_inet'
    (root / 'omnet/results').mkdir(exist_ok=True)
    import sys
    sys.path.insert(0, str(root / 'sumo/scripts'))
    from traffic_scene import build_network, scene_files
    build_network(root, args.scenario)
    config = scene_files(root, args.scenario)[3]
    subprocess.run(['python3', str(root / 'omnet/scripts/build_network.py')], check=True)

    # No --start: SUMO GUI waits for the user's Play button.
    sumo = subprocess.Popen([
        'sumo-gui' if args.gui else 'sumo', '-c',
        str(config), '--end', str(args.duration),
        '--remote-port', '9999', '--seed', '42', '--no-step-log', 'true',
    ])
    network = None
    try:
        time.sleep(0.5)
        ned = ';'.join(map(str, [
            root / 'omnet/ned', veins / 'src/veins',
            veins_inet / 'src/veins_inet', inet / 'src',
        ]))
        exclusions = ';'.join((inet / '.nedexclusions').read_text().split())
        command = [
            str(opp / 'bin/opp_run'), '-u', 'Qtenv' if args.qtenv else 'Cmdenv',
            '-n', ned, '-x', exclusions,
            '-l', str(inet / 'src/INET'),
            '-l', str(veins / 'src/veins'),
            '-l', str(veins_inet / 'src/veins_inet'),
            '-l', str(root / 'omnet/build/cosim_network'),
            '-f', 'omnetpp.ini',
            '--sim-time-limit=' + str(args.duration + 0.001) + 's',
            '--*.manager.duration=' + str(args.duration) + 's',
            '--*.manager.followSumoVehicles=' + ('true' if args.gui else 'false'),
            '--*.manager.ignoreGuiCommands=' + ('false' if args.gui else 'true'),
            '--*.node[*].wlan[0].radio.transmitter.power=' +
            os.environ.get('COSIM_RADIO_POWER', '20mW'),
        ]
        network = subprocess.Popen(command, cwd=root / 'omnet')
        network.wait()
        if network.returncode:
            raise SystemExit(network.returncode)
        # Keep the final SUMO GUI visible, just like Gazebo and RViz.
        if args.gui:
            sumo.wait()
    except KeyboardInterrupt:
        pass
    finally:
        stop(network)
        stop(sumo)


if __name__ == '__main__':
    main()
