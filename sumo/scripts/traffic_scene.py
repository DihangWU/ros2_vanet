"""Small shared scene description; keep traffic definitions in the SUMO directory."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def scene_files(project_root, scenario):
    if scenario not in ('traffic', 'two_cars'):
        raise ValueError('scenario 必须为 traffic 或 two_cars')
    root = Path(project_root) / 'sumo'
    stem = 'traffic' if scenario == 'traffic' else 'straight'
    routes = root / 'routes' / ('traffic.rou.xml' if scenario == 'traffic' else 'two_cars.rou.xml')
    config = root / 'config' / ('traffic.sumocfg' if scenario == 'traffic' else 'demo.sumocfg')
    return root, stem, routes, config


def vehicles(project_root, scenario):
    _, _, routes, _ = scene_files(project_root, scenario)
    return [dict(vehicle.attrib) for vehicle in ET.parse(routes).getroot().findall('vehicle')]


def road_length(project_root, scenario):
    root, stem, _, _ = scene_files(project_root, scenario)
    nodes = ET.parse(root / 'network' / f'{stem}.nod.xml').getroot()
    start, end = nodes.find("node[@id='start']"), nodes.find("node[@id='end']")
    return float(end.get('x')) - float(start.get('x'))


def build_network(project_root, scenario):
    root, stem, _, _ = scene_files(project_root, scenario)
    target = root / 'network' / f'{stem}.net.xml'
    inputs = [root / 'network' / f'{stem}.nod.xml', root / 'network' / f'{stem}.edg.xml']
    if target.exists() and target.stat().st_mtime >= max(p.stat().st_mtime for p in inputs):
        return
    subprocess.run(['netconvert', '--node-files', str(inputs[0]), '--edge-files', str(inputs[1]),
                    '--output-file', str(target)], check=True)
