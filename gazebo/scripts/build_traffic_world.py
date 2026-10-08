#!/usr/bin/env python3
"""Generate a readable three-lane display world from the SUMO vehicle definitions."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'sumo/scripts'))
from traffic_scene import road_length, vehicles


def visual(link, name, x, y, z, size, color):
    item = ET.SubElement(link, 'visual', name=name)
    ET.SubElement(item, 'pose').text = f'{x} {y} {z} 0 0 0'
    ET.SubElement(ET.SubElement(ET.SubElement(item, 'geometry'), 'box'), 'size').text = size
    material = ET.SubElement(item, 'material')
    ET.SubElement(material, 'ambient').text = color
    ET.SubElement(material, 'diffuse').text = color


def main():
    length = road_length(ROOT, 'traffic')
    center = length / 2
    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name='cosim_demo')
    ET.SubElement(world, 'gravity').text = '0 0 0'
    physics = ET.SubElement(world, 'physics', name='default', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = '0.01'
    ET.SubElement(physics, 'real_time_factor').text = '1'
    for plugin in ('Physics', 'UserCommands', 'SceneBroadcaster'):
        filename = {'Physics': 'physics', 'UserCommands': 'user-commands', 'SceneBroadcaster': 'scene-broadcaster'}[plugin]
        ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{plugin}')
    sensors = ET.SubElement(world, 'plugin', filename='gz-sim-sensors-system', name='gz::sim::systems::Sensors')
    ET.SubElement(sensors, 'render_engine').text = 'ogre2'
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'pose').text = '0 0 20 0 0 0'
    ET.SubElement(light, 'diffuse').text = '0.9 0.9 0.9 1'
    ET.SubElement(light, 'direction').text = '-0.3 -0.4 -1'
    model = ET.SubElement(world, 'model', name='road')
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='visuals')
    visual(link, 'grass', center, -4.8, -0.2, f'{length+100:g} 120 0.2', '0.2 0.4 0.2 1')
    visual(link, 'asphalt', center, -4.8, -0.05, f'{length:g} 9.6 0.1', '0.15 0.15 0.15 1')
    for y in (-0.15, -9.45):
        visual(link, f'edge_{y}', center, y, 0.01, f'{length:g} 0.16 0.02', '1 1 1 1')
    for y in (-3.2, -6.4):
        for x in range(0, int(length), 8):
            visual(link, f'dash_{y}_{x}', x+1.5, y, 0.01, '3 0.16 0.02', '1 1 1 1')
    for vehicle in vehicles(ROOT, 'traffic'):
        item = ET.SubElement(world, 'include')
        name = vehicle['id']
        ET.SubElement(item, 'uri').text = f'model://{name if name in ("car_a", "car_b") else "background_car"}'
        ET.SubElement(item, 'name').text = name
        y = -8.0 + 3.2*int(vehicle['departLane'])
        ET.SubElement(item, 'pose').text = f'{float(vehicle["departPos"])-2.5} {y} 0 0 0 0'
    ET.indent(sdf, space='  ')
    ET.ElementTree(sdf).write(ROOT / 'gazebo/worlds/traffic.sdf', encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    main()
