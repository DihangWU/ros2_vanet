from glob import glob
from setuptools import setup

setup(
    name='cosim_bridge', version='0.1.0', packages=['cosim_bridge'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cosim_bridge']),
        ('share/cosim_bridge', ['package.xml']),
        ('share/cosim_bridge/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='CoSimDemo learner', maintainer_email='learner@example.com',
    description='SUMO to ROS2 state bridge', license='Apache-2.0',
    entry_points={'console_scripts': ['sumo_bridge = cosim_bridge.sumo_bridge:main']},
)
