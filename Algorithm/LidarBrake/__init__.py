"""Pure lidar following/braking algorithms; no ROS2, SUMO or Gazebo imports."""
from .config import Config
from .controller import LidarBrake

__all__ = ['Config', 'LidarBrake']
