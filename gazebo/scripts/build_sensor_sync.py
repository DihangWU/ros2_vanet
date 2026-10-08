"""Build the small render-event adapter with the installed Harmonic SDK."""
from pathlib import Path
import subprocess


def build(root):
    source = root/'gazebo/plugins'
    output = root/'gazebo/build'
    library = output/'libcosim_sensor_render_sync.so'
    if library.exists() and library.stat().st_mtime >= max(
            p.stat().st_mtime for p in source.iterdir() if p.suffix in ('.cc', '.txt')):
        return library
    subprocess.run(['cmake', '-S', str(source), '-B', str(output)], check=True)
    subprocess.run(['cmake', '--build', str(output), '-j2'], check=True)
    return library


if __name__ == '__main__':
    build(Path(__file__).resolve().parents[2])
