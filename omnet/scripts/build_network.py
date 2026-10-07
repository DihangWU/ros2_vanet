#!/usr/bin/env python3
"""Build the project adapter without changing third-party installations."""
import os
from pathlib import Path
import subprocess


def main():
    root = Path(__file__).resolve().parents[2]
    home = Path('/home/maple')
    opp = Path(os.environ.get('OMNETPP_ROOT', home / 'omnetpp-6.1'))
    veins = Path(os.environ.get('VEINS_ROOT', home / 'veins-5.3.1'))
    inet = Path(os.environ.get('INET_ROOT', home / 'inet4.5'))
    veins_inet = veins / 'subprojects/veins_inet'
    build = root / 'omnet/build'
    build.mkdir(exist_ok=True)
    target = build / 'libcosim_network.so'
    source = root / 'veins/src/CoSimNetwork.cc'
    message = root / 'veins/src/EmergencyWarning.msg'
    if target.exists() and target.stat().st_mtime >= max(
            source.stat().st_mtime, message.stat().st_mtime):
        print('Network adapter is up to date')
        return

    subprocess.run([
        str(opp / 'bin/opp_msgc'), '--msg6', '-I' + str(inet / 'src'),
        message.name,
    ], cwd=message.parent, check=True)
    command = [
        'clang++', '-std=c++17', '-O2', '-fPIC', '-shared',
        str(source), str(message.with_name('EmergencyWarning_m.cc')),
        '-o', str(target),
    ]
    for path in [opp / 'include', veins / 'src', inet / 'src', veins_inet / 'src']:
        command += ['-I', str(path)]
    for path in [opp / 'lib', veins / 'src', inet / 'src', veins_inet / 'src']:
        command += ['-L', str(path), '-Wl,-rpath,' + str(path)]
    command += ['-lveins_inet', '-lveins', '-lINET', '-loppsim', '-loppcommon']
    subprocess.run(command, check=True)
    print('Built', target)


if __name__ == '__main__':
    main()
