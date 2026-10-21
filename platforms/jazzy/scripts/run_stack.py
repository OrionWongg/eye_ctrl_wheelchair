#!/usr/bin/env python3
"""Own and clean up only processes launched for this simulation session."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--eye', action='store_true')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--scene', default='willow_office')
    parser.add_argument('--left', default='/dev/video4')
    parser.add_argument('--right', default='/dev/video2')
    parser.add_argument('--left-rotation', type=int, choices=[0,180], default=180)
    parser.add_argument('--right-rotation', type=int, choices=[0,180], default=0)
    parser.add_argument('--recalibrate', action='store_true')
    args = parser.parse_args()
    processes = []

    def start(command):
        processes.append(subprocess.Popen(command, cwd=ROOT, start_new_session=True))

    def shutdown(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, shutdown)
    try:
        start(['ros2', 'launch', 'wheelchair_sim', 'simulation.launch.py',
               'scene:=' + args.scene, 'gui:=' + str(not args.headless).lower(),
               'first_person:=true', 'first_person_view:=false'])
        console = [sys.executable, '-m', 'whill_runtime.console',
                   '--mode', 'eye' if args.eye else 'keyboard']
        if args.headless:
            console.append('--no-display')
        start(console)
        if args.eye:
            command = [str(ROOT / '.venv/bin/python'), '-m', 'whill_runtime.eye_tracker',
                       '--left', args.left, '--right', args.right,
                       '--left-rotation', str(args.left_rotation),
                       '--right-rotation', str(args.right_rotation), '--publish-ros']
            if args.recalibrate:
                command.append('--recalibrate')
            start(command)
        while all(p.poll() is None for p in processes):
            time.sleep(.2)
        return next((p.returncode for p in processes if p.returncode), 0)
    except KeyboardInterrupt:
        return 0
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        deadline = time.monotonic() + 8
        for p in processes:
            try:
                p.wait(timeout=max(.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()


if __name__ == '__main__':
    sys.exit(main())
