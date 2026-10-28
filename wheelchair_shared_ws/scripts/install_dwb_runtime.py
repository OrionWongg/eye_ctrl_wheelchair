#!/usr/bin/env python3
"""Install verified DWB preview dependencies in a separate local prefix."""
from pathlib import Path
import install_nav2_runtime as installer

if __name__ == '__main__':
    installer.ROOT = Path(__file__).resolve().parents[1] / 'vendor/dwb'
    installer.main(['ros-humble-nav2-controller', 'ros-humble-dwb-core',
                    'ros-humble-dwb-plugins', 'ros-humble-dwb-critics'])
