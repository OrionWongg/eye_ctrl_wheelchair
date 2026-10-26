from setuptools import setup
import os
from glob import glob

package_name = 'wheelchair_hardware_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='whill',
    description='Hardware bridge: MID360 cloud→scan, Odin odom relay, Twist→key adapter',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scan_map = wheelchair_hardware_bridge.scan_map:main',
            'cloud_to_scan = wheelchair_hardware_bridge.cloud_to_scan:main',
            'odom_relay    = wheelchair_hardware_bridge.odom_relay:main',
            'twist_to_key  = wheelchair_hardware_bridge.twist_to_key:main',
        ],
    },
)
