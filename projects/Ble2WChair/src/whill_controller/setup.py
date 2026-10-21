import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'whill_controller'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yongying-Zhu',
    maintainer_email='Yongying-Zhu@users.noreply.github.com',
    description='ROS 2 integration package for the WHILL ESP32 controller.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = whill_controller.controller_node:main',
            'calibration_logger = '
            'whill_controller.calibration_logger:main',
        ],
    },
)
