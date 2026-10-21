from setuptools import setup

package_name = 'eye_turtle_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/eye_turtle.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'eye_turtle_controller = eye_turtle_control.controller:main',
            'camera_eye_tracker = eye_turtle_control.camera_tracker:main',
        ],
    },
)
