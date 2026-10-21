from setuptools import setup
from glob import glob
package_name = 'eye_shared_control'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/config', glob('config/*.yaml')),
                  ('share/' + package_name + '/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='whill', maintainer_email='whill@example.com',
      description='Eye joystick intent state machine for shared wheelchair simulation.',
      license='Apache-2.0',
      entry_points={'console_scripts': ['eye_intent = eye_shared_control.node:main',
                                       'mock_eye = eye_shared_control.mock:main']})
