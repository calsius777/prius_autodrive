from setuptools import setup
from glob import glob
import os

package_name = 'prius_sensor_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name), ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Prius Project Team',
    maintainer_email='maintainer@example.com',
    description='Sensor bringup package for Prius autonomous vehicle project',
    license='Apache-2.0',
    tests_require=['pytest'],
)
