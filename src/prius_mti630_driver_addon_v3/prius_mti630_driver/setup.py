from glob import glob
from setuptools import setup

package_name = 'prius_mti630_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
        ('share/' + package_name + '/docker', glob('docker/*')),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Prius Project',
    maintainer_email='user@example.com',
    description='Pure Python Xbus MTData2 ROS 2 driver for Movella/Xsens MTi-630 AHRS.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mti630_node = prius_mti630_driver.mti630_node:main',
            'xbus_sniffer = prius_mti630_driver.xbus_sniffer:main',
            'xbus_diag = prius_mti630_driver.xbus_diag:main',
        ],
    },
)
