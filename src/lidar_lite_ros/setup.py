from setuptools import setup
import os
from glob import glob

package_name = 'lidar_lite_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='I2C driver for Garmin Lidar Lite v3HP',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_lite_node = lidar_lite_ros.lidar_lite_node:main',
        ],
    },
)
