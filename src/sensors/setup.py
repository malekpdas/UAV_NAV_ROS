from setuptools import setup
import os
from glob import glob

package_name = 'sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 driver for BMX160 9DOF IMU',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'imu_bmx160 = {package_name}.imu_bmx160:main',
            f'imu_bno085 = {package_name}.imu_bno085:main',
            f'gps_zoe_m8q = {package_name}.gps_zoe_m8q:main',
            f'lidar_lite_v3hp = {package_name}.lidar_lite_v3hp:main',
        ],
    },
)
