from setuptools import setup
import os
from glob import glob

package_name = 'rc_control_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rc_control.launch.py']),
        ('share/' + package_name + '/config', ['config/rc_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='RC Receiver Control Package for reading GPIO signals',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rc_reader_node = rc_control_ros.rc_reader_node:main',
        ],
    },
)
