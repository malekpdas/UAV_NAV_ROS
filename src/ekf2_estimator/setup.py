from setuptools import setup
import os
from glob import glob

package_name = 'ekf2_estimator'

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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='PX4 EKF2-grade estimator for ROS 2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_node = ekf2_estimator.ekf_node:main',
            'plot_orientation = ekf2_estimator.plot_orientation:main',
        ],
    },
)
