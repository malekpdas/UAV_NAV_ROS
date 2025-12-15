from setuptools import setup
import os
from glob import glob

package_name = 'gps_zoe_m8q_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'smbus2'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='I2C driver for U-Blox ZOE-M8Q GPS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_zoe_m8q_node = gps_zoe_m8q_driver.gps_zoe_m8q_node:main',
        ],
    },
)
