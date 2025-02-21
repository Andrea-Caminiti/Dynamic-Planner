from setuptools import setup
import os
from glob import glob

package_name = 'laserScanner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Laser scanner dummy publisher for ROS2 testing.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_broadcaster = laserScanner.static_tf_broadcaster:main',
            'laser_scanner = laserScanner.laserScanner:main',
        ],
    },
)
