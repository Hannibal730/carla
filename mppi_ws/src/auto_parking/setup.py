import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'auto_parking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/resource',
            ['resource/parking_Zone']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'docs'),
            glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dolbat',
    maintainer_email='dolbat@todo.todo',
    description='ROS 2 nodes for parking-zone scanning and automatic parking.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'zone_scan = auto_parking.zone_scan:main',
            'point_parking = auto_parking.point_parking:main',
            'parking = auto_parking.parking:main',
            # Compatibility with the command used before the node split.
            'parking_zone_visualizer = '
            'auto_parking.zone_scan:main',
        ],
    },
)
