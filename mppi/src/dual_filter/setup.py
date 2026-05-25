from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'dual_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hannibal',
    maintainer_email='cds730730@gmail.com',
    description='Dual EKF sensor fusion for autonomous driving (REP-105)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'utm_to_odometry = dual_filter.utm_to_odometry:main',
            'odom_path_publisher = dual_filter.odom_path_publisher:main',
        ],
    },
)
