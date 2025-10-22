from setuptools import setup
import os
from glob import glob

package_name = 'smart_patrol'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'waypoints'),
            glob('waypoints/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'),  # ← NUEVO
            glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='your_email@example.com',
    description='Sistema avanzado de patrullaje para Unitree GO2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_node = smart_patrol.patrol_node:main',
            'waypoint_recorder = smart_patrol.waypoint_recorder:main',  # ← NUEVO

        ],
    },
)
