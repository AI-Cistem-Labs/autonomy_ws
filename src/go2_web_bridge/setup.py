from setuptools import setup
import os
from glob import glob

package_name = 'go2_web_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 👇 Incluye todos los archivos dentro de la carpeta web/
        (os.path.join('share', package_name, 'web'), glob('go2_web_bridge/web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@example.com',
    description='WebSocket + REST bridge for Unitree Go2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go2_web_bridge = go2_web_bridge.go2_web_bridge:main'
        ],
    },
)

