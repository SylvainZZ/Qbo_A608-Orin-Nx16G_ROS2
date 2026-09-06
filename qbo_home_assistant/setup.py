from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qbo_home_assistant'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='Konex Inc',
    maintainer_email='contact@konexinc.fr',
    description='ROS2 bridge between the Qbo robot and Home Assistant via WebSocket.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ha_bridge = qbo_home_assistant.main:main',
        ],
    },
)
