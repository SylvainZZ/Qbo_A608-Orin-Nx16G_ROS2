from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'qbo_social'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(include=['qbo_social', 'qbo_social.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zwolinski',
    maintainer_email='sylvain-zwolinski@orange.fr',
    description=(
        'Social behavior orchestration package for Qbo. '
        'Provides event normalization, world state management, '
        'behavior decision, action coordination, and system mode handling '
        'for social interaction in ROS 2.'
    ),
    license='BSD-3-Clause',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'event_adapter = qbo_social.event_adapter:main',
            'world_model = qbo_social.world_model:main',
            'behavior_engine = qbo_social.behavior_engine:main',
            'action_executor = qbo_social.action_executor:main',
            'system_mode_manager = qbo_social.system_mode_manager:main',
            'sbe_follower_client_example = qbo_social.sbe_follower_client_example:main',
            'debug_behavior_state = qbo_social.debug_behavior_state:main',
        ],
    },
)
