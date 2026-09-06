from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qbo_home_intent'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=['qbo_home_intent', 'qbo_home_intent.*'], exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'knowledge'), glob('knowledge/*')),
        (os.path.join('share', package_name, 'tests'), glob('tests/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zwolinski',
    maintainer_email='sylvain-zwolinski@orange.fr',
    description='Natural language home intent parser and executor for Qbo robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'home_intent = qbo_home_intent.main:main',
            'ha_entities_sync = qbo_home_intent.ha_entities_sync:main',
            'runner_tests = qbo_home_intent.runner_tests:main',
        ],
    },
)
