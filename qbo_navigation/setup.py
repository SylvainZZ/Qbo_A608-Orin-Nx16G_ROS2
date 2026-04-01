from setuptools import setup
import os
from glob import glob

package_name = 'qbo_navigation'

setup(
    name=package_name,
    version='0.1.1',
    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='zwolinski',
    maintainer_email='sylvain-zwolinski@orange.fr',

    description='Qbo navigation package with Nav2 and D435',
    license='Apache License 2.0',

    # tests_require=['pytest'],

    entry_points={
        'console_scripts': [
        ],
    },
)