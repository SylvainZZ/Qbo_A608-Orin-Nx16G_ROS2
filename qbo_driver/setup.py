from setuptools import find_packages, setup

package_name = 'qbo_driver'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/diag_launch.py']),
        ('share/' + package_name, ['config/thresholds.yaml']),
        ('share/' + package_name, ['config/diagnostic_aggregator.yaml']),
        ('share/' + package_name, ['config/RS/qbo.rive']),
    ],
    install_requires=[
        "rclpy",
        "sounddevice",
        "webrtcvad",
        "faster-whisper",
        "numpy",
        "scipy",
        "rivescript",
        "qbo_msgs",
    ],
    zip_safe=True,
    maintainer='zwolinski',
    maintainer_email='sylvain-zwolinski@orange.fr',
    description='ROS 2 hardware diagnostics driver for the QBO platform based on Jetson Orin NX.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'diag_node = src.diag_node:main',
            'qbo_listen = src.listen_whisper:main',
            'qbo_talk = src.talk_piper:main',
            'qbo_brain = src.brainRS:main',
        ],
    },
)
