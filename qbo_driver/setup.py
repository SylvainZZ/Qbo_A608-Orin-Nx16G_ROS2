from setuptools import find_packages, setup

package_name = 'qbo_driver'

setup(
    name=package_name,
    version='0.1.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/qbo_diag.launch.py',
            'launch/qbo_audio.launch.py',
            'launch/gscam_imx219_launch.py',
            'launch/dual_camera_v412_launch.py'
        ]),
        ('share/' + package_name, [
            'config/thresholds.yaml',
            'config/diagnostic_aggregator.yaml',
            'config/RS/qbo.rive'
        ]),
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
            'diag_node = qbo_driver.diag_node:main',
            'qbo_listen = qbo_driver.listen_whisper:main',
            'qbo_talk = qbo_driver.talk_piper:main',
            'qbo_brain = qbo_driver.brainRS:main',
            'test_dxl_node = qbo_driver.pan_tilt_tester_node:main',
            'OrinA608Diag = qbo_driver.hardwareOrinA608:main',
        ],
    },
)
