from setuptools import find_packages, setup
import glob

package_name = 'qbo_driver'

setup(
    name=package_name,
    version='0.1.3',
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
        # Paramètres YAML du diagnostic Orin A608
        ('share/' + package_name+ '/config', ['config/OrinA608Diag_param.yaml']),
        # Fichiers JSON d'entraînement
        ('share/' + package_name + '/config/data_pairs',
            glob.glob('config/data_pairs/*.json')),
        # Dossier vide pour les futurs .faiss/.json générés
        ('share/' + package_name + '/config/LLM',
            glob.glob('config/LLM/*.faiss') + glob.glob('config/LLM/*.json')),
    ],
    install_requires=[
        "rclpy",
        "sounddevice",
        "webrtcvad",
        "faster-whisper",
        "numpy",
        "scipy",
        "qbo_msgs",
    ],
    zip_safe=True,
    maintainer='zwolinski',
    maintainer_email='sylvain-zwolinski@orange.fr',
    description='ROS 2 hardware diagnostics driver for the QBO platform based on Jetson Orin NX.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'qbo_listen = qbo_driver.listen_whisper:main',
            'qbo_talk = qbo_driver.talk_piper:main',
            'OrinA608Diag = qbo_driver.hardwareOrinA608:main',
            'qbo_aiml = qbo_driver.aiml:main',
            'qbo_diagnosticLogger = qbo_driver.diagnosticLog:main',
            'qbo_test = qbo_driver.test_joint_latency:main'
        ],
    },
)
