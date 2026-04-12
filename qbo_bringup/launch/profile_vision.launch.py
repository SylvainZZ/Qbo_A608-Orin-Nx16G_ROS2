#!/usr/bin/env python3
"""
QBO Profile: VISION

Profil MINIMAL + capacités visuelles :
  - usb_cam (caméra USB avec calibration)
  - qbo_face_tracker (détection visages)
  - qbo_face_recognition (reconnaissance visages)
  - qbo_face_following (suivi visuel)

Ce profil permet :
  ✓ Tout MINIMAL
  ✓ Détection et reconnaissance de visages
  ✓ Suivi visuel (tête + base)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ===== Inclure le profil MINIMAL =====
    bringup_dir = get_package_share_directory('qbo_bringup')

    # ===== Configs YAML pour la vision =====
    camera_config = os.path.join(
        get_package_share_directory('qbo_vision'),
        'config',
        'calibration',
        'left_camera_640x480.yaml'
    )

    face_tracker_config = os.path.join(
        get_package_share_directory('qbo_vision'),
        'config',
        'face_tracker.yaml'
    )

    face_recognition_config = os.path.join(
        get_package_share_directory('qbo_vision'),
        'config',
        'face_recognition.yaml'
    )

    face_follower_config = os.path.join(
        get_package_share_directory('qbo_vision'),
        'config',
        'face_follower.yaml'
    )

    return LaunchDescription([

        # =====================================================================
        # USB CAMERA — Caméra avec calibration
        # =====================================================================
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                'video_device': '/dev/cam_usb',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'mjpeg2rgb',
                'io_method': 'mmap',
                'framerate': 30.0,
                'camera_name': 'left_camera',
                'camera_info_url': f'file://{camera_config}',
                'queue_size': 1
            }],
            remappings=[
                ('/image_raw', 'camera_left/image_raw'),
                ('/camera_info', 'camera_left/camera_info')
            ]
        ),

        # =====================================================================
        # QBO FACE TRACKER — Détection de visages (démarre 2s après la caméra)
        # =====================================================================
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='qbo_vision',
                    executable='face_tracker_node',
                    name='qbo_face_tracker',
                    output='screen',
                    parameters=[
                        face_tracker_config,
                        {'publish_debug_image': False},
                        {'start_enabled': True}
                    ]
                )
            ]
        ),

        # =====================================================================
        # QBO FACE RECOGNITION — Reconnaissance de visages (démarre 3s après)
        # =====================================================================
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='qbo_vision',
                    executable='face_recognition_node',
                    name='qbo_face_recognition',
                    output='screen',
                    parameters=[face_recognition_config]
                )
            ]
        ),

        # =====================================================================
        # QBO FACE FOLLOWING — Suivi visuel (démarre 4s après)
        # =====================================================================
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='qbo_vision',
                    executable='face_follower_node',
                    name='qbo_face_following',
                    output='screen',
                    parameters=[
                        face_follower_config  # Utilise UNIQUEMENT les valeurs du YAML
                    ]
                )
            ]
        ),

    ])
