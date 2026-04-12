#!/usr/bin/env python3
"""
QBO Profile: NAVIGATION

Profil MINIMAL + capacités de navigation autonome :
  - qbo_navigation (stack navigation ROS 2)
  - qbo_lidar (capteur LIDAR)
  - qbo_slam (cartographie et localisation)

Ce profil permet :
  ✓ Tout MINIMAL
  ✓ Navigation autonome
  ✓ Cartographie SLAM
  ✓ Évitement d'obstacles

NOTE: Ce profil est un template. Adaptez selon votre configuration de navigation.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Inclure le profil MINIMAL
    bringup_dir = get_package_share_directory('qbo_bringup')
    minimal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'profile_minimal.launch.py')
        )
    )

    return LaunchDescription([

        # =====================================================================
        # INCLURE PROFIL MINIMAL
        # =====================================================================
        minimal_launch,

        # =====================================================================
        # QBO LIDAR — Capteur LIDAR (exemple RPLidar)
        # =====================================================================
        # Node(
        #     package='rplidar_ros',
        #     executable='rplidar_node',
        #     name='rplidar',
        #     output='screen',
        #     parameters=[{
        #         'serial_port': '/dev/ttyUSB2',
        #         'frame_id': 'laser_frame',
        #     }]
        # ),

        # =====================================================================
        # QBO NAVIGATION — Stack Nav2 (à implémenter)
        # =====================================================================
        # (Inclure ici votre launch file Nav2 ou SLAM)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(bringup_dir, 'launch', 'nav2_bringup.launch.py')
        #     )
        # ),

        # =====================================================================
        # PLACEHOLDER NODE (à remplacer par vos nodes de navigation)
        # =====================================================================
        Node(
            package='qbo_navigation',
            executable='qbo_navigation_stub',
            name='qbo_navigation',
            output='screen',
            parameters=[{
                'placeholder': True,
            }]
        ),

    ])
