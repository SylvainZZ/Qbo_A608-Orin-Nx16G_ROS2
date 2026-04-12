#!/usr/bin/env python3
"""
QBO Profile: MINIMAL

Nœuds essentiels pour le fonctionnement de base :
  - qbo_arduqbo (System, Qboard_1, Qboard_3, dynamixel)
  - qbo_driver (Orin NX diagnostics)

Ce profil permet :
  ✓ Moteurs de base
  ✓ Batterie
  ✓ Servos tête
  ✓ Diagnostics système
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    qbo_driver_dir = get_package_share_directory('qbo_driver')
    qbo_arduqbo_dir = get_package_share_directory('qbo_arduqbo')

    return LaunchDescription([

        # =====================================================================
        # QBO DIAGNOSTICS — Orin NX monitoring
        # =====================================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(qbo_driver_dir, 'launch', 'orin_diagnostics.launch.py')
            )
        ),

        # =====================================================================
        # QBO FULL — ArduQbo + Dynamixel
        # =====================================================================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(qbo_arduqbo_dir, 'launch', 'base_hardware.launch.py')
            )
        ),

    ])
