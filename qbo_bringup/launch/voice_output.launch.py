#!/usr/bin/env python3
"""
QBO Voice Output — Text-to-Speech (TTS)

Lance le node de synthèse vocale PICO pour QBO.
Convertit le texte en parole pour les réponses du robot.

Node lancé :
  - qbo_tts_pico (PICO TTS engine)

Configuration :
  - qbo_tts_pico.yaml : paramètres TTS
  - pronunciation_map.json : dictionnaire de prononciation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    qbo_driver_dir = get_package_share_directory('qbo_driver')

    return LaunchDescription([

        # =====================================================================
        # QBO TTS PICO — Synthèse vocale
        # =====================================================================
        Node(
            package="qbo_driver",
            executable="qbo_tts_pico",
            name="qbo_tts_pico",
            output="screen",
            parameters=[
                PathJoinSubstitution([qbo_driver_dir, "config", "qbo_tts_pico.yaml"]),
                {
                    "pronunciation_file": PathJoinSubstitution([
                        qbo_driver_dir, "config", "others", "pronunciation_map.json"
                    ]),
                },
            ]
        ),

    ])
