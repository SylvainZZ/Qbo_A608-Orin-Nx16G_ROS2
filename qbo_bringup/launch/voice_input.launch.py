#!/usr/bin/env python3
"""
QBO Voice Input — Whisper STT (Speech-to-Text)

Lance le système de reconnaissance vocale Whisper pour QBO.
Capture et transcrit la parole en texte.

Node lancé :
  - qbo_listen (Whisper STT)

⚠️  Important : Ce node utilise un environnement virtuel Python dédié
    Chemin : /home/qbo-v2/venvs/whisper/bin/python
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    qbo_driver_dir = FindPackageShare("qbo_driver")

    return LaunchDescription([

        # =====================================================================
        # QBO LISTEN — Reconnaissance vocale Whisper
        # =====================================================================
        Node(
            package="qbo_driver",
            executable="/home/qbo-v2/venvs/whisper/bin/python",
            name="qbo_listen",
            output="screen",
            arguments=[
                "-m", "qbo_driver.listen_whisper",
                "--ros-args",
                "--params-file",
                PathJoinSubstitution([qbo_driver_dir, "config", "listen_whisper.yaml"]),
            ]
        ),

    ])
