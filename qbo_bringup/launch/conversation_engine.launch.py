#!/usr/bin/env python3
"""
QBO Conversation Engine — AIML Dialog Manager

Lance le moteur de conversation AIML pour QBO.
Gère les dialogues et la logique conversationnelle du robot.

Node lancé :
  - qbo_aiml (moteur AIML)

⚠️  Important : Ce node utilise un environnement virtuel Python dédié
    Chemin : /home/qbo-v2/venvs/aiml/bin/python
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # =====================================================================
        # QBO AIML — Moteur de conversation
        # =====================================================================
        Node(
            package="qbo_driver",
            executable="/home/qbo-v2/venvs/aiml/bin/python",
            name="qbo_aiml",
            output="screen",
            arguments=[
                "-m", "qbo_driver.aiml",
            ]
        ),

    ])
