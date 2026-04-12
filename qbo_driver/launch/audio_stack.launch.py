from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare("qbo_driver")

    return LaunchDescription([
        Node(
            package="qbo_driver",
            executable="/home/qbo-v2/venvs/whisper/bin/python",
            name="qbo_listen",
            arguments=[
                "-m", "qbo_driver.listen_whisper",
                "--ros-args",
                "--params-file",
                PathJoinSubstitution([pkg_share, "config", "listen_whisper.yaml"]),
            ],
            output="screen",
        ),
        Node(
            package="qbo_driver",
            executable="qbo_tts_pico",
            name="talk_pico",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "qbo_tts_pico.yaml"]),
                {
                    "pronunciation_file": PathJoinSubstitution([
                        pkg_share, "config", "others", "pronunciation_map.json"
                    ]),
                },
            ],
            output="screen",
        ),
        Node(
            package="qbo_driver",
            executable="/home/qbo-v2/venvs/aiml/bin/python",
            name="qbo_aiml",
            arguments=[
                "-m", "qbo_driver.aiml",
            ],
            output="screen",
        ),
    ])
