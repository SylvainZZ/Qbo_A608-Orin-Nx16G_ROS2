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
            executable="/home/qbo-v2/venvs/tts/bin/python",
            name="qbo_talk",
            arguments=[
                "-m", "qbo_driver.talk_TTS",
                "--ros-args",
                "--params-file",
                PathJoinSubstitution([pkg_share, "config", "talk_tts.yaml"]),
            ],
            output="screen",
        ),
    ])
