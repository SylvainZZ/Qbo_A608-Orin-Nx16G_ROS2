from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare("qbo_driver")

    tts_params = PathJoinSubstitution([pkg_share, "config", "talk_tts.yaml"])
    asr_params = PathJoinSubstitution([pkg_share, "config", "listen_whisper.yaml"])

    # Important: on passe les substitutions directement dans cmd, pas en str()
    tts_cmd = [
        "bash", "-lc",
        "source /opt/ros/humble/setup.bash; "
        "source /home/qbo-v2/venvs/tts/bin/activate; "
        "python -m qbo_driver.talk_TTS "
        "--ros-args --params-file " + "$0",
    ]

    asr_cmd = [
        "bash", "-lc",
        "source /opt/ros/humble/setup.bash; "
        "source /home/qbo-v2/venvs/whisper/bin/activate; "
        "python -m qbo_driver.listen_whisper "
        "--ros-args --params-file " + "$0",
    ]

    return LaunchDescription([
        ExecuteProcess(
            cmd=tts_cmd + [tts_params],
            output="screen",
        ),
        ExecuteProcess(
            cmd=asr_cmd + [asr_params],
            output="screen",
        ),
    ])
