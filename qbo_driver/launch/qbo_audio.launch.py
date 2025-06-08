from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Déclaration des arguments personnalisables
    return LaunchDescription([
        DeclareLaunchArgument("audio_in", default_value="default", description="Nom du périphérique micro (entrée)"),
        DeclareLaunchArgument("audio_out", default_value="default", description="Nom du périphérique haut-parleur (sortie)"),
        DeclareLaunchArgument("lang", default_value="fr", description="Langue système initiale"),
        DeclareLaunchArgument("volume", default_value="70", description="Volume initial du playback"),
        DeclareLaunchArgument("mute_micro", default_value="true", description="Mute micro pendant lecture TTS"),

        # Node TTS (talk)
        Node(
            package="qbo_driver",
            executable="qbo_talk",
            name="qbo_talk",
            output="screen",
            parameters=[
                {"audio_out_device_name": LaunchConfiguration("audio_out")},
                {"audio_in_device_name": LaunchConfiguration("audio_in")},
                {"audio_playback_volume": LaunchConfiguration("volume")},
                {"default_lang": LaunchConfiguration("lang")},
                {"mute_micro_during_playback": LaunchConfiguration("mute_micro")},
            ],
        ),

        # Node ASR (listen)
        Node(
            package="qbo_driver",
            executable="qbo_listen",
            name="qbo_listen",
            output="screen",
            parameters=[
                {"audio_in_device_name": LaunchConfiguration("audio_in")},
                {"default_lang": LaunchConfiguration("lang")},
            ],
        ),
    ])
