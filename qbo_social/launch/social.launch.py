import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Chemin vers le package qbo_social
    pkg_qbo_social = get_package_share_directory('qbo_social')

    # Chemin par défaut vers le fichier de paramètres
    default_params_file = os.path.join(pkg_qbo_social, 'config', 'qbo_social_params.yaml')

    # Déclaration de l'argument pour permettre de surcharger le fichier de paramètres
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the ROS2 parameters file for qbo_social nodes'
    )

    # Configuration du fichier de paramètres
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        params_file_arg,

        Node(
            package='qbo_social',
            executable='event_adapter',
            name='qbo_social_event_adapter',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='qbo_social',
            executable='world_model',
            name='qbo_social_world_model',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='qbo_social',
            executable='behavior_engine',
            name='qbo_social_behavior_engine',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='qbo_social',
            executable='action_executor',
            name='qbo_social_action_executor',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='qbo_social',
            executable='system_mode_manager',
            name='qbo_system_mode_manager',
            output='screen',
            parameters=[params_file]
        ),
    ])