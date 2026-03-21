from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    nav2_bringup_dir = os.path.join(
        '/opt/ros/humble/share/nav2_bringup/launch'
    )

    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=['/cmd_vel', '/qbo_arduqbo/base_ctrl/cmd_vel'],
        output='screen'
    )

    return LaunchDescription([

        cmd_vel_relay,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                os.getenv('HOME'),
                'qbo_ws/src/qbo_navigation/config/nav2_params.yaml'
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file
            }.items()
        )

    ])