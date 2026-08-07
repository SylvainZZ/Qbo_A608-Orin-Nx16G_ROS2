from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="ROS2 log level",
        ),
        Node(
            package="qbo_home_intent",
            executable="home_intent",
            name="home_intent",
            output="screen",
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
    ])
