from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qbo_arduqbo',
            executable='qbo_arduqbo',
            # name='qbo_arduqbo_node',
            parameters=['src/qbo_arduqbo/config/qboards_config.yaml']
        ),
        Node(
            package='qbo_arduqbo',
            executable='qbo_dynamixel',
            # name='qbo_dynamixel_node',
            parameters=['src/qbo_arduqbo/config/dynamixel_config.yaml']
        )
    ])
