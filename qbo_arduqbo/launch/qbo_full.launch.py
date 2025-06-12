from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    qboards_config = os.path.join(
        get_package_share_directory('qbo_arduqbo'),
        'config',
        'qboards_config.yaml'
    )
    dynamixel_config = os.path.join(
        get_package_share_directory('qbo_arduqbo'),
        'config',
        'dynamixel_config.yaml'
    )

    return LaunchDescription([
        # Node(
        #     package='qbo_arduqbo',
        #     executable='qbo_arduqbo',
        #     # name='qbo_arduqbo_node',
        #     parameters=[qboards_config]
        # ),
        Node(
            package='qbo_arduqbo',
            executable='qbo_dynamixel',
            # name='qbo_dynamixel_node',
            parameters=[dynamixel_config]
        )
    ])
