from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # QBoards Config
    qboards_config = os.path.join(
        get_package_share_directory('qbo_arduqbo'),
        'config',
        'qboards_config.yaml'
    )
    # Dynamixel Config
    dynamixel_config = os.path.join(
        get_package_share_directory('qbo_arduqbo'),
        'config',
        'dynamixel_config.yaml'
    )

    return LaunchDescription([

       # Qbo ArduQbo Node
        Node(
            package='qbo_arduqbo',
            executable='qbo_arduqbo',
            parameters=[qboards_config]
        ),

        # Qbo Dynamixel Node
        Node(
            package='qbo_arduqbo',
            executable='qbo_dynamixel',
            parameters=[dynamixel_config]
        ),
    ])
