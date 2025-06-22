from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('qbo_driver'),
        'config',
        'OrinA608Diag_param.yaml'
    )

    return LaunchDescription([
        Node(
            package='qbo_driver',
            executable='OrinA608Diag',
            name='OrinA608Diag_node',
            output='screen',
            parameters=[param_file]
        ),
    ])
