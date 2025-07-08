from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

# Lancer manuellement le service =>

def generate_launch_description():
    qbo_driver_dir = get_package_share_directory('qbo_driver')
    qbo_arduqbo_dir = get_package_share_directory('qbo_arduqbo')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(qbo_driver_dir, 'launch', 'qbo_diag.launch.py')
            )
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(qbo_arduqbo_dir, 'launch', 'qbo_full.launch.py')
        #     )
        # )
        # Node(
        #     package='qbo_bringup',
        #     executable='smach.py',
        #     name='smatch',
        #     output='screen',
        # ),
    ])
