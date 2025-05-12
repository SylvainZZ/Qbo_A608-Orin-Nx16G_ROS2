from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    thresholds_file = os.path.join(
        get_package_share_directory('qbo_driver'),
        'thresholds.yaml'
    )
    diagnostic_aggregator_file = os.path.join(
        get_package_share_directory('qbo_driver'),
        'diagnostic_aggregator.yaml'
    )

    return LaunchDescription([
        Node(
            package='qbo_driver',
            executable='diag_node',
            name='diag_node',
            parameters=[{'thresholds_path': thresholds_file}],
            output='screen'
        ),
        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='aggregator_node',
            parameters=[{
                'analyzers': {
                        'path': 'Qbo',
                        'orin-nx-16g': {
                            'type': 'diagnostic_aggregator/GenericAnalyzer',
                            'path': 'orin-nx-16g',
                            'startswith': ['QBO Diagnostics [orin-nx-16g]'],
                            'remove_prefix': ['QBO Diagnostics [orin-nx-16g]']
                        }
                }
            }],
            output='screen'
        )
    ])
