from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('qbo_vision'),
        'config',
        'calibration',
        'default_camera_info.yaml'
    )

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 800,
                'image_height': 600,
                'pixel_format': 'yuyv',
                'framerate': 20.0,
                'camera_name': 'left_camera',
                'camera_info_url': f'file://{config_path}'
            }],
            remappings=[
                ('/image_raw', 'camera_left/image_raw'),
                ('/camera_info', 'camera_left/camera_info')
            ]
        )
    ])
