from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory('qbo_vision'),
        'config',
        'calibration',
        'left_camera_640x480.yaml'
    )

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'mjpeg2rgb',
                'io_method': 'mmap',
                'framerate': 60.0,
                'camera_name': 'left_camera',
                'camera_info_url': f'file://{config_path}',
                'queue_size': 1
            }],
            remappings=[
                ('/image_raw', 'camera_left/image_raw'),
                ('/camera_info', 'camera_left/camera_info')
            ]
        )
    ])