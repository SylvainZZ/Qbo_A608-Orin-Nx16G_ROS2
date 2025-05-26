from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gscam2',
            executable='gscam_main',
            name='gscam_right',
            output='screen',
            parameters=[
                {
                    'gscam_config': 'nvarguscamerasrc sensor-id=1 ! '
                                    'video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1, format=NV12 ! '
                                    'nvvidconv ! video/x-raw, format=BGRx ! '
                                    'videoconvert ! video/x-raw, format=BGR ! appsink'
                },
                {'camera_name': 'right_camera'},
                {'frame_id': 'camera_right_optical_frame'}
            ],
            remappings=[
                ('/image_raw', '/camera/right/image_raw'),
                ('/camera_info', '/camera/right/camera_info')
            ]
        )
    ])
