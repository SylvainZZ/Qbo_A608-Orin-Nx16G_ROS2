from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # Camera Gauche
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='left_camera',
            namespace='camera/left',
            parameters=[
                {'video_device': '/dev/video0'},
                {'image_size': [1280, 720]},
                {'pixel_format': 'YUYV'}
            ],
            remappings=[
                ('/image_raw', 'image_raw'),
                ('/camera_info', 'camera_info')
            ]
        ),

        Node(
            package='image_proc',
            executable='image_proc',
            name='left_proc',
            namespace='camera/left',
            remappings=[
                ('image_raw', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image', 'image')
            ]
        ),

        # Camera Droite
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='right_camera',
            namespace='camera/right',
            parameters=[
                {'video_device': '/dev/video1'},
                {'image_size': [1280, 720]},
                {'pixel_format': 'YUYV'}
            ],
            remappings=[
                ('/image_raw', 'image_raw'),
                ('/camera_info', 'camera_info')
            ]
        ),

        Node(
            package='image_proc',
            executable='image_proc',
            name='right_proc',
            namespace='camera/right',
            remappings=[
                ('image_raw', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image', 'image')
            ]
        )
    ])
