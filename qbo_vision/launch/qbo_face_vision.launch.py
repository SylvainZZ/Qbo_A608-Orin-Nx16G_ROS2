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

    face_tracker_config = os.path.join(
        get_package_share_directory('qbo_vision'),
        'config',
        'face_tracker.yaml'
    )

    face_recognition_config = os.path.join(
        get_package_share_directory('qbo_vision'),
        'config',
        'face_recognition.yaml'
    )

    face_follower_config = os.path.join(
        get_package_share_directory('qbo_vision'),
        'config',
        'face_follower.yaml'
    )

    return LaunchDescription([
        # USB Camera Node with calibration
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/cam_usb',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'mjpeg2rgb',
                'io_method': 'mmap',
                'framerate': 30.0,
                'camera_name': 'left_camera',
                'camera_info_url': f'file://{config_path}',
                'queue_size': 1
            }],
            remappings=[
                ('/image_raw', 'camera_left/image_raw'),
                ('/camera_info', 'camera_left/camera_info')
            ]
        ),

        # Face Tracker Node
        Node(
            package='qbo_vision',
            executable='face_tracker_node',
            name='qbo_face_tracker',
            parameters=[
                face_tracker_config,
                {'publish_debug_image': False}, # Enable debug image for visualization
                {'start_enabled': True} # Start the node enabled
                ],
            output='screen'
        ),

        # Face Recognition Node
        Node(
            package='qbo_vision',
            executable='face_recognition_node',
            name='qbo_face_recognition',
            parameters=[face_recognition_config],
            output='screen'
        ),

        # Face Follower Node with move_base disabled
        Node(
            package='qbo_vision',
            executable='face_follower_node',
            name='qbo_face_following',
            parameters=[
                face_follower_config,
                {'base_rotation_enabled': False, 'head_movement_enabled': False}  # Override parameter to disable base and head movement
            ],
            output='screen'
        )
    ])
