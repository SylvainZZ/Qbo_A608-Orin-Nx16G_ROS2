from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera_d435',
        namespace='camera_d435',
        output='screen',

        parameters=[{

            # 'base_frame_id': 'camera_link',
            # 'camera_name': 'camera_d435',

            # 'depth_module.profile': '640x480x30',
            # 'rgb_camera.color_profile': '640x480x30',
            # 'depth_module.profile': '424x240x30',
            # 'rgb_camera.color_profile': '424x240x30',
            'depth_module.profile': '424x240x15',
            'rgb_camera.color_profile': '424x240x15',

            'align_depth.enable': True,
            'pointcloud.enable': True,

            'pointcloud.allow_no_texture_points': True,

            'spatial_filter.enable': True,
            'spatial_filter.smooth_alpha': 0.5,
            'spatial_filter.smooth_delta': 20,

            'temporal_filter.enable': False,
            'hole_filling_filter.enable': False,
            'decimation_filter.enable': True,
            'decimation_filter.filter_magnitude': 2,

            'pointcloud.allow_no_texture_points': False,

            'depth_module.emitter_enabled': 1,
            'depth_module.laser_power': 150.0,

            'publish_tf': True,
            'tf_publish_rate': 30.0,
            'use_sim_time': False

        }]
    )
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        output='screen',
        arguments=[
            '--x', '0.11',
            '--y', '0.0',
            '--z', '0.30',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ]
    )
    laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depth_to_scan',
        output='screen',

        parameters=[{
            'scan_time': 0.066,  # ~15 Hz
            'range_min': 0.2,
            'range_max': 3.0,

            'output_frame': 'camera_link',

            'scan_height': 10,   # nombre de lignes utilisées (important)
        }],

        remappings=[
            ('depth', '/camera_d435/camera_d435/aligned_depth_to_color/image_raw'),
            ('depth_camera_info', '/camera_d435/camera_d435/aligned_depth_to_color/camera_info'),
            ('scan', '/scan')
        ]
    )


    return LaunchDescription([
        realsense_node,
        camera_tf,
        laserscan_node

    ])