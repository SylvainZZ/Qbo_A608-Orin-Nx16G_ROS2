from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "ha_ws_url",
            default_value="ws://192.168.1.56:8123/api/websocket",
            description="Home Assistant WebSocket URL",
        ),
        DeclareLaunchArgument(
            "ha_token",
            default_value="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiI5MzRjZTYwM2E4Yjg0MjI2YWE4Yzg2NGYwYTFiZDdmOSIsImlhdCI6MTc4NjAxNzMxNywiZXhwIjoyMTAxMzc3MzE3fQ.2mblY8P9E2TYAQioeYd1VQ8OCzulgc0oiWcW3J5Jo2A",
            description="HA long-lived access token (required)",
        ),
        DeclareLaunchArgument(
            "reconnect_delay_s",
            default_value="5.0",
            description="Seconds between reconnection attempts",
        ),
        DeclareLaunchArgument(
            "all_states_period_s",
            default_value="0.0",
            description="Periodic full-snapshot interval in seconds (0 = disabled)",
        ),
        Node(
            package="qbo_home_assistant",
            executable="ha_bridge",
            name="ha_bridge",
            output="screen",
            parameters=[{
                "ha_ws_url": LaunchConfiguration("ha_ws_url"),
                "ha_token": LaunchConfiguration("ha_token"),
                "reconnect_delay_s": LaunchConfiguration("reconnect_delay_s"),
                "all_states_period_s": LaunchConfiguration("all_states_period_s"),
            }],
        ),
    ])
