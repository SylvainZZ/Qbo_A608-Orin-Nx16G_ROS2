#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class SystemModeManager(Node):
    def __init__(self):
        super().__init__('qbo_system_mode_manager')
        self.get_logger().info('SystemModeManager started')


def main(args=None):
    rclpy.init(args=args)
    node = SystemModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()