#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node

from qbo_msgs.msg import SocialEvent
from qbo_social.adapters.diagnostics_adapter import DiagnosticsAdapter
from qbo_social.adapters.face_adapter import FaceAdapter


class SocialEventAdapter(Node):

    def __init__(self):
        super().__init__('qbo_social_event_adapter')

        # ===== Params =====
        # Temps avant considérer un visage comme stable (augmenté pour comportement intelligent)
        self.declare_parameter("face_stable_time", 3.0)
        # Temps avant considérer un visage comme perdu
        self.declare_parameter("face_lost_timeout", 2.0)
        # Confidence minimum pour reconnaissance
        self.declare_parameter("min_confidence", 0.6)

        # ===== ROS Publishers =====
        self.pub_event = self.create_publisher(
            SocialEvent,
            '/qbo_social/events',
            10
        )

        self.get_logger().info("Publisher created: /qbo_social/events")

        # ===== Adapters =====
        self.get_logger().info("Initializing adapters...")
        self.diagnostics_adapter = DiagnosticsAdapter(self)
        self.get_logger().info("  ✓ DiagnosticsAdapter initialized")

        self.face_adapter = FaceAdapter(self)
        self.get_logger().info("  ✓ FaceAdapter initialized")

        self.get_logger().info("SocialEventAdapter started (with FaceAdapter and DiagnosticsAdapter)")

    # =========================
    # EVENT PUBLISHER HELPER
    # =========================
    def _create_event_msg(self, event_type, source, active=True, payload=None):
        """Create a SocialEvent message with specified parameters."""
        msg = SocialEvent()
        now = self.get_clock().now().to_msg()

        # Header complet
        msg.header.stamp = now
        msg.header.frame_id = 'qbo_social'

        # Timestamp
        msg.stamp = now

        # Données de base
        msg.event_type = event_type
        msg.source = source
        msg.person_id = ""
        msg.person_name = ""
        msg.confidence = 0.0
        msg.payload_json = json.dumps(payload) if payload else "{}"

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = SocialEventAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()