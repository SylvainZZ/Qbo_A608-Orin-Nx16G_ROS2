#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node

from qbo_msgs.msg import SocialEvent
from qbo_social.adapters.diagnostics_adapter import DiagnosticsAdapter
from qbo_social.adapters.diagnostics_inspector import DiagnosticsInspector
from qbo_social.adapters.face_adapter import FaceAdapter


class SocialEventAdapter(Node):

    def __init__(self):
        super().__init__('qbo_social_event_adapter')

        # ===== Params =====
        # --- Face tracking ---
        # Temps avant considérer un visage comme stable (augmenté pour comportement intelligent)
        self.declare_parameter("face_stable_time", 3.0)
        # Temps avant considérer un visage comme perdu
        self.declare_parameter("face_lost_timeout", 2.0)
        # Confidence minimum pour reconnaissance
        self.declare_parameter("min_confidence", 0.6)

        # --- Diagnostics ---
        # Délai de stabilité avant publication d'un diagnostic
        self.declare_parameter("diagnostics_stability_delay", 3.0)
        # Timeout avant considérer un nœud comme manquant
        self.declare_parameter("diagnostics_node_timeout", 10.0)
        # Fréquence de vérification du watchdog
        self.declare_parameter("diagnostics_check_frequency", 2.0)

        # ===== ROS Publishers =====
        self.pub_event = self.create_publisher(
            SocialEvent,
            '/qbo_social/events',
            10
        )

        self.get_logger().info("Publisher created: /qbo_social/events")

        # ===== Adapters =====
        self.get_logger().info("Initializing adapters...")

        # DiagnosticsAdapter avec paramètre de stabilité
        stability_delay = self.get_parameter("diagnostics_stability_delay").value
        self.diagnostics_adapter = DiagnosticsAdapter(self, stability_delay=stability_delay)
        self.get_logger().info(f"  ✓ DiagnosticsAdapter initialized (stability_delay={stability_delay}s)")

        # DiagnosticsInspector avec paramètres de monitoring
        node_timeout = self.get_parameter("diagnostics_node_timeout").value
        check_frequency = self.get_parameter("diagnostics_check_frequency").value
        self.diagnostics_inspector = DiagnosticsInspector(
            self,
            timeout_s=node_timeout,
            check_hz=check_frequency
        )
        self.get_logger().info(
            f"  ✓ DiagnosticsInspector initialized (timeout={node_timeout}s, freq={check_frequency}Hz)"
        )

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
        # rclpy.shutdown()