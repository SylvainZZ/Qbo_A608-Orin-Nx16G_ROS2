#!/usr/bin/env python3
"""
Client encapsulant le service de face following.
"""

from qbo_msgs.srv import SetFollowerStatus


class FollowerClient:
    """Encapsule les appels au service de face tracking."""

    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()

        self.service = node.create_client(
            SetFollowerStatus,
            '/qbo_face_following/set_follower_status'
        )

        self.logger.info("Waiting for face follower service...")
        self.service.wait_for_service(timeout_sec=5.0)

    def set_mode(self, enable_head: bool, enable_base: bool, callback=None):
        """
        Configure le mode de suivi du visage.

        Args:
            enable_head: Active les mouvements de tête (pan/tilt)
            enable_base: Active la rotation de la base mobile
            callback: Fonction appelée avec la réponse (optionnel)
        """
        if not self.service.service_is_ready():
            self.logger.warn("Face follower service not available")
            return False

        request = SetFollowerStatus.Request()
        request.enable_head_movement = enable_head
        request.enable_base_rotation = enable_base

        future = self.service.call_async(request)

        if callback:
            future.add_done_callback(
                lambda f: callback(f, enable_head, enable_base)
            )

        return True
