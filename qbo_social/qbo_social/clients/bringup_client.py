#!/usr/bin/env python3
"""
Client encapsulant le service de gestion des profils de nœuds.
"""

from qbo_msgs.srv import ManageProfile


class BringupClient:
    """Encapsule les appels au service de bringup manager."""

    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()

        self.service = node.create_client(
            ManageProfile,
            '/qbo_bringup/manage_profile'
        )

        self.logger.info("Waiting for bringup manager service...")
        self.service.wait_for_service(timeout_sec=5.0)

    def start_profile(self, profile_name: str, callback=None):
        """
        Démarre un profil de nœuds.

        Args:
            profile_name: Nom du profil (ex: "MINIMAL", "FULL", "NAVIGATION")
            callback: Fonction appelée avec la réponse (optionnel)
        """
        return self._call_service(profile_name, "START", [], callback)

    def stop_profile(self, profile_name: str, target_nodes=None, callback=None):
        """
        Arrête un profil ou des nœuds spécifiques.

        Args:
            profile_name: Nom du profil
            target_nodes: Liste de nœuds à arrêter (None = tout le profil)
            callback: Fonction appelée avec la réponse (optionnel)
        """
        return self._call_service(
            profile_name,
            "STOP",
            target_nodes or [],
            callback
        )

    def _call_service(self, profile_name: str, action: str, target_nodes: list, callback):
        """Appel générique au service."""
        if not self.service.service_is_ready():
            self.logger.error("Bringup manager service not available!")
            return False

        request = ManageProfile.Request()
        request.profile_name = profile_name
        request.action = action
        request.target_nodes = target_nodes

        future = self.service.call_async(request)

        if callback:
            future.add_done_callback(
                lambda f: callback(f, profile_name, action)
            )

        self.logger.info(f"Requesting {action} for profile {profile_name}...")
        return True
