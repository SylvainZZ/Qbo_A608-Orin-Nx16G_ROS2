#!/usr/bin/env python3
"""
Actions de gestion des profils de nœuds (lifecycle).
"""

import sys
import os
import json
sys.path.append(os.path.dirname(__file__))

from base_action import BaseAction


class StartProfileAction(BaseAction):
    """Démarre un profil de nœuds."""

    intent_types = ["START_PROFILE"]

    def __init__(self, node, bringup_client):
        super().__init__(node)
        self.bringup_client = bringup_client

    def execute(self, intent):
        try:
            payload = json.loads(intent.payload_json or "{}")
            profile = payload.get("profile", "MINIMAL")
            missing = payload.get("missing_nodes", [])

            self.log_info(
                f"Starting profile: {profile} (missing nodes: {missing})"
            )

            success = self.bringup_client.start_profile(
                profile, 
                callback=self._on_response
            )
            return success

        except Exception as e:
            self.log_error(f"Failed to start profile: {e}")
            return False

    def _on_response(self, future, profile, action):
        """Callback pour la réponse du service."""
        try:
            response = future.result()
            if response.success:
                self.log_info(
                    f"✓ Profile {profile} started: {response.message}"
                )
                self.log_info(f"  Active nodes: {response.active_nodes}")
            else:
                self.log_error(
                    f"✗ Profile {profile} start failed: {response.message}"
                )
        except Exception as e:
            self.log_error(f"Service call failed: {e}")


class StopProfileAction(BaseAction):
    """Arrête un profil de nœuds ou des nœuds spécifiques."""

    intent_types = ["STOP_PROFILE"]

    def __init__(self, node, bringup_client):
        super().__init__(node)
        self.bringup_client = bringup_client

    def execute(self, intent):
        try:
            payload = json.loads(intent.payload_json or "{}")
            profile = payload.get("profile", "MINIMAL")
            target_nodes = payload.get("nodes", [])

            self.log_info(
                f"Stopping profile: {profile} (target nodes: {target_nodes})"
            )

            success = self.bringup_client.stop_profile(
                profile,
                target_nodes,
                callback=self._on_response
            )
            return success

        except Exception as e:
            self.log_error(f"Failed to stop profile: {e}")
            return False

    def _on_response(self, future, profile, action):
        """Callback pour la réponse du service."""
        try:
            response = future.result()
            if response.success:
                self.log_info(
                    f"✓ Profile {profile} stopped: {response.message}"
                )
                self.log_info(f"  Active nodes: {response.active_nodes}")
            else:
                self.log_error(
                    f"✗ Profile {profile} stop failed: {response.message}"
                )
        except Exception as e:
            self.log_error(f"Service call failed: {e}")
