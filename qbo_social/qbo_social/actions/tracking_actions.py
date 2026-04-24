#!/usr/bin/env python3
"""
Actions de suivi de visage et recherche de personnes.
"""

import sys
import os
sys.path.append(os.path.dirname(__file__))

from base_action import BaseAction


class TrackFaceHeadOnlyAction(BaseAction):
    """Active le suivi de visage avec mouvements de tête uniquement."""

    intent_types = ["TRACK_FACE_HEAD_ONLY"]

    def __init__(self, node, follower_client):
        super().__init__(node)
        self.follower_client = follower_client

    def execute(self, intent):
        self.log_info("Activating head-only face tracking")
        success = self.follower_client.set_mode(
            enable_head=True,
            enable_base=False,
            callback=self._on_response
        )
        return success

    def _on_response(self, future, enable_head, enable_base):
        try:
            response = future.result()
            if response.success:
                self.log_info(f"✓ Head-only tracking enabled: {response.message}")
            else:
                self.log_warn(f"✗ Failed: {response.message}")
        except Exception as e:
            self.log_error(f"Service call failed: {e}")


class TrackFaceFullAction(BaseAction):
    """Active le suivi de visage complet (tête + base mobile)."""

    intent_types = ["TRACK_FACE_FULL"]

    def __init__(self, node, follower_client):
        super().__init__(node)
        self.follower_client = follower_client

    def execute(self, intent):
        self.log_info("Activating full face tracking (head + base)")
        success = self.follower_client.set_mode(
            enable_head=True,
            enable_base=True,
            callback=self._on_response
        )
        return success

    def _on_response(self, future, enable_head, enable_base):
        try:
            response = future.result()
            if response.success:
                self.log_info(f"✓ Full tracking enabled: {response.message}")
            else:
                self.log_warn(f"✗ Failed: {response.message}")
        except Exception as e:
            self.log_error(f"Service call failed: {e}")


class StopFaceTrackingAction(BaseAction):
    """Désactive complètement le suivi de visage."""

    intent_types = ["STOP_FACE_TRACKING"]

    def __init__(self, node, follower_client):
        super().__init__(node)
        self.follower_client = follower_client

    def execute(self, intent):
        self.log_info("Stopping face tracking")
        success = self.follower_client.set_mode(
            enable_head=False,
            enable_base=False,
            callback=self._on_response
        )
        return success

    def _on_response(self, future, enable_head, enable_base):
        try:
            response = future.result()
            if response.success:
                self.log_info(f"✓ Tracking stopped: {response.message}")
            else:
                self.log_warn(f"✗ Failed: {response.message}")
        except Exception as e:
            self.log_error(f"Service call failed: {e}")


class StartPersonSearchAction(BaseAction):
    """Active la recherche active de personnes (rotation + détection)."""

    intent_types = ["START_PERSON_SEARCH"]

    def __init__(self, node, follower_client):
        super().__init__(node)
        self.follower_client = follower_client

    def execute(self, intent):
        self.log_info("Starting person search mode")
        # Active rotation de base pour faciliter la détection
        success = self.follower_client.set_mode(
            enable_head=True,
            enable_base=True,
            callback=self._on_response
        )
        return success

    def _on_response(self, future, enable_head, enable_base):
        try:
            response = future.result()
            if response.success:
                self.log_info(f"✓ Search mode enabled: {response.message}")
            else:
                self.log_warn(f"✗ Failed: {response.message}")
        except Exception as e:
            self.log_error(f"Service call failed: {e}")


class StopPersonSearchAction(BaseAction):
    """Désactive la recherche de personnes."""

    intent_types = ["STOP_PERSON_SEARCH"]

    def __init__(self, node, follower_client):
        super().__init__(node)
        self.follower_client = follower_client

    def execute(self, intent):
        self.log_info("Stopping person search")
        success = self.follower_client.set_mode(
            enable_head=False,
            enable_base=False,
            callback=self._on_response
        )
        return success

    def _on_response(self, future, enable_head, enable_base):
        try:
            response = future.result()
            if response.success:
                self.log_info(f"✓ Search stopped: {response.message}")
            else:
                self.log_warn(f"✗ Failed: {response.message}")
        except Exception as e:
            self.log_error(f"Service call failed: {e}")
