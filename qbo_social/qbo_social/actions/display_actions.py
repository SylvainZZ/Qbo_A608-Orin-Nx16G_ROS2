#!/usr/bin/env python3
"""
Actions d'affichage visuel (LED, LCD, matrice LED).
"""

import sys
import os
import json
sys.path.append(os.path.dirname(__file__))

from base_action import BaseAction


class SetNoseColorAction(BaseAction):
    """Change la couleur du nez LED."""

    intent_types = ["SET_NOSE_COLOR"]

    def __init__(self, node, display_client):
        super().__init__(node)
        self.display_client = display_client

    def execute(self, intent):
        try:
            payload = json.loads(intent.payload_json or "{}")
            color = payload.get("color", 4)  # Vert par défaut
            duration = payload.get("duration", 2.0)  # 2 secondes par défaut

            self.log_info(f"Setting nose color to {color} for {duration}s")
            self.display_client.set_nose_color(color, duration=duration)
            return True

        except Exception as e:
            self.log_error(f"Failed to set nose color: {e}")
            return False


class ShowSmileAction(BaseAction):
    """Affiche un sourire sur la matrice LED de la bouche."""

    intent_types = ["SHOW_SMILE"]

    def __init__(self, node, display_client):
        super().__init__(node)
        self.display_client = display_client

    def execute(self, intent):
        try:
            payload = json.loads(intent.payload_json or "{}")
            variant = payload.get("variant", "normal")  # normal, big, wink
            duration = payload.get("duration", 2.0)  # 2 secondes par défaut

            self.log_info(f"Showing {variant} smile for {duration}s")
            self.display_client.show_smile(variant=variant, duration=duration)
            return True

        except Exception as e:
            self.log_error(f"Failed to show smile: {e}")
            return False


class DisplayTextAction(BaseAction):
    """Affiche du texte sur l'écran LCD."""

    intent_types = ["DISPLAY_TEXT"]

    def __init__(self, node, display_client):
        super().__init__(node)
        self.display_client = display_client

    def execute(self, intent):
        try:
            payload = json.loads(intent.payload_json or "{}")
            line1 = payload.get("line1", "")
            line2 = payload.get("line2", "")

            self.log_info(f"Displaying text: {line1} / {line2}")
            self.display_client.display_text(line1, line2)
            return True

        except Exception as e:
            self.log_error(f"Failed to display text: {e}")
            return False


class ClearDisplayAction(BaseAction):
    """Efface l'affichage LCD."""

    intent_types = ["CLEAR_DISPLAY"]

    def __init__(self, node, display_client):
        super().__init__(node)
        self.display_client = display_client

    def execute(self, intent):
        try:
            self.log_info("Clearing display")
            self.display_client.clear_display()
            return True

        except Exception as e:
            self.log_error(f"Failed to clear display: {e}")
            return False
