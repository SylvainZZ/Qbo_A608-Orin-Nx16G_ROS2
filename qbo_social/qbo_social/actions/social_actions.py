#!/usr/bin/env python3
"""
Actions d'interaction sociale (salutations, reconnaissance, etc.).
"""

import sys
import os
import json
sys.path.append(os.path.dirname(__file__))

from base_action import BaseAction


class GreetPersonAction(BaseAction):
    """Salue une personne détectée."""

    intent_types = ["GREET_PERSON"]

    def __init__(self, node, display_client, tts_client):
        super().__init__(node)
        self.display_client = display_client
        self.tts_client = tts_client

    def execute(self, intent):
        try:
            payload = json.loads(intent.payload_json or "{}")
            person_name = intent.target_person_name or payload.get("name", "")

            # LED verte pour indiquer la salutation
            self.display_client.set_nose_color(4)  # Vert

            # Message de salutation
            if person_name:
                greeting = f"Bonjour {person_name} !"
            else:
                greeting = "Bonjour !"

            self.log_info(f"Greeting: {greeting}")
            self.tts_client.speak(greeting, "fr")

            return True

        except Exception as e:
            self.log_error(f"Failed to greet person: {e}")
            return False


class ExpressRecognitionAction(BaseAction):
    """Exprime la reconnaissance d'une personne connue."""

    intent_types = ["EXPRESS_RECOGNITION"]

    def __init__(self, node, display_client, tts_client):
        super().__init__(node)
        self.display_client = display_client
        self.tts_client = tts_client

    def execute(self, intent):
        try:
            payload = json.loads(intent.payload_json or "{}")
            person_name = intent.target_person_name or payload.get("name", "")

            if not person_name:
                self.log_warn("No person name for recognition")
                return False

            # LED jaune pour la reconnaissance
            self.display_client.set_nose_color(5)  # Jaune

            # Message de reconnaissance
            message = f"Je te reconnais, {person_name} !"

            self.log_info(f"Recognizing: {person_name}")
            self.tts_client.speak(message, "fr")

            return True

        except Exception as e:
            self.log_error(f"Failed to express recognition: {e}")
            return False


class ExpressEmotionAction(BaseAction):
    """Exprime une émotion via LED et/ou son."""

    intent_types = ["EXPRESS_EMOTION"]

    def __init__(self, node, display_client, tts_client):
        super().__init__(node)
        self.display_client = display_client
        self.tts_client = tts_client

    def execute(self, intent):
        try:
            payload = json.loads(intent.payload_json or "{}")
            emotion = payload.get("emotion", "happy")

            # Mapping émotion -> couleur LED
            emotion_colors = {
                "happy": 4,      # Vert
                "sad": 2,        # Bleu
                "angry": 1,      # Rouge
                "neutral": 7,    # Blanc
                "surprised": 5,  # Jaune
            }

            color = emotion_colors.get(emotion, 7)
            self.display_client.set_nose_color(color)

            self.log_info(f"Expressing emotion: {emotion}")
            return True

        except Exception as e:
            self.log_error(f"Failed to express emotion: {e}")
            return False
