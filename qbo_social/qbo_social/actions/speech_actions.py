#!/usr/bin/env python3
"""
Actions de synthèse vocale et parole.
"""

import sys
import os
import json
sys.path.append(os.path.dirname(__file__))

from base_action import BaseAction


class SayShortPhraseAction(BaseAction):
    """Prononce une phrase courte."""

    intent_types = ["SAY_SHORT_PHRASE"]

    def __init__(self, node, tts_client):
        super().__init__(node)
        self.tts_client = tts_client

    def execute(self, intent):
        try:
            payload = json.loads(intent.payload_json or "{}")
            text = payload.get("text", "")
            language = payload.get("language", "fr")

            if not text:
                self.log_warn("No text to speak")
                return False

            self.log_info(f"Speaking: {text}")
            success = self.tts_client.speak(text, language)
            return success

        except Exception as e:
            self.log_error(f"Failed to speak: {e}")
            return False


class SpeakTextAction(BaseAction):
    """Action générique pour faire parler le robot."""

    intent_types = ["SPEAK_TEXT"]

    def __init__(self, node, tts_client):
        super().__init__(node)
        self.tts_client = tts_client

    def execute(self, intent):
        try:
            payload = json.loads(intent.payload_json or "{}")
            text = payload.get("text", "")
            language = payload.get("language", "fr")

            if not text:
                self.log_warn("No text to speak")
                return False

            self.log_info(f"TTS: {text}")
            success = self.tts_client.speak(text, language)
            return success

        except Exception as e:
            self.log_error(f"Failed to speak: {e}")
            return False


class StopSpeakingAction(BaseAction):
    """Interrompt la parole en cours."""

    intent_types = ["STOP_SPEAKING"]

    def __init__(self, node, tts_client):
        super().__init__(node)
        self.tts_client = tts_client

    def execute(self, intent):
        try:
            self.log_info("Stopping speech")
            self.tts_client.stop_speaking()
            return True

        except Exception as e:
            self.log_error(f"Failed to stop speaking: {e}")
            return False
