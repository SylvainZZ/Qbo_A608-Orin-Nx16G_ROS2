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
            smile_variant = payload.get("smile", "happy")  # Par défaut: grand sourire
            duration = payload.get("duration", 3.0)  # 3 secondes

            # 🎨 Animation: Nez vert + Sourire sur la bouche (temporisés)
            self.display_client.set_nose_color(4, duration=duration)  # Vert
            self.display_client.show_smile(variant=smile_variant, duration=duration)

            # 🔊 Message de salutation
            if person_name:
                greeting = f"Bonjour {person_name} !"
            else:
                greeting = "Bonjour !"

            self.log_info(f"Greeting: {greeting} (smile={smile_variant})")
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
            smile_variant = payload.get("smile", "wink")  # Par défaut: clin d'œil
            duration = payload.get("duration", 2.5)

            if not person_name:
                self.log_warn("No person name for recognition")
                return False

            # 🎨 Animation: Nez jaune + Clin d'œil sur la bouche (temporisés)
            self.display_client.set_nose_color(5, duration=duration)  # Jaune
            self.display_client.show_smile(variant=smile_variant, duration=duration)

            # 🔊 Message de reconnaissance
            message = f"Je te reconnais, {person_name} !"

            self.log_info(f"Recognizing: {person_name} (smile={smile_variant})")
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
            duration = payload.get("duration", 2.5)
            speak = payload.get("speak", True)  # Parler par défaut

            # 🎨 Mapping émotion -> couleur LED nez + animation bouche + texte
            emotion_config = {
                "happy": {
                    "color": 4,       # Vert
                    "mouth": "happy",
                    "text": "Je suis content !"
                },
                "sad": {
                    "color": 2,       # Bleu
                    "mouth": "sad",
                    "text": "Je suis triste"
                },
                "angry": {
                    "color": 1,       # Rouge
                    "mouth": "angry",
                    "text": "Ça m'énerve !"
                },
                "neutral": {
                    "color": 7,       # Blanc
                    "mouth": "neutral",
                    "text": "Je suis neutre"
                },
                "surprised": {
                    "color": 5,       # Jaune
                    "mouth": "surprised",
                    "text": "Oh ! Quelle surprise !"
                },
                "worried": {
                    "color": 3,       # Violet
                    "mouth": "worried",
                    "text": "Je suis inquiet"
                },
                "excited": {
                    "color": 6,       # Cyan
                    "mouth": "excited",
                    "text": "C'est génial !"
                },
                "thinking": {
                    "color": 7,       # Blanc
                    "mouth": "thinking",
                    "text": "Laisse-moi réfléchir"
                },
                "sleepy": {
                    "color": 2,       # Bleu
                    "mouth": "sleepy",
                    "text": "Je suis fatigué"
                }
            }

            config = emotion_config.get(emotion, emotion_config["neutral"])

            # Appliquer l'animation visuelle (nez + bouche temporisés)
            self.display_client.set_nose_color(config["color"], duration=duration)
            self.display_client.show_smile(variant=config["mouth"], duration=duration)

            # Parler si demandé
            if speak:
                # Utiliser texte personnalisé si fourni, sinon texte par défaut
                text = payload.get("text", config["text"])
                self.tts_client.speak(text, "fr")
                self.log_info(f"Expressing emotion: {emotion} (mouth={config['mouth']}, text={text})")
            else:
                self.log_info(f"Expressing emotion: {emotion} (mouth={config['mouth']}, silent)")

            return True

        except Exception as e:
            self.log_error(f"Failed to express emotion: {e}")
            return False
