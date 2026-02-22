import json
import os
import random


class SpeechGenerator:

    def __init__(self, phrases_file):

        self.phrases = {}

        if os.path.exists(phrases_file):
            with open(phrases_file, "r", encoding="utf-8") as f:
                self.phrases = json.load(f)

    def generate(self, event_key, severity="info", context=None):

        if event_key not in self.phrases:
            return None

        options = self.phrases[event_key]

        # Si structure hiérarchique par severity
        if isinstance(options, dict):
            options = options.get(severity, [])

        if not options:
            return None

        sentence = random.choice(options)

        # Remplacement contextuel futur possible
        if context:
            for k, v in context.items():
                sentence = sentence.replace(f"{{{k}}}", str(v))

        return sentence
