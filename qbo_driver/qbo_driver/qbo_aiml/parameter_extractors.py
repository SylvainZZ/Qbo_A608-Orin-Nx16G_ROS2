import re
from .constants import COLOR_KEYWORDS


class ParameterExtractor:

    def __init__(self):
        pass

    def extract(self, sentence: str) -> dict:
        sentence = sentence.lower()
        params = {}

        # Couleurs
        for name, code in COLOR_KEYWORDS.items():
            if name in sentence:
                params["color"] = code
                params["color_name"] = name
                break

        # Oui / Non
        if any(word in sentence for word in ["oui", "d'accord", "ok"]):
            params["confirmation"] = True

        if any(word in sentence for word in ["non", "annule", "stop"]):
            params["confirmation"] = False

        # Valeurs numériques
        numbers = re.findall(r"\d+", sentence)
        if numbers:
            params["number"] = int(numbers[0])

        return params

