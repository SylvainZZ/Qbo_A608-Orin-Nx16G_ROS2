import re
from .constants import COLOR_KEYWORDS

COLOR_KEYWORDS = {
    "rouge": 1, "bleu": 2, "violet": 3, "vert": 4,
    "jaune": 5, "magenta": 6, "blanc": 7
}

CONFIRM_YES = ["oui", "ok", "d'accord", "vas-y", "go", "bien sûr", "bien sur"]
CONFIRM_NO  = ["non", "annule", "stop", "laisse", "pas maintenant"]

class ParameterExtractor:
    def __init__(self):
        pass

    def _is_confirm_yes(self, sentence: str) -> bool:
        s = sentence.strip().lower()
        if s.startswith("oui"):
            return True
        return any(w in s for w in CONFIRM_YES)

    def _is_confirm_no(self, sentence: str) -> bool:
        s = sentence.strip().lower()
        if s.startswith("non"):
            return True
        return any(w in s for w in CONFIRM_NO)

    def extract(self, sentence: str) -> dict:
        s = sentence.lower().strip()
        params = {}

        # ✅ Confirmation
        if self._is_confirm_yes(s):
            params["confirm"] = "yes"
            return params

        if self._is_confirm_no(s):
            params["confirm"] = "no"
            return params

        # 🎨 Couleurs
        for name, code in COLOR_KEYWORDS.items():
            if name in s:
                params["color"] = code
                params["color_name"] = name
                break

        # 🔢 Nombres
        numbers = re.findall(r"\d+", s)
        if numbers:
            params["number"] = int(numbers[0])

        return params