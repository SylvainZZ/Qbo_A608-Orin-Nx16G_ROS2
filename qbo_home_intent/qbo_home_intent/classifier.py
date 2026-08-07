"""Intent classifier: maps normalized text to an intent type constant."""

from __future__ import annotations

import re
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

_CONFIG_DIR = Path(get_package_share_directory('qbo_home_intent')) / 'config'

# Patterns for query / status / cancel detection (accent-free)
_QUERY_RE = re.compile(
    r"\b(quelle|quel|combien|temperature|etat|est ce|lire|montre|affiche|donne|connais|sais)\b"
)
_STATUS_RE = re.compile(r"\b(statut|status|rapport|resume|bilan|recapitulatif)\b")
_CANCEL_RE = re.compile(r"\b(annule|annuler|stop|stoppe|arrete|laisse tomber|oublie|rien)\b")


class IntentClassifier:
    INTENT_UNKNOWN = 0
    INTENT_CONTROL = 1
    INTENT_QUERY = 2
    INTENT_STATUS = 3
    INTENT_CANCEL = 4

    def __init__(self, actions_path: Path | None = None) -> None:
        path = actions_path or (_CONFIG_DIR / "actions.yaml")
        with open(path, encoding="utf-8") as fh:
            data = yaml.safe_load(fh)

        self._control_verbs: set[str] = set()
        self._query_verbs: set[str] = set()
        for action_data in data.get("control", {}).values():
            self._control_verbs.update(action_data.get("verbs", []))
        for action_data in data.get("query", {}).values():
            self._query_verbs.update(action_data.get("verbs", []))

    def classify(self, normalized_text: str) -> int:
        if _CANCEL_RE.search(normalized_text):
            return self.INTENT_CANCEL
        if _STATUS_RE.search(normalized_text):
            return self.INTENT_STATUS
        if _QUERY_RE.search(normalized_text):
            return self.INTENT_QUERY
        words = set(normalized_text.split())
        if words & self._query_verbs:
            return self.INTENT_QUERY
        if words & self._control_verbs:
            return self.INTENT_CONTROL
        # Fallback: multi-word verb phrases
        for verb in sorted(self._query_verbs, key=len, reverse=True):
            if verb in normalized_text:
                return self.INTENT_QUERY
        for verb in sorted(self._control_verbs, key=len, reverse=True):
            if verb in normalized_text:
                return self.INTENT_CONTROL
        return self.INTENT_UNKNOWN
