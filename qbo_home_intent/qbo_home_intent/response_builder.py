"""Response builder: generates natural language spoken responses in French."""

from __future__ import annotations

import json


class ResponseBuilder:
    def build_query_response(
        self,
        entity_id: str,
        state: str,
        attributes_json: str,
        device_class: str,
    ) -> str:
        name = _friendly_name(entity_id)
        try:
            attrs = json.loads(attributes_json) if attributes_json else {}
        except (json.JSONDecodeError, ValueError):
            attrs = {}

        if device_class == "sensor":
            unit = attrs.get("unit_of_measurement", "")
            return f"{name} : {state} {unit}".strip() + "."

        if device_class == "binary_sensor":
            label = "actif" if state == "on" else "inactif"
            return f"{name} est {label}."

        if device_class == "climate":
            current = attrs.get("current_temperature", "?")
            setpoint = attrs.get("temperature", "?")
            return f"{name} : {current} °C mesurés, consigne {setpoint} °C."

        if device_class in ("light", "switch"):
            label = "allumé" if state == "on" else "éteint"
            return f"{name} est {label}."

        if device_class == "cover":
            pos = attrs.get("current_position")
            if pos is not None:
                return f"{name} est ouvert à {pos} %."
            opened = "ouvert" if state == "open" else "fermé"
            return f"{name} est {opened}."

        if device_class == "select":
            return f"{name} est en mode {state}."

        return f"{name} : {state}."

    def build_clarification_request(self, missing_slot: str, candidates: list[str]) -> str:
        prompts = {
            "area": "Dans quelle pièce ?",
            "device_class": "Quel type d'appareil ?",
            "name": "Lequel ?",
            "value": "Quelle valeur ?",
        }
        question = prompts.get(missing_slot, "Pouvez-vous préciser ?")
        if candidates:
            options = ", ".join(candidates[:5])
            return f"Je ne suis pas sûr. {question} ({options})"
        return question

    def build_confirmation_request(self, warning: str) -> str:
        return warning or "Confirmez-vous cette action ?"

    def build_error_response(self, error_code: str) -> str:
        messages = {
            "NOT_CONNECTED": "Je ne peux pas accéder à la maison pour l'instant.",
            "ENTITY_NOT_FOUND": "Je n'ai pas trouvé cet équipement.",
            "ACTION_NOT_SUPPORTED": "Cette action n'est pas supportée pour cet équipement.",
            "TIMEOUT": "Home Assistant n'a pas répondu à temps.",
            "REJECTED": "Je ne peux pas effectuer cette action.",
        }
        return messages.get(error_code, "Une erreur s'est produite.")

    def build_unknown_response(self) -> str:
        return "Je n'ai pas compris. Pouvez-vous reformuler ?"


def _friendly_name(entity_id: str) -> str:
    parts = entity_id.split(".", 1)
    raw = parts[-1] if len(parts) > 1 else entity_id
    return raw.replace("_", " ").capitalize()
