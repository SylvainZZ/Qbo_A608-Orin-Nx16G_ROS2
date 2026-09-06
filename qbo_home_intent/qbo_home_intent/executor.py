"""Executor: sends commands to Home Assistant via ha_bridge ROS2 services."""

from __future__ import annotations

import json
import random
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from qbo_ha_interfaces.srv import CallHaService, GetEntityState

_HA_DOMAIN_MAP: dict[str, str] = {
    "light": "light",
    "cover": "cover",
    "switch": "switch",
    "climate": "climate",
    "select": "select",
}

# Maps internal action names to HA service names
_ACTION_TO_HA_SERVICE: dict[str, str] = {
    "turn_on": "turn_on",
    "turn_off": "turn_off",
    "open": "open_cover",
    "close": "close_cover",
    "stop": "stop_cover",
    "set_position": "set_cover_position",
    "set_brightness": "turn_on",
    "set_temperature": "set_temperature",
    "set_color": "turn_on",
    "select_option": "select_option",
}

# Maps action name to the HA service_data key for its primary parameter
_ACTION_PARAM_KEY: dict[str, str] = {
    "set_brightness": "brightness_pct",
    "set_position": "position",
    "set_temperature": "temperature",
    "set_color": "color_name",
    "select_option": "option",
}

_SPOKEN_VARIANTS: dict[str, list[str]] = {
    "turn_on": [
        "J'ai allumé {article} {name}.",
        "C'est fait, {article} {name} est allumé.",
        "Voilà, {article} {name} est maintenant allumé.",
    ],
    "turn_off": [
        "J'ai éteint {article} {name}.",
        "C'est fait, {article} {name} est éteint.",
        "{article} {name} est maintenant éteint.",
    ],
    "open": [
        "J'ai ouvert {article} {name}.",
        "C'est fait, {article} {name} est ouvert.",
        "Voilà, {article} {name} est ouvert.",
    ],
    "close": [
        "J'ai fermé {article} {name}.",
        "C'est fait, {article} {name} est fermé.",
        "{article} {name} est maintenant fermé.",
    ],
    "stop": [
        "J'ai arrêté {article} {name}.",
        "{article} {name} est arrêté.",
    ],
    "set_temperature": [
        "Consigne réglée à {temperature} degrés.",
        "D'accord, je règle à {temperature} degrés.",
        "Température fixée à {temperature} °C.",
    ],
    "set_brightness": [
        "Luminosité de {article} {name} réglée à {brightness_pct} %.",
        "D'accord, {brightness_pct} % pour {article} {name}.",
    ],
    "set_position": [
        "{article} {name} positionné à {position} %.",
        "Position réglée à {position} %.",
    ],
    "select_option": [
        "Mode {option} activé pour {article} {name}.",
        "D'accord, mode {option} sélectionné.",
    ],
}

# Article par device_class
_ARTICLE: dict[str, str] = {
    "light": "la",
    "cover": "le",
    "climate": "le",
    "switch": "le",
    "select": "le",
}


class HomeExecutor:
    _TIMEOUT_S = 15.0
    _CONNECT_TIMEOUT_S = 2.0

    def __init__(self, node: Node) -> None:
        self._node = node
        # ReentrantCallbackGroup allows response callbacks to run while the
        # service handler thread is blocked in the polling loop below.
        _cbg = ReentrantCallbackGroup()
        self._call_cli = node.create_client(CallHaService, "/ha_bridge/call_service", callback_group=_cbg)
        self._state_cli = node.create_client(GetEntityState, "/ha_bridge/get_entity_state", callback_group=_cbg)

    def execute(
        self,
        action: str,
        entity_id: str,
        device_class: str,
        parameters: dict[str, str],
        dry_run: bool = False,
        label: str = "",
    ) -> tuple[bool, str, str]:
        """Return (success, spoken_response, technical_message)."""
        # Query actions read entity state instead of calling a HA service
        if action in ("read_temperature", "read_state", "read_power", "read_energy"):
            return self._execute_read(action, entity_id, label=label, device_class=device_class)

        ha_service = _ACTION_TO_HA_SERVICE.get(action)
        if not ha_service:
            return False, "Je ne sais pas comment exécuter cette action.", f"No HA service for '{action}'"

        domain = _HA_DOMAIN_MAP.get(device_class, device_class)
        service_data = self._build_service_data(action, parameters)

        if dry_run:
            technical = f"[DRY RUN] {domain}.{ha_service}({entity_id}, {service_data})"
            return True, "Simulation réussie.", technical

        if not self._call_cli.wait_for_service(timeout_sec=self._CONNECT_TIMEOUT_S):
            return False, "Je ne peux pas accéder à la maison pour l'instant.", "NOT_CONNECTED"

        req = CallHaService.Request()
        req.domain = domain
        req.service = ha_service
        req.entity_id = entity_id
        req.service_data = json.dumps(service_data) if service_data else ""

        self._node.get_logger().info(
            f"→ ha_bridge  domain={domain}  service={ha_service}"
            f"  entity_id={entity_id!r}  data={req.service_data!r}"
        )

        future = self._call_cli.call_async(req)
        if not self._wait(future):
            return False, "Pas de réponse de Home Assistant.", "TIMEOUT"

        resp = future.result()
        if resp.success:
            spoken = self._render_spoken(action, label, device_class, parameters)
            return True, spoken, resp.result
        return False, "L'action a échoué.", resp.error_message

    def _execute_read(self, action: str, entity_id: str, label: str = "", device_class: str = "") -> tuple[bool, str, str]:
        """Handle query actions by reading entity state from HA."""
        import json as _json
        ok, state, attrs_raw = self.read_state(entity_id)
        if not ok:
            return False, "Je ne peux pas lire l'état de cet appareil.", attrs_raw

        try:
            attrs = _json.loads(attrs_raw) if attrs_raw else {}
        except (ValueError, TypeError):
            attrs = {}

        unit = attrs.get("unit_of_measurement", "")
        name = label.lower() if label else (attrs.get("friendly_name") or entity_id.split(".")[-1].replace("_", " "))

        # Convert Wh to kWh for readability
        display_state = state
        display_unit = unit
        if unit == "Wh":
            try:
                kwh = float(state) / 1000
                display_state = f"{kwh:.1f}"
                display_unit = "kWh"
            except (ValueError, TypeError):
                pass

        value_str = f"{display_state} {display_unit}".strip()

        _read_variants: dict[str, list[str]] = {
            "read_temperature": [
                f"La {name} est de {value_str}.",
                f"Il fait {value_str} ({name}).",
                f"{name.capitalize()} : {value_str}.",
            ],
            "read_power": [
                f"La {name} est de {value_str}.",
                f"{name.capitalize()} : {value_str}.",
            ],
            "read_energy": [
                f"La {name} est de {value_str}.",
                f"{name.capitalize()} : {value_str}.",
            ],
        }

        if action == "read_state":
            variants = self._build_read_state_variants(name, state, attrs, device_class)
        else:
            variants = _read_variants.get(action, [f"{name.capitalize()} : {value_str}."])
        return True, random.choice(variants), f"state={state} unit={unit}"

    @staticmethod
    def _build_read_state_variants(
        name: str, state: str, attrs: dict, device_class: str
    ) -> list[str]:
        # Translations for raw HA states
        _STATE_FR: dict[str, str] = {
            "on": "allumé", "off": "éteint",
            "open": "ouvert", "closed": "fermé", "closing": "en cours de fermeture", "opening": "en cours d'ouverture",
            "unavailable": "indisponible", "unknown": "inconnu",
            "heat": "en chauffe", "cool": "en refroidissement", "auto": "automatique",
            "idle": "en veille",
        }

        if device_class == "cover":
            pos = attrs.get("current_position")
            if pos is not None:
                ouvert = state == "open"
                etat = "ouvert" if ouvert else "fermé"
                return [
                    f"Le {name} est {etat} à {pos} %.",
                    f"{name.capitalize()} : {etat}, position {pos} %.",
                    f"Oui, le {name} est {etat} ({pos} %)." if ouvert else f"Non, le {name} est {etat} ({pos} %).",
                ]
            etat = _STATE_FR.get(state, state)
            return [
                f"Le {name} est {etat}.",
                f"{name.capitalize()} : {etat}.",
            ]

        if device_class in ("light", "switch"):
            etat = _STATE_FR.get(state, state)
            allume = state == "on"
            return [
                f"La {name} est {etat}.",
                f"{'Oui' if allume else 'Non'}, la {name} est {etat}.",
                f"{name.capitalize()} : {etat}.",
            ]

        if device_class == "climate":
            current = attrs.get("current_temperature", "?")
            setpoint = attrs.get("temperature", "?")
            return [
                f"Le {name} : {current} °C mesurés, consigne {setpoint} °C.",
                f"Il fait {current} °C, consigne réglée à {setpoint} °C.",
            ]

        if device_class == "select":
            return [
                f"Le {name} est en mode {state}.",
                f"Mode actuel du {name} : {state}.",
            ]

        if device_class == "battery":
            unit = attrs.get("unit_of_measurement", "%")
            return [
                f"La batterie du {name} est à {state} {unit}.",
                f"{name.capitalize()} : {state} {unit} de charge.",
            ]

        etat = _STATE_FR.get(state, state)
        return [f"{name.capitalize()} : {etat}."]

    def read_state(self, entity_id: str) -> tuple[bool, str, str]:
        """Return (success, state_value, attributes_json)."""
        if not self._state_cli.wait_for_service(timeout_sec=self._CONNECT_TIMEOUT_S):
            return False, "", "NOT_CONNECTED"

        req = GetEntityState.Request()
        req.entity_id = entity_id

        future = self._state_cli.call_async(req)
        if not self._wait(future):
            return False, "", "TIMEOUT"

        resp = future.result()
        if resp.success:
            return True, resp.entity_state.state, resp.entity_state.attributes
        return False, "", resp.error_message

    def _wait(self, future) -> bool:
        """Poll until future done; the MultiThreadedExecutor handles response dispatch."""
        deadline = time.monotonic() + self._TIMEOUT_S
        while not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.01)
        return True

    @staticmethod
    def _build_service_data(action: str, parameters: dict[str, str]) -> dict:
        data: dict = {}
        param_key = _ACTION_PARAM_KEY.get(action)
        if param_key:
            raw = parameters.get(param_key) or parameters.get("value")
            if raw is not None:
                try:
                    data[param_key] = float(raw)
                except ValueError:
                    data[param_key] = raw
        return data

    @staticmethod
    def _render_spoken(action: str, label: str, device_class: str, parameters: dict[str, str]) -> str:
        variants = _SPOKEN_VARIANTS.get(action, ["Action effectuée."])
        template = random.choice(variants)
        article = _ARTICLE.get(device_class, "le")
        name = label.lower() if label else "l'appareil"
        ctx = dict(parameters)
        ctx["name"] = name
        ctx["article"] = article
        try:
            return template.format(**ctx)
        except KeyError:
            return f"C'est fait."
