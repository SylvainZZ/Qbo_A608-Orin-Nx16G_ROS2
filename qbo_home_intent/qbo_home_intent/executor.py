"""Executor: sends commands to Home Assistant via ha_bridge ROS2 services."""

from __future__ import annotations

import json
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

_SPOKEN_TEMPLATES: dict[str, str] = {
    "turn_on": "J'ai allumé {name}.",
    "turn_off": "J'ai éteint {name}.",
    "open": "J'ai ouvert {name}.",
    "close": "J'ai fermé {name}.",
    "stop": "J'ai arrêté {name}.",
    "set_temperature": "Consigne réglée à {temperature} degrés.",
    "set_brightness": "Luminosité de {name} réglée à {brightness_pct} %.",
    "set_position": "{name} positionné à {position} %.",
    "select_option": "Mode {option} activé pour {name}.",
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
    ) -> tuple[bool, str, str]:
        """Return (success, spoken_response, technical_message)."""
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
            name = entity_id.split(".")[-1].replace("_", " ")
            spoken = self._render_spoken(action, name, parameters)
            return True, spoken, resp.result
        return False, "L'action a échoué.", resp.error_message

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
    def _render_spoken(action: str, name: str, parameters: dict[str, str]) -> str:
        template = _SPOKEN_TEMPLATES.get(action, "Action effectuée.")
        ctx = dict(parameters)
        ctx["name"] = name
        try:
            return template.format(**ctx)
        except KeyError:
            return f"Action {action} effectuée sur {name}."
