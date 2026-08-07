"""Slot extractor: action, area, device_class, entity_id and numeric parameters."""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

_CONFIG_DIR = Path(get_package_share_directory('qbo_home_intent')) / 'config'

# Matches integers or decimals optionally followed by a unit
_NUMBER_RE = re.compile(
    r"\b(\d+(?:[.,]\d+)?)\s*(pourcents?|%|degres?|°c|watts?|w\b|lux|litres?|l\b)?"
)


@dataclass
class Slots:
    action: str = ""
    area: str = ""
    device_class: str = ""
    name: str = ""
    entity_id: str = ""
    parameters: dict[str, str] = field(default_factory=dict)


class SlotExtractor:
    def __init__(
        self,
        actions_path: Path | None = None,
        areas_path: Path | None = None,
        aliases_path: Path | None = None,
    ) -> None:
        actions_path = actions_path or (_CONFIG_DIR / "actions.yaml")
        areas_path = areas_path or (_CONFIG_DIR / "areas.yaml")
        aliases_path = aliases_path or (_CONFIG_DIR / "aliases.yaml")

        with open(actions_path, encoding="utf-8") as fh:
            self._actions_data = yaml.safe_load(fh)
        with open(areas_path, encoding="utf-8") as fh:
            self._areas_data = yaml.safe_load(fh).get("areas", {})
        with open(aliases_path, encoding="utf-8") as fh:
            alias_data = yaml.safe_load(fh)

        self._device_aliases = alias_data.get("devices", {})
        self._sensor_aliases = alias_data.get("sensors", {})

        # verb → (category, action_name) — sorted longest-first for greedy match
        self._action_map: dict[str, tuple[str, str]] = {}
        for category in ("control", "query"):
            for action_name, data in self._actions_data.get(category, {}).items():
                for verb in data.get("verbs", []):
                    self._action_map[verb] = (category, action_name)

        # area alias → area_id
        self._area_map: dict[str, str] = {}
        for area_id, data in self._areas_data.items():
            self._area_map[area_id] = area_id
            for alias in data.get("aliases", []):
                self._area_map[alias] = area_id

        # device alias → (dev_id, device_class)
        self._device_map: dict[str, tuple[str, str]] = {}
        for dev_id, data in self._device_aliases.items():
            for alias in data.get("aliases", []):
                self._device_map[alias] = (dev_id, data.get("device_class", ""))
            if data.get("entity_id"):
                self._device_map[dev_id] = (dev_id, data.get("device_class", "switch"))

        # sensor alias → entity_id
        self._sensor_entity_map: dict[str, str] = {}
        for sensor_data in self._sensor_aliases.values():
            for alias in sensor_data.get("aliases", []):
                self._sensor_entity_map[alias] = sensor_data.get("entity_id", "")

    def extract(self, normalized_text: str) -> Slots:
        slots = Slots()

        # --- Action: try phrases of decreasing length (up to 6 words) ---
        words = normalized_text.split()
        for length in range(min(6, len(words)), 0, -1):
            for start in range(len(words) - length + 1):
                phrase = " ".join(words[start : start + length])
                if phrase in self._action_map:
                    _, slots.action = self._action_map[phrase]
                    break
            if slots.action:
                break

        # --- Area: longest alias match ---
        for alias in sorted(self._area_map, key=len, reverse=True):
            if alias in normalized_text:
                slots.area = self._area_map[alias]
                break

        # --- Device class or entity_id from device aliases ---
        for alias in sorted(self._device_map, key=len, reverse=True):
            if alias in normalized_text:
                dev_id, dev_class = self._device_map[alias]
                # Check if device has a direct entity_id
                dev_data = self._device_aliases.get(dev_id, {})
                if dev_data.get("entity_id"):
                    slots.entity_id = dev_data["entity_id"]
                slots.device_class = dev_class
                break

        # --- Sensor entity_id (overrides device if more specific) ---
        for alias in sorted(self._sensor_entity_map, key=len, reverse=True):
            if alias in normalized_text:
                slots.entity_id = self._sensor_entity_map[alias]
                break

        # --- Numeric parameters ---
        for match in _NUMBER_RE.finditer(normalized_text):
            value = match.group(1).replace(",", ".")
            unit = (match.group(2) or "").strip()
            if unit in ("pourcent", "pourcents", "%"):
                slots.parameters["brightness_pct"] = value
            elif unit in ("degre", "degres", "°c"):
                slots.parameters["temperature"] = value
            elif unit in ("watt", "watts", "w"):
                slots.parameters["power"] = value
            else:
                slots.parameters.setdefault("value", value)

        return slots
