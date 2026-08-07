"""Resolver: maps slots to concrete objects from the knowledge base."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory

_KNOWLEDGE_DIR = Path(get_package_share_directory('qbo_home_intent')) / 'knowledge'


class HomeResolver:
    def __init__(
        self,
        objects_path: Path | None = None,
        virtual_path: Path | None = None,
    ) -> None:
        objects_path = objects_path or (_KNOWLEDGE_DIR / "home_objects.yaml")
        virtual_path = virtual_path or (_KNOWLEDGE_DIR / "virtual_objects.yaml")

        with open(objects_path, encoding="utf-8") as fh:
            self._objects: list[dict[str, Any]] = yaml.safe_load(fh).get("objects", [])
        with open(virtual_path, encoding="utf-8") as fh:
            self._virtual: list[dict[str, Any]] = yaml.safe_load(fh).get("virtual_objects", [])

        self._all = self._objects + self._virtual

    def resolve(
        self,
        area: str = "",
        device_class: str = "",
        name: str = "",
        entity_id: str = "",
    ) -> list[dict[str, Any]]:
        """Return matching objects ordered by specificity (most specific first)."""
        # Exact entity_id match is unambiguous
        if entity_id:
            exact = [o for o in self._all if o.get("entity_id") == entity_id]
            if exact:
                return exact

        candidates = list(self._all)

        if area:
            candidates = [o for o in candidates if o.get("area") == area]
        if device_class:
            candidates = [o for o in candidates if o.get("device_class") == device_class]
        if name:
            name_lower = name.lower()
            by_name = [
                o for o in candidates
                if name_lower in o.get("label", "").lower()
                or any(name_lower in a for a in o.get("aliases", []))
            ]
            if by_name:
                return by_name

        return candidates

    def find_by_alias(self, alias: str) -> list[dict[str, Any]]:
        """Return objects whose aliases list contains *alias*."""
        return [o for o in self._all if alias in o.get("aliases", [])]
