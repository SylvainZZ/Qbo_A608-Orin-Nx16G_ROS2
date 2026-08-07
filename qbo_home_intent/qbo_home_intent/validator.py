"""Validator: checks action feasibility and applies safety rules."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory

_CONFIG_DIR = Path(get_package_share_directory('qbo_home_intent')) / 'config'


class IntentValidator:
    def __init__(
        self,
        capabilities_path: Path | None = None,
        safety_path: Path | None = None,
    ) -> None:
        cap_path = capabilities_path or (_CONFIG_DIR / "capabilities.yaml")
        safe_path = safety_path or (_CONFIG_DIR / "safety.yaml")

        with open(cap_path, encoding="utf-8") as fh:
            self._capabilities: dict[str, Any] = yaml.safe_load(fh).get("capabilities", {})
        with open(safe_path, encoding="utf-8") as fh:
            safety_data: dict[str, Any] = yaml.safe_load(fh)

        self._rules: list[dict[str, Any]] = safety_data.get("rules", [])
        self._risk_levels: dict[int, Any] = {
            int(k): v for k, v in safety_data.get("risk_levels", {}).items()
        }

    def validate_action(self, device_class: str, action: str) -> tuple[bool, str]:
        """Return (valid, error_message). Read-only sensors always reject control actions."""
        cap = self._capabilities.get(device_class)
        if cap is None:
            return True, ""  # unknown class — pass through
        if cap.get("read_only") and action not in ("", None):
            return False, f"'{device_class}' is read-only"
        allowed: list[str] = cap.get("actions", [])
        if action and allowed and action not in allowed:
            return False, f"Action '{action}' not supported for '{device_class}'"
        return True, ""

    def get_risk(
        self,
        entity_id: str,
        domain: str,
        action: str,
        parameters: dict[str, str],
    ) -> tuple[int, str]:
        """Return (risk_level, warning_message) for the first matching rule."""
        for rule in self._rules:
            rule_entity = rule.get("entity_id", "")
            rule_domain = rule.get("domain", "")
            rule_action = rule.get("action", "")

            if rule_entity and rule_entity != entity_id:
                continue
            if rule_domain and rule_domain != domain:
                continue
            if rule_action and rule_action != action:
                continue

            condition = rule.get("condition")
            if condition:
                param_val = parameters.get(condition["parameter"])
                if param_val is None:
                    continue
                try:
                    fval = float(param_val)
                    threshold = float(condition["value"])
                    op = condition["operator"]
                    if op == ">" and fval <= threshold:
                        continue
                    if op == "<" and fval >= threshold:
                        continue
                except ValueError:
                    continue

            return rule.get("risk_level", 0), rule.get("warning", "")

        return 0, ""

    def confirmation_required(self, risk_level: int) -> bool:
        return self._risk_levels.get(risk_level, {}).get("requires_confirmation", False)
