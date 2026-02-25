from diagnostic_msgs.msg import DiagnosticArray
import copy
import time
import random
import os
import json

from qbo_driver.qbo_aiml.constants import BATTERY_LOW_VOLTAGE


class DiagnosticsParser:

    def __init__(self, node, watchers_dir):
        self.node = node
        self.watchers_dir = watchers_dir
        self.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.callback,
            10
        )
        self.initialized = False
        self._level_memory = {}   # {event_key: {"level": int, "since": timestamp}}
        self._stability_delay = 3.0  # secondes de stabilité requise
        self.previous_state = {}
        self.last_spoken = {}

        rules_path = os.path.join(self.watchers_dir, "state_watchers.json")

        if not os.path.exists(rules_path):
            self.node.get_logger().warn(
                f"Fichier watchers introuvable : {rules_path}"
            )
            self.state_rules = []
        else:
            with open(rules_path, "r") as f:
                self.state_rules = json.load(f)

        self.last_spoken = {}

    def safe_level(self, level):
        if isinstance(level, int):
            return level
        if isinstance(level, (bytes, bytearray)):
            return level[0]
        try:
            return int(level)
        except:
            return 0


    def callback(self, msg: DiagnosticArray):

        old_state = copy.deepcopy(self.node.robot_state)

        for status in msg.status:

            hardware = status.hardware_id or "unknown"
            category = status.name
            level = self.safe_level(status.level)
            message = status.message

            values = {v.key: v.value for v in status.values}

            if hardware not in self.node.robot_state:
                self.node.robot_state[hardware] = {}

            self.node.robot_state[hardware][category] = {
                "level": level,
                "message": message,
                "values": values
            }

        # 🔒 Ne pas générer d’event au premier cycle
        if not self.initialized:
            self.initialized = True
            return

        self.detect_changes(self.node.robot_state)
        self.detect_state_transitions(old_state, self.node.robot_state)

    # ============================
    # DETECTION CHANGEMENTS
    # ============================

    def detect_changes(self, new):

        severity_map = {
            1: "warning",
            2: "error"
        }

        now = time.time()

        for hardware, categories in new.items():

            for category, data in categories.items():

                clean_category = category.split(":")[-1].strip()
                event_key = f"{hardware}|{clean_category}"

                new_level = data.get("level", 0)
                new_message = data.get("message", "")

                # 🔹 Gestion stabilité
                memory = self._level_memory.get(event_key)

                if memory is None:
                    self._level_memory[event_key] = {
                        "level": new_level,
                        "since": now
                    }
                    continue

                # Niveau changé → reset timer
                if memory["level"] != new_level:
                    self._level_memory[event_key] = {
                        "level": new_level,
                        "since": now
                    }
                    continue

                # Pas encore stable
                if now - memory["since"] < self._stability_delay:
                    continue

                # 🔹 Niveau stable → on regarde l’état courant de l’event manager
                event_data = self.node.event_manager.events.get(event_key)

                # Cas 1 : niveau > 0 → doit être actif
                if new_level > 0:

                    if not event_data or event_data["state"] in ["inactive", "resolved"]:

                        severity = severity_map.get(new_level, "info")

                        self.node.event_manager.update_event(
                            event_key,
                            active=True,
                            severity=severity,
                            message=new_message
                        )

                # Cas 2 : niveau = 0 → doit être résolu
                else:

                    if event_data and event_data["state"] not in ["inactive", "resolved"]:

                        self.node.event_manager.update_event(
                            event_key,
                            active=False
                        )
    # ============================
    # DÉTECTION CHANGEMENT MODE CHARGE
    # ============================
    def detect_state_transitions(self, old, new):

        for hardware, categories in new.items():

            for category, data in categories.items():

                for rule in self.state_rules:

                    match = rule["match"]

                    if match.get("hardware_contains") not in hardware:
                        continue

                    if match.get("category_contains") not in category:
                        continue

                    key = match.get("key")
                    if key not in data["values"]:
                        continue

                    new_val = data["values"].get(key)

                    try:
                        old_val = old[hardware][category]["values"].get(key)
                    except KeyError:
                        old_val = None

                    if new_val == old_val:
                        continue

                    self.handle_rule(rule, old_val, new_val)

    def handle_rule(self, rule, old, new):

        rule_id = rule["id"]
        cooldown = rule.get("cooldown", 5)

        now = time.time()
        last = self.last_spoken.get(rule_id, 0)

        if now - last < cooldown:
            return

        # Cas transitions mapping (battery)
        if "transitions" in rule:
            phrases = rule["transitions"].get(new)
            if not phrases:
                return
            text = random.choice(phrases)

        # Cas simple on_change (IP)
        elif "on_change" in rule:
            phrases = rule["on_change"]
            text = random.choice(phrases).format(old=old, new=new)

        else:
            return

        self.node.enqueue_speech(text, priority="info")
        self.last_spoken[rule_id] = now
