from diagnostic_msgs.msg import DiagnosticArray
import time
import json

""" Objectif : Extraire les données depuis /diagnostics selon configuration.
- Configuration via paramètres ROS (priorité) ou fichier YAML (fallback)
- Extraire les clés critiques définies (valeurs brutes, sans transformation)
- Publier un événement SYSTEM_STATE_UPDATED quand les clés critiques changent
"""

class DiagnosticsAdapter:

    def __init__(self, node, critical_keys: list = None, monitored_nodes: list = None):

        self.node = node

        self.critical_keys = critical_keys or []
        self.monitored_nodes = monitored_nodes or []
        self.node.get_logger().info(
            f"DiagnosticsAdapter : {len(self.critical_keys)} clés critiques, "
            f"{len(self.monitored_nodes)} nœuds surveillés"
        )

        # Stockage brut des diagnostics (pour référence complète)
        self.robot_state = {}

        # État système simplifié (clés critiques extraites)
        self._system_state = {}

        # Pour les diagnostics de niveau (WARNING/ERROR)
        self._level_memory = {}
        self._stability_delay = 3.0
        self.active_events = {}

        # Compteur pour log périodique
        self._callback_count = 0

        # Suivi des profils actifs (depuis qbo_bringup_manager)
        # None = jamais reçu → force la publication du premier PROFILE_STATUS
        self._last_active_profiles = None

        self.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self._callback,
            10
        )

    def safe_level(self, level):
        if isinstance(level, int):
            return level
        if isinstance(level, (bytes, bytearray)):
            return level[0]
        try:
            return int(level)
        except:
            return 0

    def _callback(self, msg):

        now = time.time()
        self._callback_count += 1

        for status in msg.status:

            hardware = status.hardware_id or "unknown"
            category = status.name
            level = self.safe_level(status.level)
            message = status.message
            values = {kv.key: kv.value for kv in status.values}

            # ── Stockage complet dans robot_state (toujours, sans délai) ──
            self.robot_state.setdefault(hardware, {})[category] = {
                "level": level,
                "message": message,
                "values": values,
                "last_seen": now,
            }

            # ===== EXTRACTION DES CLÉS CRITIQUES =====
            self._extract_critical_keys(values)

            # ===== PROFILS ACTIFS (qbo_bringup_manager) =====
            self._extract_profile_status(hardware, values)

            # ===== DIAGNOSTICS NORMAUX (stabilité + événements) =====
            key = f"{hardware}|{category.split(':')[-1].strip()}"

            memory = self._level_memory.get(key)

            if memory is None:
                self._level_memory[key] = {"level": level, "since": now}
                continue

            if memory["level"] != level:
                self._level_memory[key] = {"level": level, "since": now}
                continue

            if (now - memory["since"]) < self._stability_delay:
                continue

            self._process_diagnostic(key, level, message, hardware, category)

    def _extract_profile_status(self, hardware: str, values: dict):
        """
        Détecte les profils actifs publiés par qbo_bringup_manager dans /diagnostics.
        Publie PROFILE_STATUS à chaque changement, y compris [] au démarrage
        quand active_profiles vaut 'none' (débloque _profile_state_received dans system_mode_manager).
        """
        if hardware != "qbo_bringup_manager":
            return

        raw = values.get("active_profiles", "")
        if raw.strip().lower() in ("none", "", "null"):
            active = []
        else:
            active = [p.strip() for p in raw.split(",") if p.strip()]

        if active == self._last_active_profiles:
            return

        self._last_active_profiles = active
        self._publish_profile_status(active)

    def _publish_profile_status(self, active_profiles: list):
        """Publie un événement PROFILE_STATUS avec la liste des profils actifs."""
        inspector = getattr(self.node, "diagnostics_inspector", None)
        nodes_present = dict(inspector._present) if inspector is not None else {}

        msg = self.node._create_event_msg(
            event_type="PROFILE_STATUS",
            source="diagnostics_adapter",
            payload={
                "active_profiles": active_profiles,
                "nodes_present": nodes_present,
            }
        )
        self.node.pub_event.publish(msg)
        self.node.get_logger().info(
            f"EVENT → PROFILE_STATUS : {active_profiles}"
        )

    def _extract_critical_keys(self, values: dict):
        """
        Extrait les clés critiques définies dans diagnostics_config.yaml.
        Stocke les valeurs brutes (pas de transformation).
        Publie un événement SYSTEM_STATE_UPDATED si au moins une clé change.
        """
        changed = False
        changes = {}

        for key in self.critical_keys:
            if key in values:
                raw_value = values[key]
                old_value = self._system_state.get(key)

                if raw_value != old_value:
                    self._system_state[key] = raw_value
                    changes[key] = {"old": old_value, "new": raw_value}
                    changed = True

                    self.node.get_logger().info(
                        f"[CRITICAL_KEY] {key}: {old_value} → {raw_value}"
                    )

        # Publier un seul événement si au moins une clé a changé
        if changed:
            self._publish_system_state_event(changes)

    def _publish_system_state_event(self, changes: dict):
        """
        Publie un événement SYSTEM_STATE_UPDATED avec l'état complet et les changements.
        """
        inspector = getattr(self.node, "diagnostics_inspector", None)
        nodes_present = dict(inspector._present) if inspector is not None else {}

        msg = self.node._create_event_msg(
            event_type="SYSTEM_STATE_UPDATED",
            source="diagnostics_adapter",
            payload={
                "system_state": self._system_state,
                "nodes_present": nodes_present,
                "changes": changes,
            }
        )

        self.node.pub_event.publish(msg)

        self.node.get_logger().info(
            f"EVENT → SYSTEM_STATE_UPDATED : {list(changes.keys())}"
        )

    def _process_diagnostic(self, key, level, message, hardware, category):

        severity_map = {
            1: "warning",
            2: "error"
        }

        previous_level = self.active_events.get(key, 0)
        is_first_seen = key not in self.active_events

        # =========================
        # CAS 1 : ACTIVATION
        # =========================
        if level > 0:

            # nouvel event OU changement de niveau
            if previous_level != level:

                self.active_events[key] = level

                self._publish_event(
                    key=key,
                    severity=severity_map.get(level, "info"),
                    message=message,
                    level=level,
                    active=True,
                    hardware=hardware,
                    category=category
                )

        # =========================
        # CAS 2 : RÉSOLUTION
        # =========================
        else:

            if previous_level > 0:
                # Résolution d'un problème existant
                self.active_events[key] = 0

                self._publish_event(
                    key=key,
                    severity="info",
                    message="resolved",
                    level=0,
                    active=False,
                    hardware=hardware,
                    category=category
                )

            elif is_first_seen:
                # Première apparition stable à niveau OK
                # Permet à system_mode_manager de savoir que le controller est sain dès le démarrage
                self.active_events[key] = 0

                self._publish_event(
                    key=key,
                    severity="info",
                    message=message,
                    level=0,
                    active=True,
                    hardware=hardware,
                    category=category
                )

    def _publish_event(self, key, severity, message, level=0, active=True, hardware="unknown", category="unknown"):

        severity_map = {
            0: "info",
            1: "warning",
            2: "error"
        }

        msg = self.node._create_event_msg(
            event_type="DIAGNOSTIC",
            source="diagnostics",
            payload={
                "key": key,
                "severity": severity,
                "message": message,
                "level": level,
                "active": active,
                "hardware": hardware,
                "category": category,
            }
        )

        self.node.pub_event.publish(msg)

        self.node.get_logger().info(
            f"EVENT → DIAGNOSTIC {key} ({level}) {'ACTIVE' if active else 'RESOLVED'} : {message}"
        )
