import time
from collections import deque

class EventManager:

    VALID_STATES = [
        "inactive",
        "active",
        "proposed",
        "executing",
        "resolved"
    ]

    def __init__(self, cooldown_default=30, logger=None):

        self.events = {}            # état courant des events
        self.event_queue = deque()  # queue à traiter
        self.history = []           # archive des events résolus
        self.cooldown_default = cooldown_default
        self.logger = logger

    # -----------------------------------------------------
    # Logging interne propre
    # -----------------------------------------------------
    def _log(self, level, message):
        if not self.logger:
            return

        if level == "info":
            self.logger.info(message)
        elif level == "warn":
            self.logger.warn(message)
        elif level == "error":
            self.logger.error(message)

    # -----------------------------------------------------
    # Création ou récupération d’un event
    # -----------------------------------------------------
    def _get_or_create(self, key, severity="info", message=""):

        now = time.time()

        if key not in self.events:
            self.events[key] = {
                "key": key,
                "state": "inactive",
                "severity": severity,
                "message": message,
                "first_seen": now,
                "last_change": now,
                "last_update": now,
                "cooldown_until": 0,
                "snoozed_until": 0
            }

        return self.events[key]

    # -----------------------------------------------------
    # Mise à jour d’un event (entrée depuis diagnostics)
    # -----------------------------------------------------
    def update_event(self, key, active=True, severity="info",
                     message="", cooldown=None):

        now = time.time()
        cooldown = cooldown or self.cooldown_default

        event = self._get_or_create(key, severity, message)

        event["severity"] = severity
        event["message"] = message
        event["last_update"] = now

        # 🔒 Cooldown protection
        if active and now < event["cooldown_until"]:
            return

        # 🔹 Activation
        if active:

            if event["state"] in ["inactive", "resolved"]:
                event["state"] = "active"
                event["last_change"] = now
                self.event_queue.append(event)

                self._log(
                    "warn",
                    f"🚨 Event activé: {key} ({severity}) | {message}"
                )

        # 🔹 Résolution
        else:
            if event["state"] not in ["inactive", "resolved"]:
                event["state"] = "resolved"
                event["last_change"] = now
                event["cooldown_until"] = now + cooldown

                self._archive_event(event)

                self._log("info", f"✅ Event résolu: {key}")

    # -----------------------------------------------------
    # Récupération prochain event à traiter
    # -----------------------------------------------------
    def get_next_event(self):

        if not self.event_queue:
            return None

        now = time.time()

        # 🔹 Filtrer snooze + état valide
        valid = []

        for event in list(self.event_queue):

            if event["state"] != "active":
                continue

            if now < event["snoozed_until"]:
                continue

            valid.append(event)

        if not valid:
            return None

        # 🔹 Priorité
        sorted_events = sorted(
            valid,
            key=lambda e: {
                "error": 0,
                "warning": 1,
                "info": 2
            }.get(e["severity"], 3)
        )

        event = sorted_events[0]
        self.event_queue.remove(event)

        event["state"] = "proposed"
        event["last_change"] = now

        self._log("info", f"📢 Event dispatch: {event['key']}")

        return event

    # -----------------------------------------------------
    # Marquer en exécution (après confirmation utilisateur)
    # -----------------------------------------------------
    def mark_executing(self, key):

        event = self.events.get(key)
        if not event:
            return

        event["state"] = "executing"
        event["last_change"] = time.time()

    # -----------------------------------------------------
    # Résolution explicite (ex: action réussie)
    # -----------------------------------------------------
    def resolve_event(self, key):

        event = self.events.get(key)
        if not event:
            return

        if event["state"] != "inactive":
            event["state"] = "resolved"
            event["last_change"] = time.time()
            event["cooldown_until"] = time.time() + self.cooldown_default

            self._archive_event(event)

            self._log("info", f"✅ Event résolu: {key}")

    # -----------------------------------------------------
    # Snooze temporel
    # -----------------------------------------------------
    def snooze_event(self, key, duration=30):

        event = self.events.get(key)
        if not event:
            return

        event["snoozed_until"] = time.time() + duration

        self._log("info", f"😴 Event snoozed: {key} ({duration}s)")

    # -----------------------------------------------------
    # Archive historique
    # -----------------------------------------------------
    def _archive_event(self, event):

        self.history.append({
            "key": event["key"],
            "severity": event["severity"],
            "message": event["message"],
            "first_seen": event["first_seen"],
            "resolved_at": event["last_change"]
        })

        # Reset état interne
        event["state"] = "inactive"

    # -----------------------------------------------------
    # Accès historique
    # -----------------------------------------------------
    def get_history(self):
        return self.history
