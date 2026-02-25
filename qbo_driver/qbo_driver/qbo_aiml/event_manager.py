import time
from collections import deque

class EventManager:

    VALID_STATES = [
        "inactive",
        "active",
        "notified",
        "proposed",
        "executing",
        "resolved",
        "ignored"
    ]

    ALLOWED_TRANSITIONS = {
        "inactive": ["active"],
        "active": ["notified", "proposed", "resolved"],
        "notified": ["proposed", "resolved", "ignored"],
        "proposed": ["executing", "resolved"],
        "executing": ["resolved"],
        "resolved": ["active"],
        "ignored": ["active", "resolved"]
    }

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
                "first_seen": None,
                "last_change": now,
                "last_update": now,
                "cooldown_until": 0,
                "snoozed_until": 0
            }

        return self.events[key]

    # =====================================================
    # Transition centralisée (FSM)
    # =====================================================
    def _transition(self, event, new_state):

        current = event["state"]

        if new_state not in self.ALLOWED_TRANSITIONS.get(current, []):
            self._log(
                "error",
                f"❌ Transition invalide {current} → {new_state} "
                f"pour {event['key']}"
            )
            return False

        event["state"] = new_state
        event["last_change"] = time.time()

        self._archive_transition(event, new_state)

        return True

    def get_events_by_state(self, state):
        return [
            (key, event)
            for key, event in self.events.items()
            if event["state"] == state
        ]

    # =====================================================
    # Update depuis diagnostics
    # =====================================================
    def update_event(self, key, active=True, severity="info",
                     message="", cooldown=None):

        now = time.time()
        cooldown = cooldown or self.cooldown_default

        event = self._get_or_create(key, severity, message)

        event["severity"] = severity
        event["message"] = message
        event["last_update"] = now

        # 🔒 Protection cooldown
        if active and now < event["cooldown_until"]:
            return

        # -----------------------------
        # Activation
        # -----------------------------
        if active:

            if event["state"] in ["inactive", "resolved"]:

                if event["first_seen"] is None:
                    event["first_seen"] = now

                if self._transition(event, "active"):
                    self.event_queue.append(event)

                    self._log(
                        "warn",
                        f"🚨 Event activé: {key} ({severity}) | {message}"
                    )

        # -----------------------------
        # Résolution automatique (diagnostic revenu OK)
        # -----------------------------
        else:

            if event["state"] in ["active", "proposed", "executing"]:

                if self._transition(event, "resolved"):

                    event["cooldown_until"] = now + cooldown

                    self._log("info", f"✅ Event résolu: {key}")

    # =====================================================
    # Dispatch prochain event
    # =====================================================
    def get_next_event(self):

        if not self.event_queue:
            return None

        now = time.time()

        valid = []

        for event in list(self.event_queue):

            if event["state"] != "active":
                continue

            if now < event["snoozed_until"]:
                continue

            valid.append(event)

        if not valid:
            return None

        # Priorité ERROR > WARNING > INFO
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

        # if self._transition(event, "proposed"):
        #     self._log("info", f"📢 Event dispatch: {event['key']}")
        return event

        #return None

    # =====================================================
    # Marquer en exécution (après confirmation)
    # =====================================================
    def mark_executing(self, key):

        event = self.events.get(key)
        if not event:
            return

        self._transition(event, "executing")

    # =====================================================
    # Résolution explicite (action réussie)
    # =====================================================
    def resolve_event(self, key):

        event = self.events.get(key)
        if not event:
            return

        if self._transition(event, "resolved"):
            event["cooldown_until"] = time.time() + self.cooldown_default
            self._log("info", f"✅ Event résolu: {key}")

    # =====================================================
    # Snooze
    # =====================================================
    def snooze_event(self, key, duration=30):

        event = self.events.get(key)
        if not event:
            return

        event["snoozed_until"] = time.time() + duration

        self._log("info", f"😴 Event snoozed: {key} ({duration}s)")

    # =====================================================
    # Archive transitions
    # =====================================================
    def _archive_transition(self, event, transition_type):

        self.history.append({
            "key": event["key"],
            "severity": event["severity"],
            "message": event["message"],
            "transition": transition_type,
            "timestamp": time.time()
        })

    # =====================================================
    # Historique
    # =====================================================
    def get_history(self):
        return self.history
