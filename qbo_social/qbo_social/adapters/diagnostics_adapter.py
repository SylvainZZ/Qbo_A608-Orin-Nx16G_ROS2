from diagnostic_msgs.msg import DiagnosticArray
import time
import json


class DiagnosticsAdapter:

    def __init__(self, node):

        self.node = node

        self.robot_state = {}
        self._level_memory = {}
        self._stability_delay = 3.0
        self.active_events = {}  # key → level

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

        for status in msg.status:

            hardware = status.hardware_id or "unknown"
            category = status.name
            level = self.safe_level(status.level)
            message = status.message

            key = f"{hardware}|{category.split(':')[-1].strip()}"

            # stabilité
            memory = self._level_memory.get(key)

            if memory is None:
                self._level_memory[key] = {
                    "level": level,
                    "since": now
                }
                continue

            if memory["level"] != level:
                self._level_memory[key] = {
                    "level": level,
                    "since": now
                }
                continue

            if (now - memory["since"]) < self._stability_delay:
                continue

            # 🔥 Publication
            self._process_diagnostic(key, level, message)

    def _process_diagnostic(self, key, level, message):

        severity_map = {
            1: "warning",
            2: "error"
        }

        previous_level = self.active_events.get(key, 0)

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
                    active=True
                )

        # =========================
        # CAS 2 : RÉSOLUTION
        # =========================
        else:

            # seulement si l’event existait
            if previous_level > 0:

                self.active_events[key] = 0

                self._publish_event(
                    key=key,
                    severity="info",
                    message="resolved",
                    level=0,
                    active=False
            )

    def _publish_event(self, key, severity, message, level=0, active=True):

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
                "active": active
            }
        )

        self.node.pub_event.publish(msg)

        self.node.get_logger().info(
            f"EVENT → DIAGNOSTIC {key} ({level}) {'ACTIVE' if active else 'RESOLVED'} : {message}"
        )