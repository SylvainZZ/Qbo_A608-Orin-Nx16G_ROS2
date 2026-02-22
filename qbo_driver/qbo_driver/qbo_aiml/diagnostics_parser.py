from diagnostic_msgs.msg import DiagnosticArray
import copy
import time

from qbo_driver.qbo_aiml.constants import BATTERY_LOW_VOLTAGE


'''
Ce module parse les messages de diagnostic ROS pour extraire des informations sur l'état du robot (batterie, moteurs, IMU, température)
et détecter des événements importants (ex: batterie faible, IMU non calibrée).
Il est utilisé par l'AIMLNode pour alimenter sa compréhension du contexte robotique et adapter les réponses de l'IA en conséquence.
- DiagnosticsParser : classe qui s'abonne à /diagnostics, parse les messages et met à jour le robot_state du node parent.
- parse_battery(), parse_motors(), parse_imu(), parse_nose(), parse_orin() : méthodes de parsing spécifiques pour chaque composant.
- detect_changes() : compare l'ancien et le nouveau état du robot pour détecter des événements importants et générer des alertes.

🔥 Ce que ce module apporte :
    ✔ Extraction d'informations détaillées sur l'état du robot
    ✔ Détection d'événements critiques
    ✔ Mise à jour automatique du robot_state
    ✔ Intégration transparente avec l'AIMLNode

🧠 Évolutions futures possibles :
    - Ajouter parsing pour d'autres composants (ex: caméra, micro)
    - Ajouter détection d'événements plus complexes (ex: surchauffe + batterie faible)
    - Intégrer apprentissage pour détecter des patterns d'événements
    - Ajouter publication de topics d'état pour d'autres nodes

'''

class DiagnosticsParser:

    def __init__(self, node):
        self.node = node
        self.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.callback,
            10
        )
        self.initialized = False
        self._level_memory = {}   # {event_key: {"level": int, "since": timestamp}}
        self._stability_delay = 3.0  # secondes de stabilité requise

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

        # old_state = copy.deepcopy(self.node.robot_state)

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
