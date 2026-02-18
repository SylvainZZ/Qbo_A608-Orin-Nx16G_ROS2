from diagnostic_msgs.msg import DiagnosticArray
import copy

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

    def callback(self, msg: DiagnosticArray):

        old_state = copy.deepcopy(self.node.robot_state)

        for status in msg.status:

            name = status.name

            if "battery_ctrl" in name:
                self.parse_battery(status)

            elif "base_ctrl" in name:
                self.parse_motors(status)

            elif "imu_ctrl" in name:
                self.parse_imu(status)

            elif "nose_ctrl" in name:
                self.parse_nose(status)

            elif "OrinA608Diag_node" in name:
                self.parse_orin(status)

        events = self.detect_changes(old_state, self.node.robot_state)

        for key, message, severity in events:
            if message:
                self.node.event_manager.update_event(key, True, message, severity)
            else:
                self.node.event_manager.update_event(key, False, "")
            # Optionnel : parler automatiquement
            # self.node.say(event)

    # ============================
    # PARSERS
    # ============================

    def get_value(self, status, key, cast=float, default=0):
        for v in status.values:
            if v.key == key:
                try:
                    return cast(v.value)
                except:
                    return v.value
        return default

    def parse_battery(self, status):
        self.node.robot_state["battery"] = {
            "voltage": self.get_value(status, "Voltage (V)", float, 0.0),
            "charge_mode": self.get_value(status, "Charge Mode Description", cast=str, default=""),
            "external_power": self.get_value(status, "External Power", cast=lambda x: x == "Yes", default=False),
            "runtime_min": self.get_value(status, "Estimated Runtime (min)", float, 0.0)
        }

    def parse_motors(self, status):
        values = {v.key: v.value for v in status.values}
        self.node.robot_state["motors"] = {
            "left_ok": values.get("Left Motor OK", "") == "yes",
            "right_ok": values.get("Right Motor OK", "") == "yes"
        }

    def parse_imu(self, status):
        values = {v.key: v.value for v in status.values}
        self.node.robot_state["imu"] = {
            "calibrated": values.get("IMU calibrated", "") == "yes"
        }

    def parse_nose(self, status):
        values = {v.key: v.value for v in status.values}
        self.node.robot_state["nose"] = {
            "color_code": int(values.get("color_code", 0)),
            "color_name": values.get("color_name", "Off")
        }

    def parse_orin(self, status):
        values = {v.key: v.value for v in status.values}
        self.node.robot_state["temperature"] = {
            "cpu": float(values.get("CPU °C", 0)),
            "gpu": float(values.get("GPU °C", 0))
        }

    # ============================
    # DETECTION CHANGEMENTS
    # ============================

    def detect_changes(self, old, new):

        events = []

        old_batt = old.get("battery", {})
        new_batt = new.get("battery", {})

        old_level = old_batt.get("level")
        new_level = new_batt.get("level")

        # 🔋 Changement de niveau batterie
        if old_level != new_level:

            if new_level == 1:  # WARN
                events.append(("battery_low", "battery_low", "warning"))

            elif new_level == 2:  # ERROR
                events.append(("battery_empty", "battery_empty", "error"))

            elif new_level == 0:  # Retour OK
                events.append(("battery_low", None, None))
                events.append(("battery_empty", None, None))

        # 🔌 Passage en charge
        if old_batt.get("external_power") is False \
        and new_batt.get("external_power") is True:
            events.append(("battery_charging", "battery_charging", "info"))

        # 🧭 IMU

        if old_imu != new_imu:
            if new_imu is False:
                events.append(("imu_not_calibrated", "IMU not calibrated.", "warning"))
            else:
                events.append(("imu_not_calibrated", None, None))

        return events
