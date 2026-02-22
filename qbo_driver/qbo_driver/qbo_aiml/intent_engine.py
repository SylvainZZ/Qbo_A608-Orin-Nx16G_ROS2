import rclpy
from rosidl_runtime_py.utilities import get_message, get_service


''' IntentEngine:
- Reçoit un intent structuré (action + params)
- Accède à l'état robot partagé (diagnostics)
- Résout les paramètres dynamiques (ex: {battery_voltage})
- Exécute l'action correspondante (publication topic ou appel service)

'''

class IntentEngine:

    def __init__(self, node):
        """
        node = AIMLNode (pour publisher/service/log)
        """
        self.node = node

        self.intent_handlers = {
            "get_led": self.intent_get_led,
            "set_led": self.intent_set_led,
            "test_leds": self.intent_test_leds,
            "get_battery": self.intent_get_battery,
            "get_temperature": self.intent_get_temperature,
            "calibrate_imu": self.intent_calibrate_imu
        }

    # =====================================================
    # PUBLIC EXECUTION ENTRY
    # =====================================================
    def execute(self, intent: dict, robot_state: dict, params: dict):

        try:
            action = intent.get("action", "")
            handler = self.intent_handlers.get(action)

            if handler:
                return handler(intent, robot_state, params)

            self.node.get_logger().warn(f"❌ Action inconnue : {action}")

        except Exception as e:
            self.node.get_logger().error(f"❌ Erreur exécution intent : {e}")

        return None

    # =====================================================
    # UTILITAIRES
    # =====================================================

    def resolve_params(self, param_template: dict, detected_params: dict):

        resolved = {}

        for key, value in param_template.items():
            if isinstance(value, str) and value.startswith("{") and value.endswith("}"):
                var = value.strip("{}")
                resolved[key] = detected_params.get(var, 0)
            else:
                resolved[key] = value

        return resolved

    def publish_ros_topic(self, target, msg_type, args):

        msg_class = get_message(msg_type)
        msg = msg_class(**args)

        pub = self.node.create_publisher(msg_class, target, 10)
        pub.publish(msg)

        self.node.get_logger().info(f"✅ Topic publié sur {target} avec {args}")

    def call_ros_service(self, target: str, srv_type: str, timeout: float = 5.0) -> bool:
        """
        Appelle un service ROS et attend la réponse.
        Retourne True si succès, False sinon.
        """
        try:
            # 1️⃣ Récupération type service
            srv_class = get_service(srv_type)

            # 2️⃣ Création client
            client = self.node.create_client(srv_class, target)

            # 3️⃣ Attente disponibilité
            if not client.wait_for_service(timeout_sec=timeout):
                self.node.get_logger().warn(
                    f"❌ Service non disponible : {target}"
                )
                return False

            # 4️⃣ Création requête
            request = srv_class.Request()

            # 5️⃣ Appel async + attente résultat
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)

            # 6️⃣ Vérification résultat
            if future.result() is None:
                self.node.get_logger().error(
                    f"❌ Service {target} a échoué (pas de réponse)."
                )
                return False

            self.node.get_logger().info(
                f"✅ Service exécuté avec succès : {target}"
            )

            return True

        except Exception as e:
            self.node.get_logger().error(
                f"❌ Erreur appel service {target} : {e}"
            )
            return False

    # =====================================================
    # INTENTS NOSE
    # =====================================================

    def intent_get_led(self, intent, robot_state, params):

        try:
            nose = robot_state.get("Q.Board_5", {}) \
                .get("nose_ctrl: Nose LED", {})

            values = nose.get("values", {})

            color_code = int(values.get("color_code", 0))
            color_name = values.get("color_name", "éteint").lower()

            if color_code == 0:
                return {
                    "state": "off",
                    "color_code": 0,
                    "color_label": "éteint"
                }

            return {
                "state": "on",
                "color_code": color_code,
                "color_label": color_name
            }

        except Exception:
            return {
                "state": "unknown",
                "color_label": "inconnu"
            }

    def intent_set_led(self, intent, robot_state, params):

        resolved = self.resolve_params(intent.get("params", {}), params)

        color_code = resolved.get("color", 0)

        # # Mise à jour état local (sera écrasé par diagnostics ensuite)
        # robot_state.setdefault("nose", {})
        # robot_state["nose"]["color_code"] = color_code

        if intent.get("type") == "topic":
            self.publish_ros_topic(intent["target"], intent["msg_type"], resolved)

        return None

    def intent_test_leds(self, intent, robot_state, params):

        if intent.get("type") == "service":
            self.call_ros_service(intent["target"], intent["msg_type"])

        return None

    # =====================================================
    # INTENTS IMU
    # =====================================================

    def intent_calibrate_imu(self, intent, robot_state, params):

        if intent.get("type") == "service":
            success = self.call_ros_service(intent["target"], intent["msg_type"])

        if success:
            return {"status": "success"}
        else:
            return {"status": "failed"}

    # =====================================================
    # INTENTS BATTERY
    # =====================================================

    def intent_get_battery(self, intent, robot_state, params):

        battery = robot_state.get("battery", {})
        voltage = battery.get("voltage", 0)
        runtime = battery.get("runtime_min", 0)
        charging = battery.get("external_power", False)

        return {
            "voltage": voltage,
            "runtime": runtime,
            "charging": charging
        }

    # =====================================================
    # INTENTS TEMPERATURE
    # =====================================================

    def intent_get_temperature(self, intent, robot_state, params):

        temp = robot_state.get("temperature", {})
        return {
            "cpu": temp.get("cpu", 0),
            "gpu": temp.get("gpu", 0)
        }
