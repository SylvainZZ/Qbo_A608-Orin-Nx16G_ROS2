import rclpy
from rosidl_runtime_py.utilities import get_message, get_service
from qbo_driver.qbo_aiml.constants import COLOR_TRANSLATIONS_FR

class IntentEngine:

    def __init__(self, node):
        """
        node = AIMLNode (pour publisher/service/log)
        """
        self.node = node

        self.intent_handlers = {
            "publish_topic": self.intent_publish_topic,
            "call_service": self.intent_call_service,
            "get_multi_state": self.intent_get_multi_state,
            "get_state": self.intent_get_state
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

    def intent_publish_topic(self, intent, robot_state, params):
        try:
            resolved = self.resolve_params(intent.get("params", {}), params)

            target = intent.get("target")
            msg_type = intent.get("msg_type")

            if not target or not msg_type:
                self.node.get_logger().warn("❌ Intent publish_topic incomplet (target/msg_type).")
                return {"status": "failed"}

            self.publish_ros_topic(target, msg_type, resolved)
            return {"status": "sent"}

        except Exception as e:
            self.node.get_logger().error(f"❌ Erreur publish_topic : {e}")
            return {"status": "failed"}

    def publish_ros_topic(self, target, msg_type, args):

        msg_class = get_message(msg_type)
        msg = msg_class(**args)

        pub = self.node.create_publisher(msg_class, target, 10)
        pub.publish(msg)

        self.node.get_logger().info(f"✅ Topic publié sur {target} avec {args}")

    def intent_call_service(self, intent, robot_state, params):
        try:
            _ = self.resolve_params(intent.get("params", {}), params)

            target = intent.get("target")
            srv_type = intent.get("srv_type") or intent.get("msg_type")

            if not target or not srv_type:
                self.node.get_logger().warn("❌ Intent call_service incomplet (target/srv_type).")
                return {"status": "failed"}

            ok = self.call_ros_service(target, srv_type)
            return {"status": "success" if ok else "failed"}

        except Exception as e:
            self.node.get_logger().error(f"❌ Erreur call_service : {e}")
            return {"status": "failed"}

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
    # INTENTS GÉNÉRIQUES
    # =====================================================

    def intent_get_state(self, intent, robot_state, params):

        cfg = intent.get("params", {})
        hw = cfg.get("hardware")
        cat = cfg.get("category")
        key = cfg.get("key")
        map_name = cfg.get("map")

        out_key = cfg.get("result_key") or "value"

        try:
            value = robot_state[hw][cat]["values"][key]
        except KeyError:
            return {out_key: None}

        # Mapping optionnel
        if map_name == "color":
            value = COLOR_TRANSLATIONS_FR.get(value, value)

        return {out_key: value}

    def intent_get_multi_state(self, intent, robot_state, params):

        results = {}
        config = intent.get("params", {})

        for result_key, path in config.items():

            hw = path.get("hardware")
            cat = path.get("category")
            key = path.get("key")

            try:
                value = robot_state[hw][cat]["values"][key]
            except KeyError:
                value = None

            results[result_key] = value

        return results
