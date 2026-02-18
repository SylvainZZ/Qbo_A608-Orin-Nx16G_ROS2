import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from qbo_msgs.msg import ListenResult
from qbo_msgs.srv import Text2Speach
from std_srvs.srv import Trigger

import os

# 🔽 Imports internes
from qbo_driver.qbo_aiml.qa_loader import QALoader
from qbo_driver.qbo_aiml.intent_engine import IntentEngine
from qbo_driver.qbo_aiml.parameter_extractors import ParameterExtractor
from qbo_driver.qbo_aiml.diagnostics_parser import DiagnosticsParser


""" ---/* Structure du code AIMLNode */--- :
🎯 aiml.py
    Node ROS2
    Souscriptions
    Publication TTS
    Orchestration globale

    AIMLNode
    ├── QALoader
    ├── IntentEngine
    ├── DiagnosticsParser
    ├── ParameterExtractor
    └── TTS client

🔥 intent_engine.py
    execute_intent()
    resolve_params()
    handlers (get_led, set_led, get_battery…)

📊 diagnostics_parser.py
    parse /diagnostics_agg
    update self.robot_state
    detect changements importants

🧠 parameter_extractors.py
    extract_color()
    extract_number()
    extract_boolean()
    futur NLP léger

📦 qa_loader.py
    load JSON
    construire index FAISS
    charger dernier index

🧾 constants.py
    seuils température
    seuil batterie
    color_map
    phrases fallback
"""


# 🔁 Obtenir le chemin absolu vers le dossier 'qbo_driver'
package_share = get_package_share_directory('qbo_driver')

# === Configuration des chemins ===
DATA_DIR = os.path.join(package_share, 'config', 'data_pairs')
LLM_DIR = os.path.join(package_share, 'config', 'LLM')
# EMBED_MODEL_NAME = "intfloat/e5-small-v2"
# GEN_MODEL_NAME = "bigscience/bloomz-1b1"

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from qbo_msgs.msg import ListenResult
from qbo_msgs.srv import Text2Speach
from std_srvs.srv import Trigger

import os

# 🔽 Imports internes
from qbo_driver.qbo_aiml.qa_loader import QALoader
from qbo_driver.qbo_aiml.intent_engine import IntentEngine
from qbo_driver.qbo_aiml.parameter_extractors import ParameterExtractor
from qbo_driver.qbo_aiml.diagnostics_parser import DiagnosticsParser
from qbo_driver.qbo_aiml.event_manager import EventManager

package_share = get_package_share_directory('qbo_driver')

DATA_DIR = os.path.join(package_share, 'config', 'data_pairs')
LLM_DIR = os.path.join(package_share, 'config', 'LLM')


class AIMLNode(Node):

    def __init__(self):
        super().__init__('qbo_aiml')

        self.get_logger().info("🚀 Initialisation AIML...")

        # === Modules internes ===
        self.qa_loader = QALoader(LLM_DIR, DATA_DIR, self.get_logger())
        self.intent_engine = IntentEngine(self)
        self.parameter_extractor = ParameterExtractor()
        self.diagnostics_parser = DiagnosticsParser(self)
        self.event_manager = EventManager()

        # === Robot State partagé ===
        self.robot_state = {}
        self.last_detected_params = {}

        # === ROS Interfaces ===
        self.subscription = self.create_subscription(
            ListenResult,
            '/listen',
            self.listen_callback,
            10
        )

        self.speaker_client = self.create_client(
            Text2Speach,
            '/qbo_driver/say_to_TTS'
        )

        self.vector_service = self.create_service(
            Trigger,
            '/vectorize_index',
            self.vectorize_callback
        )

        self.get_logger().info("✅ AIML prêt.")

    # ==============================
    # SERVICE VECTORISATION
    # ==============================
    def vectorize_callback(self, request, response):
        success, message = self.qa_loader.rebuild_index()
        response.success = success
        response.message = message
        return response

    # ==============================
    # CALLBACK LISTEN
    # ==============================
    def listen_callback(self, msg: ListenResult):

        sentence = msg.sentence.lower().strip()
        whisper_conf = msg.confidence

        self.get_logger().info(f"🗣️ Reçu : {sentence} (conf: {whisper_conf:.2f})")

        # 1️⃣ Extraction paramètres
        self.last_detected_params = self.parameter_extractor.extract(sentence)

        # 2️⃣ Recherche QA
        best_item, confidence = self.qa_loader.search(sentence)

        if not best_item:
            self.get_logger().warn("❌ Aucun match.")
            return

        self.get_logger().info(f"🔎 Score FAISS : {confidence:.2f}")

        # 3️⃣ Exécution intent si présent
        intent_result = None
        if "intent" in best_item:
            intent_result = self.intent_engine.execute(
                best_item["intent"],
                self.robot_state,
                self.last_detected_params
            )

        # 4️⃣ Génération réponse
        final_text = self.generate_answer(
            best_item,
            intent_result,
            self.last_detected_params
        )

        self.get_logger().info(f"💬 Réponse : {final_text}")

        # 5️⃣ TTS
        if self.speaker_client.wait_for_service(timeout_sec=1.0):
            req = Text2Speach.Request()
            req.sentence = final_text
            self.speaker_client.call_async(req)

    # ==============================
    # GÉNÉRATION RÉPONSE
    # ==============================
    def generate_answer(self, best_item, intent_result, params):

        import random

        answers = best_item.get("answer", ["Je ne sais pas."])
        if isinstance(answers, str):
            answers = [answers]

        template = random.choice(answers)
        text = template

        # interpolation dynamique simple
        for key, value in params.items():
            text = text.replace(f"{{{key}}}", str(value))

        if intent_result:
            text = text.replace("{result}", str(intent_result))

        return text


# ==================================
# MAIN
# ==================================
def main(args=None):
    rclpy.init(args=args)
    node = AIMLNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⏹️ Arrêt demandé...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
