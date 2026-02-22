import rclpy
import random
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from transformers import AutoTokenizer, AutoModel

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
from qbo_driver.qbo_aiml.speech_generator import SpeechGenerator
from qbo_driver.qbo_aiml.constants import COLOR_MAP

# 🔁 Obtenir le chemin absolu vers le dossier 'qbo_driver'
package_share = get_package_share_directory('qbo_driver')

# === Configuration des chemins ===
DATA_DIR = os.path.join(package_share, 'config', 'data_pairs')
LLM_DIR = os.path.join(package_share, 'config', 'LLM')
path_to_json = os.path.join(package_share, 'config', 'event_phrases.json')
EMBED_MODEL_NAME = "intfloat/e5-small-v2"
# GEN_MODEL_NAME = "bigscience/bloomz-1b1"


class AIMLNode(Node):

    THRESHOLDS = {
        "listen": 0.70,      # actions
        "diagnostic": 0.80,  # événements
        "dialog": 0.75       # conversation pure
    }

    def __init__(self):
        super().__init__('qbo_aiml')

        self.get_logger().info("🚀 Initialisation AIML...")

        # ==============================
        # 🔹 1️⃣ Charger modèles embedding
        # ==============================

        self.get_logger().info("🔄 Chargement modèle embedding...")

        self.embed_tokenizer = AutoTokenizer.from_pretrained(EMBED_MODEL_NAME)
        self.embed_model = AutoModel.from_pretrained(EMBED_MODEL_NAME).eval()

        # ==============================
        # 🔹 2️⃣ Modules internes
        # ==============================

        self.qa_loader = QALoader(
            LLM_DIR,
            self.embed_model,
            self.embed_tokenizer,
            self.get_logger(),
            DATA_DIR
        )

        self.intent_engine = IntentEngine(self)
        self.parameter_extractor = ParameterExtractor()
        self.diagnostics_parser = DiagnosticsParser(self)
        self.event_manager = EventManager(
            cooldown_default=30,
            logger=self.get_logger()
        )

        self.event_timer = self.create_timer(10.0, self.process_events)
        self.pending_confirmation = None

        # Option CLI pour tests rapides
        self.enable_cli = True
        if self.enable_cli:
            import threading
            threading.Thread(target=self.cli_loop, daemon=True).start()

        # ==============================
        # 🔹 3️⃣ Charger index RAG
        # ==============================

        self.qa_loader.load_latest_index()

        # ==============================
        # 🔹 4️⃣ Robot state
        # ==============================

        self.robot_state = {}
        self.last_detected_params = {}

        # ==============================
        # 🔹 5️⃣ ROS interfaces
        # ==============================

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
    # CALLBACK LISTEN OU CLI
    # ==============================
    def listen_callback(self, msg: ListenResult):

        sentence = msg.sentence.lower().strip()
        whisper_conf = msg.confidence

        self.get_logger().info(f"🗣️ Reçu : {sentence} (conf: {whisper_conf:.2f})")

        # 1️⃣ Extraction paramètres
        self.last_detected_params = self.parameter_extractor.extract(sentence)
        self.get_logger().info(f"🧪 extract() -> {self.last_detected_params}")

        if self.pending_confirmation:

            confirm = self.last_detected_params.get("confirm")
            self.get_logger().info(
                f"🔎 En attente de confirmation... détecté : {confirm}"
            )

            intent_result = None
            event_key = self.pending_confirmation["key"]

            if confirm == "yes":

                self.get_logger().info("✅ Confirmation reçue")

                self.event_manager.mark_executing(event_key)

                intent_result = self.intent_engine.execute(
                    self.pending_confirmation["intent"],
                    self.robot_state,
                    self.last_detected_params
                )

                self.pending_confirmation = None

                if intent_result and intent_result.get("status") == "success":

                    self.say("Calibration lancée avec succès.")
                    self.event_manager.resolve_event(event_key)

                else:
                    self.say("La calibration a échoué.")

                return

            if confirm == "no":

                self.say("D'accord, j'annule.")

                # on snooze pour éviter répétition immédiate
                self.event_manager.snooze_event(event_key)

                self.pending_confirmation = None
                return

            # réponse ambiguë
            self.say("Tu veux que je le fasse ? Réponds par oui ou non.")
            return


        # 2️⃣ Recherche QA
        candidates = self.qa_loader.search_topk(sentence, k=5)

        best_item, confidence = self.select_best_candidate(
            candidates,
            sentence,
            self.last_detected_params
        )

        if not best_item:
            self.say("Je ne suis pas sûr de comprendre ta question.")
            self.get_logger().warn("❌ Aucun match.")
            return

        if "intent" in best_item:
            threshold = self.THRESHOLDS["listen"]
        else:
            threshold = self.THRESHOLDS["dialog"]

        if confidence < threshold:
            self.say("Je ne sais pas répondre à ça pour le moment.")
            self.get_logger().warn(f"❌ Score trop faible ({confidence:.2f})")
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

        # 5️⃣ TTS
        self.say(final_text)

    def cli_loop(self):
        while rclpy.ok():
            try:
                s = input("⌨️  CLI> ").strip()
                if not s:
                    continue
                # on simule un message listen
                class Dummy:
                    sentence = s
                    confidence = 1.0
                self.listen_callback(Dummy())
            except EOFError:
                return
            except Exception as e:
                self.get_logger().error(f"CLI error: {e}")

    # ==============================
    # GÉNÉRATION RÉPONSE
    # ==============================
    def generate_answer(self, qa_item, intent_result, params):

        answers = qa_item.get("answer")

        if not answers:
            return ""

        if not isinstance(answers, list):
            answers = [answers]

        answer_template = random.choice(answers)

        # 🔹 Si on a un retour d'intent (lecture état)
        if intent_result:

            # Injecter toutes les valeurs dynamiques
            for key, value in intent_result.items():
                answer_template = answer_template.replace(
                    "{" + key + "}",
                    str(value)
                )

        # 🔹 Sinon on est dans une commande utilisateur
        else:

            # mapping couleur demandée
            if "color" in params:
                color_code = params["color"]
                params["color_label"] = COLOR_MAP.get(
                    color_code,
                    str(color_code)
                )

            for key, value in params.items():
                answer_template = answer_template.replace(
                    "{" + key + "}",
                    str(value)
                )

        return answer_template


    def select_best_candidate(self, candidates, sentence, params):

        best = None
        best_score = 0

        for c in candidates:

            item = c["item"]
            score = c["score"]
            print("Score FAISS brut:", c["score"])

            # 🔹 Bonus si intent présent et phrase ressemble à commande
            if "intent" in item and any(v in sentence for v in ["allume", "mets", "éteins", "lance"]):
                score += 0.15

            # 🔹 Bonus si slot détecté et QA utilise ce slot
            if "color" in params and "{color}" in item.get("question", ""):
                score += 0.20

            # 🔹 Bonus si question pure sans intent pour phrase interrogative
            if "intent" not in item and sentence.endswith("?"):
                score += 0.10

            if score > best_score:
                best = item
                best_score = score

            if best_score < 0.50:
                return None, 0.0

        return best, best_score

    # ==============================
    # TRAITEMENT ÉVÉNEMENTS DIAGNOSTICS
    # ==============================

    def process_events(self):

        event = self.event_manager.get_next_event()
        if not event:
            return

        key = event["key"]
        message = event["message"]
        severity = event["severity"]

        self.event_manager.snooze_event(key, 20)

        self.get_logger().info(f"🧠 Traitement event: {key}")

        best_item, confidence = self.qa_loader.search(message)

        if confidence < self.THRESHOLDS["diagnostic"]:
            self.get_logger().info(
                f"⚠ Confidence trop faible pour event ({confidence:.2f})"
            )
            return

        final_text = self.generate_answer(
            best_item,
            None,
            {}
        )

        self.say(final_text)

        if "intent" in best_item:

            self.pending_confirmation = {
                "intent": best_item["intent"],
                "key": key,
                "prompt": final_text
            }

            self.event_manager.snooze_event(key)

            self.get_logger().info(
                f"🟡 Pending confirmation set for {key}"
            )

    # ============================
    # ENONCE L'INFORMATION
    # ============================

    def say(self, text):

        if not text:
            return

        self.get_logger().info(f"💬 {text}")

        if not hasattr(self, "speaker_client"):
            return

        if self.speaker_client.wait_for_service(timeout_sec=0.5):
            req = Text2Speach.Request()
            req.sentence = text
            self.speaker_client.call_async(req)
        else:
            self.get_logger().debug("TTS non disponible.")


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
