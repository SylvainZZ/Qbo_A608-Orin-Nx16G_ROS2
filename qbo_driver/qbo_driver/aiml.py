import rclpy
import random
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# from transformers import BitsAndBytesConfig

from qbo_msgs.msg import ListenResult
from qbo_msgs.srv import Text2Speach
from std_srvs.srv import Trigger

import os
import time
from collections import deque

# 🔽 Imports internes
from qbo_driver.qbo_aiml.qa_loader import QALoader
from qbo_driver.qbo_aiml.intent_engine import IntentEngine
from qbo_driver.qbo_aiml.parameter_extractors import ParameterExtractor
from qbo_driver.qbo_aiml.diagnostics_parser import DiagnosticsParser
from qbo_driver.qbo_aiml.event_manager import EventManager
from qbo_driver.qbo_aiml.llm_engine import LLMEngine
from qbo_driver.qbo_aiml.constants import COLOR_MAP

# 🔁 Obtenir le chemin absolu vers le dossier 'qbo_driver'
package_share = get_package_share_directory('qbo_driver')

# === Configuration des chemins ===
DATA_DIR = os.path.join(package_share, 'config', 'data_pairs')
INDEX_DIR = os.path.join(package_share, 'config', 'index')
WATCHERS_DIRS = os.path.join(package_share, 'config', 'others')
path_to_json = os.path.join(package_share, 'config', 'event_phrases.json')
EMBED_MODEL_NAME = "intfloat/e5-small-v2"
GEN_MODEL_NAME = "Qwen/Qwen2-0.5B-Instruct"


class AIMLNode(Node):

    def __init__(self):
        super().__init__('qbo_aiml')

        self.get_logger().info("🚀 Initialisation AIML...")

        # ==============================
        # 🔹 1️⃣ Charger modèles
        # ==============================

        self.qa_loader = QALoader(
            model_name=EMBED_MODEL_NAME,
            logger=self.get_logger(),
            data_dir=DATA_DIR,
            index_dir=INDEX_DIR
        )

        self.qa_diag = QALoader(
            model_name=None,  # on partage le même modèle que qa_loader
            logger=self.get_logger(),
            data_dir=DATA_DIR,
            index_dir=INDEX_DIR
        )
        self.qa_diag.share_embedding(self.qa_loader)

        self.llm_engine = LLMEngine(
            model_name=GEN_MODEL_NAME,
            logger=self.get_logger(),
            enable=True
        )

        self.enable_style_rewrite = False

        # ==============================
        # 🔹 2️⃣ Modules internes
        # ==============================

        self.intent_engine = IntentEngine(self)
        self.parameter_extractor = ParameterExtractor()
        self.diagnostics_parser = DiagnosticsParser(
            node=self,
            watchers_dir=WATCHERS_DIRS
        )
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

        self.qa_loader.load_latest_index(prefix="index")   # index actuel (listen/CLI)
        self.qa_diag.load_latest_index(prefix="diag")      # index diagnostics dédié
        self.THRESHOLDS = {
            "listen": 0.70,      # actions
            "diagnostic": 0.88,  # événements
            "dialog": 0.75       # conversation pure
        }

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
        # ==============================
        # 🔹 6️⃣ TTS Queue
        # ==============================
        self.speech_queue = deque()
        self.speaking = False

        self.speaker_client = self.create_client(
            Text2Speach,
            '/qbo_driver/say_to_TTS'
        )
        # self.create_timer(2.0, self._process_speech_queue)

        # ==============================
        # 🔹 7️⃣ Service de vectorisation à la demande
        # ==============================

        self.vector_service = self.create_service(
            Trigger,
            '/vectorize_index',
            self.vectorize_callback
        )

        self._history_last_len = 0

        self.get_logger().info("✅ AIML prêt.")

    # ==============================
    # SERVICE VECTORISATION
    # ==============================
    def vectorize_callback(self, request, response):

        ok1, msg1 = self.qa_loader.rebuild_index(
            prefix="index",
            filter_fn=None
        )

        ok2, msg2 = self.qa_diag.rebuild_index(
            prefix="diag",
            filter_fn=lambda e: (e.get("meta", {}).get("intent_kind") == "diagnostic")
        )

        response.success = bool(ok1 and ok2)
        response.message = f"index: {msg1} | diag: {msg2}"
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

        # if self.pending_confirmation:

        #     confirm = self.last_detected_params.get("confirm")
        #     self.get_logger().info(
        #         f"🔎 En attente de confirmation... détecté : {confirm}"
        #     )

        #     intent_result = None
        #     event_key = self.pending_confirmation["key"]

        #     if confirm == "yes":

        #         self.get_logger().info("✅ Confirmation reçue")

        #         self.event_manager.mark_executing(event_key)

        #         intent_result = self.intent_engine.execute(
        #             self.pending_confirmation["intent"],
        #             self.robot_state,
        #             self.last_detected_params
        #         )

        #         self.pending_confirmation = None

        #         if intent_result and intent_result.get("status") == "success":

        #             self.enqueue_speech("Calibration lancée avec succès.", priority="info")
        #             self.event_manager.resolve_event(event_key)

        #         else:
        #             self.enqueue_speech("La calibration a échoué.", priority="info")

        #         return

        #     if confirm == "no":

        #         self.enqueue_speech("D'accord, j'annule.", priority="info")

        #         # on snooze pour éviter répétition immédiate
        #         self.event_manager.snooze_event(event_key)

        #         self.pending_confirmation = None
        #         return

        #     # réponse ambiguë
        #     self.enqueue_speech("Tu veux que je le fasse ? Réponds par oui ou non.", priority="info")
        #     return


        # 2️⃣ Recherche QA
        candidates = self.qa_loader.search_topk(sentence, k=5)

        best_item, confidence = self.select_best_candidate(
            candidates,
            sentence,
            self.last_detected_params
        )

        if not best_item:
            self.enqueue_speech("Je ne suis pas sûr de comprendre ta question.", priority="info")
            self.get_logger().warn("❌ Aucun match.")
            return

        if "intent" in best_item:
            threshold = self.THRESHOLDS["listen"]
        else:
            threshold = self.THRESHOLDS["dialog"]

        if confidence < threshold:
            self.enqueue_speech("Je ne sais pas répondre à ça pour le moment.", priority="info")
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

        # 4️⃣ Génération réponse de base
        base_answer = self.generate_answer(
            best_item,
            intent_result,
            self.last_detected_params
        )

        meta = best_item.get("meta", {})
        intent_kind = meta.get("intent_kind", "")
        risk = meta.get("risk", "low")

        # 🔹 Reformulation uniquement pour conversation / explain
        if (
            self.enable_style_rewrite
            and risk == "low"
            and intent_kind in ["conversation", "explain"]
        ):
            self.get_logger().info(f"🧠 Réponse avant reformulation: {base_answer}")
            final_text = self.llm_engine.rewrite(base_answer)
        else:
            final_text = base_answer

        # 5️⃣ TTS
        self.enqueue_speech(final_text, priority="info")

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
        if intent_result is not None:

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
        best_score = -1.0

        for c in candidates:
            item = c["item"]
            score = c["score"]
            # print("Score FAISS brut:", c["score"])

            # 🔹 Bonus si intent présent et phrase ressemble à commande
            # if "intent" in item and any(v in sentence for v in ["allume", "mets", "éteins", "lance"]):
            #     score += 0.15

            # 🔹 Bonus si slot détecté et QA utilise ce slot
            # if "color" in params and "{color}" in item.get("question", ""):
            #     score += 0.20

            # 🔹 Bonus si question pure sans intent pour phrase interrogative
            # if "intent" not in item and sentence.endswith("?"):
            #     score += 0.10

            if score > best_score:
                best = item
                best_score = score

        # seuil minimum
        if best is None or best_score < 0.50:
            return None, 0.0

        return best, best_score

    # ==============================
    # TRAITEMENT ÉVÉNEMENTS DIAGNOSTICS
    # ==============================

    def process_events(self):

        # self.get_logger().info("⏱️ Vérification événements diagnostics...")

        # --- Debug: afficher les N dernières transitions ---
        history = self.event_manager.get_history()
        self._log_history_tail(history, n=5)

        # --- Traiter les diagnostics actuels ---
        event = self.event_manager.get_next_event()
        if not event:
            return

        key = event["key"]
        message = event["message"]
        severity = event["severity"]

        self.get_logger().info(f"🧠 Traitement event: {key}")

        # requête enrichie (message seul est trop faible)
        query = f"{key} | {severity} | {message}"

        candidates = self.qa_diag.search_topk(query, k=20)
        route = self._route_from_event(key, message)

        self.get_logger().info(f"🔎 THRESHOLDS: {self.THRESHOLDS['diagnostic']}")

        best_item, confidence = self._select_diag_candidate(
            candidates,
            route,
            threshold=self.THRESHOLDS["diagnostic"],
            margin=0.0
        )

        if not best_item:
            # fallback silencieux : pas de proposed, pas de TTS, snooze plus long
            self.get_logger().info(
                f"⚠ Aucun QA diagnostic fiable (best={confidence:.2f}). Snooze long."
            )
            self.event_manager.snooze_event(key, 120)
            return

        final_text = self.generate_answer(best_item, None, {})

        # On notifie seulement
        self.event_manager._transition(event, "notified")
        self.event_manager.snooze_event(key, 60)

        self.enqueue_speech(final_text, priority="info")

        # On stocke le best_item pour plus tard
        event["diag_item"] = best_item

    def propose_event_action(self, key, event, best_item):

        if event["state"] != "notified":
            return

        if "intent" not in best_item:
            return

        self.event_manager._transition(event, "proposed")

        question = self.generate_answer(best_item, None, {})
        self.enqueue_speech(question, priority="info")

        self.pending_confirmation = {
            "intent": best_item["intent"],
            "key": key,
            "prompt": question
        }

        self.event_manager.snooze_event(key, 30)

    # ============================
    # ENONCE L'INFORMATION
    # ============================

    def enqueue_speech(self, text, priority="normal"):
        if not text:
            return
        item = {
            "text": text,
            "priority": priority
        }
        # priorité haute passe devant
        if priority == "critical":
            self.speech_queue.appendleft(item)
        else:
            self.speech_queue.append(item)

        self._process_speech_queue()

    def _process_speech_queue(self):
        if self.speaking:
            return
        if not self.speech_queue:
            return
        item = self.speech_queue.popleft()
        self.speaking = True

        self._speak(item["text"])

    def _speak(self, text):
        self.get_logger().info(f"💬 speaking: {text}")
        if not hasattr(self, "speaker_client"):
            self.speaking = False
            self._process_speech_queue()
            return

        if not self.speaker_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().debug("TTS indisponible (mode silencieux).")
            self.speaking = False
            self._process_speech_queue()
            return

        req = Text2Speach.Request()
        req.sentence = text

        future = self.speaker_client.call_async(req)
        future.add_done_callback(self._on_tts_done)

    def _on_tts_done(self, future):
        try:
            future.result()
        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")

        self.speaking = False
        self._process_speech_queue()

    # ============================
    # LOGGING HISTORIQUE
    # ============================

    def _log_history_tail(self, history, n=5):

        if len(history) == self._history_last_len:
            return  # rien de nouveau

        self._history_last_len = len(history)

        tail = history[-n:]
        self.get_logger().info(f"📜 Historique (+{len(tail)} derniers):")

        for h in tail:
            ts = time.strftime("%H:%M:%S", time.localtime(h["timestamp"]))
            self.get_logger().info(
                f"  [{ts}] {h['transition']} | {h['key']} | {h['severity']} | {h.get('message','')}"
            )

    # ============================
    # RECHERCHE DIAGNOSTIC APPROFONDIE
    # ============================
    def _route_from_event(self, key: str, message: str) -> dict:
        # key = "hardware|category"
        self.get_logger().info(f"🔍 Routing event: {key} | {message}")
        category = key.split("|", 1)[1] if "|" in key else key
        s = (category + " " + message).lower()

        # mapping très simple (tu pourras raffiner)
        if "dynamixel" in s or "torque" in s or "motor" in s:
            self.get_logger().info("⚠ Routing vers moteurs.")
            return {"domain": "motors", "component": "dynamixel"}
        if "imu" in s:
            self.get_logger().info("⚠ Routing vers IMU.")
            return {"domain": "imu", "component": "imu"}
        if "battery" in s or "voltage" in s:
            self.get_logger().info("⚠ Routing vers batterie.")
            return {"domain": "battery", "component": "battery"}
        if "nose" in s or "led" in s:
            self.get_logger().info("⚠ Routing vers nez.")
            return {"domain": "nose", "component": "nose_led"}
        self.get_logger().info("⚠ Routing générique appliqué.")
        return {"domain": None, "component": None}

    def _select_diag_candidate(self, candidates, route, threshold, margin=0.0):
        # filtre strict meta.intent_kind == diagnostic + domain/component si dispo
        self.get_logger().info(f"🔎 {len(candidates)} candidats diagnostics trouvés, filtrage en cours...")
        self.get_logger().info(f"🔎 th: {threshold} | margin: {margin} | route: {route}")
        filtered = []
        for c in candidates:
            item = c["item"]
            meta = item.get("meta", {})
            # self.get_logger().info(
            #     f"DEBUG: meta={meta} route={route} score={c['score']}"
            # )
            if meta.get("intent_kind") != "diagnostic":
                continue
            if route.get("domain") and meta.get("domain") != route["domain"]:
                continue
            if route.get("component") and meta.get("component"):
                if route["component"] != meta["component"]:
                    # fallback souple : accepter si domain match fort
                    if meta.get("domain") != route.get("domain"):
                        continue
            filtered.append(c)

        if not filtered:
            self.get_logger().info("⚠ Aucun candidat ne correspond au filtrage diagnostic.")
            return None, 0.0

        filtered.sort(key=lambda x: x["score"], reverse=True)
        best = filtered[0]
        if best["score"] < threshold:
            return None, best["score"]

        if len(filtered) >= 2:
            if (best["score"] - filtered[1]["score"]) < margin:
                return None, best["score"]

        self.get_logger().info(f"DEBUG: {len(filtered)} candidats après filtrage")

        return best["item"], best["score"]

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
