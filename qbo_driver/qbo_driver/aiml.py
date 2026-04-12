import rclpy
import random
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from qbo_msgs.srv import GenerateText
from std_srvs.srv import Trigger

import os
import json

# 🔽 Imports internes — AIML conserve uniquement RAG + LLM
# Les modules intent_engine, parameter_extractors, diagnostics_parser
# et event_manager sont désormais gérés par le SBE.
from qbo_driver.qbo_aiml.qa_loader import QALoader
from qbo_driver.qbo_aiml.llm_engine import LLMEngine

# 🔁 Obtenir le chemin absolu vers le dossier 'qbo_driver'
package_share = get_package_share_directory('qbo_driver')

# === Configuration des chemins ===
DATA_DIR         = os.path.join(package_share, 'config', 'data_pairs')
INDEX_DIR        = os.path.join(package_share, 'config', 'index')
EMBED_MODEL_NAME = "intfloat/e5-small-v2"
GEN_MODEL_NAME   = "Qwen/Qwen2-0.5B-Instruct"

# ─────────────────────────────────────────────────────────────────────────────
# TABLE DE ROUTAGE DIAGNOSTIC
# Dérivée des fichiers QA réels (config/data_pairs/).
# Chaque règle est un dict avec des clés optionnelles :
#   "hw"  : sous-chaînes du hardware_id  (key.split("|")[0])
#   "cat" : sous-chaînes de la catégorie (key.split("|")[1])
#   "msg" : sous-chaînes du message ROS
# Toutes les sous-chaînes d'un même champ sont évaluées en OU (any).
# Les règles sont testées dans l'ordre — la première correspondance gagne.
# ─────────────────────────────────────────────────────────────────────────────
_ROUTE_TABLE = [
    # ── Batterie  (Qboard_3 / ctrl_battery) ──────────────────────────────
    {"hw":  ["qboard_3"],
     "domain": "battery",        "component": "Qboard_3"},
    {"msg": ["battery", "charge mode", "power pc"],
     "domain": "battery",        "component": "Qboard_3"},

    # ── IMU  (Qboard_4 / imu_controller) ─────────────────────────────────
    {"hw":  ["qboard_4"],
     "domain": "imu",            "component": "Qboard_4"},
    {"msg": ["imu", "gyroscope", "accelerometer"],
     "domain": "imu",            "component": "Qboard_4"},

    # ── Dynamixel pan (prioritaire sur tilt) ─────────────────────────────
    {"hw":  ["dynamixel"], "cat": ["pan"],
     "domain": "head_pan_joint",  "component": "dynamixel"},
    {"hw":  ["dynamixel"], "msg": ["pan"],
     "domain": "head_pan_joint",  "component": "dynamixel"},

    # ── Dynamixel tilt ────────────────────────────────────────────────────
    {"hw":  ["dynamixel"], "cat": ["tilt"],
     "domain": "head_tilt_joint", "component": "dynamixel"},
    {"hw":  ["dynamixel"], "msg": ["tilt"],
     "domain": "head_tilt_joint", "component": "dynamixel"},

    # ── Dynamixel générique (torque / register / temperature / voltage) ───
    {"hw":  ["dynamixel"],
     "domain": "head_pan_joint",  "component": "dynamixel"},
    {"msg": ["torque", "register read", "position error"],
     "domain": "head_pan_joint",  "component": "dynamixel"},

    # ── LCD  (LCD / lcd_controller) ───────────────────────────────────────
    {"hw":  ["lcd"],
     "domain": "lcd",             "component": "LCD"},
    {"msg": ["lcd", "i2c"],
     "domain": "lcd",             "component": "LCD"},

    # ── Mouth  (Qboard_5 / mouth_controller) ─────────────────────────────
    {"hw":  ["qboard_5"], "cat": ["mouth"],
     "domain": "mouth",           "component": "Qboard_5"},
    {"msg": ["mouth"],
     "domain": "mouth",           "component": "Qboard_5"},
    # Qboard_5 sans discrimination de catégorie → mouth (seul diag connu)
    {"hw":  ["qboard_5"],
     "domain": "mouth",           "component": "Qboard_5"},

    # ── Sensors  (Qboard_1 / sensor_controller — prioritaire sur motor) ──
    {"hw":  ["qboard_1"], "cat": ["sensor"],
     "domain": "sensors",         "component": "Qboard_1"},
    {"msg": ["sensor"],
     "domain": "sensors",         "component": "Qboard_1"},

    # ── Motor / Base  (Qboard_1 / ctrl_base) ─────────────────────────────
    {"hw":  ["qboard_1"],
     "domain": "motor",           "component": "Qboard_1"},
    {"msg": ["motor", "odometry"],
     "domain": "motor",           "component": "Qboard_1"},

    # ── Hardware Orin  (orin-nx-16g) ──────────────────────────────────────
    {"hw":  ["orin"],
     "domain": "hardware",        "component": "orin-nx-16g"},
    {"msg": ["cpu", "gpu", "ram", "fan", "overheat", "network",
              "internet", "power overload", "power data"],
     "domain": "hardware",        "component": "orin-nx-16g"},
]


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
        # 🔹 2️⃣ Configuration seuils
        # ==============================

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
        # 🔹 4️⃣ Service de vectorisation à la demande
        # ==============================

        self.vector_service = self.create_service(
            Trigger,
            '/aiml/vectorize_index',
            self.vectorize_callback
        )

        # ==============================
        # 🔹 5️⃣ Service de génération de texte
        # ==============================

        self.generate_service = self.create_service(
            GenerateText,
            '/aiml/generate_text',
            self.generate_text_callback
        )

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
    # SERVICE GENERATION DE TEXTE
    # ==============================
    def generate_text_callback(self, request, response):

        try:

            # =========================
            # CAS 1 : CONVERSATION
            # =========================
            if request.type == "conversation":

                sentence = request.text.lower().strip()

                # Les paramètres (couleur, nombre, confirmation…) sont
                # extraits par le SBE et transmis optionnellement via context_json.
                # Format attendu : {"params": {"color": 2, "color_name": "bleu", ...}}
                params = {}
                if request.context_json:
                    try:
                        ctx = json.loads(request.context_json)
                        params = ctx.get("params", {})
                    except json.JSONDecodeError:
                        pass

                candidates = self.qa_loader.search_topk(sentence, k=5)

                best_item, confidence = self.select_best_candidate(
                    candidates,
                    sentence,
                    params
                )

                if not best_item:
                    response.success = True
                    response.response_text = "Je ne suis pas sûr de comprendre."
                    response.intent_json = ""
                    return response

                # Seuil selon présence d'un intent dans le QA
                if "intent" in best_item:
                    threshold = self.THRESHOLDS["listen"]
                else:
                    threshold = self.THRESHOLDS["dialog"]

                if confidence < threshold:
                    response.success = True
                    response.response_text = "Je ne sais pas répondre à ça."
                    response.intent_json = ""
                    return response

                # Si le QA contient un intent, on l'embarque dans la réponse
                # pour que le SBE puisse l'exécuter après réception.

                base_answer = self.generate_answer(best_item, None, params)

                response.success = True
                response.response_text = base_answer
                # Renvoi du payload intent pour que le SBE l'exécute
                if "intent" in best_item:
                    response.intent_json = json.dumps(best_item["intent"])
                else:
                    response.intent_json = ""
                return response

            # =========================
            # CAS 2 : DIAGNOSTIC
            # =========================
            elif request.type == "diagnostic":

                data = json.loads(request.context_json)

                key = data.get("key", "")
                severity = data.get("severity", "")
                message = data.get("message", "")

                query = self.build_diagnostic_query(data)

                candidates = self.qa_diag.search_topk(query, k=20)
                route = self._route_from_event(key, message)

                best_item, confidence = self._select_diag_candidate(
                    candidates,
                    route,
                    threshold=self.THRESHOLDS["diagnostic"],
                    margin=0.0
                )

                if not best_item:
                    response.success = False
                    response.response_text = ""
                    response.intent_json = ""
                    return response

                final_text = self.generate_answer(best_item, None, {})

                response.success = True
                response.response_text = final_text
                response.intent_json = ""
                return response

            else:
                response.success = False
                response.response_text = "Type inconnu"
                response.intent_json = ""
                return response

        except Exception as e:
            self.get_logger().error(f"AIML error: {e}")
            response.success = False
            response.response_text = ""
            response.intent_json = ""
            return response

    def build_diagnostic_query(self, data):

        key = data.get("key", "")
        severity = data.get("severity", "")
        message = data.get("message", "")

        # split intelligent
        parts = key.split("|")
        hardware = parts[0] if len(parts) > 0 else ""
        component = parts[1] if len(parts) > 1 else ""

        query = f"""
        diagnostic {severity}
        robot qbo
        hardware {hardware}
        component {component}
        issue {message}
        """

        return query.strip()
    # ==============================
    # GÉNÉRATION RÉPONSE
    # ==============================
    def generate_answer(self, qa_item, intent_result, params):
        """
        Remplit le template de réponse avec les paramètres reçus.
        - intent_result : réservé (toujours None depuis que le SBE exécute les intents).
        - params       : dict fourni par le SBE via context_json (couleur, nombre, …).
        """
        answers = qa_item.get("answer")

        if not answers:
            return ""

        if not isinstance(answers, list):
            answers = [answers]

        answer_template = random.choice(answers)

        # Injection des valeurs dynamiques (provenant du SBE via params)
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

            if score > best_score:
                best = item
                best_score = score

        # seuil minimum
        if best is None or best_score < 0.50:
            return None, 0.0

        return best, best_score

    # ============================
    # RECHERCHE DIAGNOSTIC APPROFONDIE
    # ============================
    def _route_from_event(self, key: str, message: str) -> dict:
        """
        Résout le domaine et le composant à partir de la clé ROS et du message.
        key format : "hardware_id|category"  (ex: "Qboard_3|Battery Controller")
        Utilise _ROUTE_TABLE définie au niveau module — dérivée des fichiers QA.
        """
        parts    = key.split("|", 1)
        hw_raw   = parts[0].strip() if len(parts) > 0 else ""
        cat_raw  = parts[1].strip() if len(parts) > 1 else ""

        hw  = hw_raw.lower()
        cat = cat_raw.lower()
        msg = message.lower()

        self.get_logger().info(f"🔍 Routing — hw='{hw_raw}' cat='{cat_raw}' msg='{message}'")

        for rule in _ROUTE_TABLE:
            # Vérification hardware (OU sur la liste)
            if "hw" in rule:
                if not any(kw in hw for kw in rule["hw"]):
                    continue
            # Vérification catégorie (OU sur la liste)
            if "cat" in rule:
                if not any(kw in cat for kw in rule["cat"]):
                    continue
            # Vérification message (OU sur la liste)
            if "msg" in rule:
                if not any(kw in msg for kw in rule["msg"]):
                    continue

            domain    = rule["domain"]
            component = rule["component"]
            self.get_logger().info(f"→ Route : domain='{domain}' component='{component}'")
            return {"domain": domain, "component": component}

        self.get_logger().info("⚠ Aucune règle de routing — fallback générique.")
        return {"domain": None, "component": None}

    def _select_diag_candidate(self, candidates, route, threshold, margin=0.0):
        """
        Sélectionne le meilleur candidat diagnostic parmi les résultats FAISS.

        Filtrage en 3 passes ordonnées :
          1. Strict   : intent_kind == "diagnostic"
          2. Préféré  : domain == route["domain"]  (si défini)
                        + bonus léger si component correspond aussi
          3. Fallback : si aucun match sur le domain, on réessaie sans filtre
                        domain avec un seuil rehaussé (+0.05) pour limiter
                        les faux positifs.

        margin > 0 : rejette si les deux meilleurs scores sont trop proches
                     (ambiguïté). Désactivé à 0.0 (valeur par défaut).
        """
        r_domain    = route.get("domain")
        r_component = route.get("component")

        # ── Passe 0 : garder uniquement les entrées diagnostic ────────────
        diag_only = [
            c for c in candidates
            if c["item"].get("meta", {}).get("intent_kind") == "diagnostic"
        ]

        self.get_logger().info(
            f"🔎 {len(candidates)} candidats → {len(diag_only)} diagnostic(s) "
            f"| domain='{r_domain}' component='{r_component}' th={threshold}"
        )

        if not diag_only:
            self.get_logger().info("⚠ Aucun candidat intent_kind=diagnostic.")
            return None, 0.0

        # ── Passe 1 : filtrage strict par domain ──────────────────────────
        if r_domain:
            by_domain = [
                c for c in diag_only
                if c["item"].get("meta", {}).get("domain") == r_domain
            ]
        else:
            by_domain = diag_only   # pas de route connue → tous les diagnostics

        # ── Passe 2 : tri avec bonus composant ────────────────────────────
        # Bonus léger (+0.02) si le component correspond exactement :
        # permet de départager deux scores FAISS très proches sans bloquer.
        COMPONENT_BONUS = 0.02

        def adjusted_score(c):
            s = c["score"]
            if r_component and \
               c["item"].get("meta", {}).get("component") == r_component:
                s += COMPONENT_BONUS
            return s

        if by_domain:
            by_domain.sort(key=adjusted_score, reverse=True)
            best       = by_domain[0]
            best_score = best["score"]   # score brut pour le threshold

            # Vérification seuil
            if best_score < threshold:
                self.get_logger().info(
                    f"⚠ Score insuffisant ({best_score:.3f} < {threshold})."
                )
                return None, best_score

            # Vérification ambiguïté (uniquement si margin > 0)
            if margin > 0.0 and len(by_domain) >= 2:
                second_score = by_domain[1]["score"]
                if (best_score - second_score) < margin:
                    self.get_logger().info(
                        f"⚠ Ambiguïté ({best_score:.3f} vs {second_score:.3f}"
                        f" < margin {margin})."
                    )
                    return None, best_score

            self.get_logger().info(
                f"✅ Match : {best_score:.3f} "
                f"| {best['item'].get('meta', {}).get('domain')} "
                f"/ {best['item'].get('meta', {}).get('component')}"
            )
            return best["item"], best_score

        # ── Passe 3 : fallback sans filtre domain (seuil rehaussé) ───────
        # Atteint uniquement si r_domain était défini mais aucun candidat
        # ne correspondait (QA manquant ou routing imprécis).
        fallback_threshold = min(threshold + 0.05, 0.98)
        self.get_logger().info(
            f"⚠ Aucun candidat pour domain='{r_domain}' "
            f"— fallback global (th={fallback_threshold:.2f})."
        )
        diag_only.sort(key=lambda c: c["score"], reverse=True)
        best = diag_only[0]

        if best["score"] < fallback_threshold:
            self.get_logger().info(
                f"⚠ Fallback insuffisant ({best['score']:.3f} < {fallback_threshold:.2f})."
            )
            return None, best["score"]

        self.get_logger().info(
            f"✅ Match fallback : {best['score']:.3f} "
            f"| {best['item'].get('meta', {}).get('domain')}"
        )
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
