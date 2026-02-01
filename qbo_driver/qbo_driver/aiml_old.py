import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
# from std_msgs.msg import String
from qbo_msgs.msg import ListenResult
from qbo_msgs.srv import Text2Speach
from std_srvs.srv import Trigger
from datetime import datetime
import os
import json
import faiss
import torch
from transformers import AutoTokenizer, AutoModel, AutoModelForCausalLM
import threading
import re
import random



# 🔁 Obtenir le chemin absolu vers le dossier 'qbo_driver'
package_share = get_package_share_directory('qbo_driver')

# === Configuration des chemins ===
DATA_DIR = os.path.join(package_share, 'config', 'data_pairs')
LLM_DIR = os.path.join(package_share, 'config', 'LLM')
EMBED_MODEL_NAME = "intfloat/e5-small-v2"
GEN_MODEL_NAME = "bigscience/bloomz-1b1"

class AIMLNode(Node):
    def __init__(self):
        super().__init__('qbo_aiml')
        self.stop_evt = threading.Event()

        # Souscription au texte de Whisper
        self.subscription = self.create_subscription(
            ListenResult,
            '/listen',
            self.listen_callback,
            10)

        # Client pour faire parler Qbo
        self.speaker_client = self.create_client(Text2Speach, '/qbo_driver/say_to_TTS')

        # Service de vectorisation
        self.vector_service = self.create_service(Trigger, '/vectorize_index', self.vectorize_callback)

        # Charger les modèles au démarrage
        self.get_logger().info("Chargement des modèles...")
        self.embed_tokenizer = AutoTokenizer.from_pretrained(EMBED_MODEL_NAME)
        self.embed_model = AutoModel.from_pretrained(EMBED_MODEL_NAME).eval()

        self.gen_tokenizer = AutoTokenizer.from_pretrained(GEN_MODEL_NAME)
        self.gen_model = AutoModelForCausalLM.from_pretrained(GEN_MODEL_NAME).eval()

        # Charger dernier index disponible
        self.index = None
        self.qa_pairs = []
        self.load_latest_index()

        # État du robot
        self.robot_state = {
            "nose_color": "0",
            "battery_level": "13.2",
            "external_power": True
        }

        self.last_detected_params = {}

    def load_latest_index(self):
        try:
            files = [f for f in os.listdir(LLM_DIR) if f.endswith('.faiss')]
            if not files:
                self.get_logger().warn("Aucun index FAISS trouvé dans config/LLM/")
                return
            files.sort(reverse=True)
            latest_faiss = files[0]
            json_name = latest_faiss.replace('.faiss', '.json')

            self.index = faiss.read_index(os.path.join(LLM_DIR, latest_faiss))
            with open(os.path.join(LLM_DIR, json_name), 'r', encoding='utf-8') as f:
                self.qa_pairs = json.load(f)

            self.get_logger().info(f"Index chargé : {latest_faiss}")
        except Exception as e:
            self.get_logger().error(f"Erreur lors du chargement de l'index : {e}")

    def vectorize_callback(self, request, response):
        try:
            all_data = []
            for fname in os.listdir(DATA_DIR):
                if fname.endswith(".json"):
                    with open(os.path.join(DATA_DIR, fname), 'r', encoding='utf-8') as f:
                        all_data.extend(json.load(f))

            texts = [f"{item['question']}, {item['answer']}" for item in all_data]
            inputs = self.embed_tokenizer(["passage: " + t for t in texts], padding=True, truncation=True, return_tensors="pt")
            with torch.no_grad():
                vectors = self.embed_model(**inputs).last_hidden_state[:, 0].numpy()

            index = faiss.IndexFlatL2(vectors.shape[1])
            index.add(vectors)

            now = datetime.now().strftime("%Y_%m_%d_%H_%M")
            index_file = f"index_{now}.faiss"
            json_file = f"index_{now}.json"

            faiss.write_index(index, os.path.join(LLM_DIR, index_file))
            with open(os.path.join(LLM_DIR, json_file), 'w', encoding='utf-8') as f:
                json.dump(all_data, f, ensure_ascii=False, indent=2)

            self.get_logger().info(f"Index sauvegardé : {index_file}")
            response.success = True
            response.message = f"Index créé avec {len(all_data)} entrées."

            # Recharger l'index fraîchement créé
            self.load_latest_index()

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"Erreur vectorisation : {e}")

        return response

    def execute_intent(self, intent: dict):
        try:
            action = intent.get("action", "")

            # 1. INTENT "get_led" → lire self.robot_state
            if action == "get_led":
                color = self.robot_state.get("nose_color", "0")
                color_map = {
                    "0": "éteint",
                    "1": "rouge",
                    "2": "bleu",
                    "3": "violet",
                    "4": "vert",
                    "5": "jaune",
                    "6": "magenta",
                    "7": "blanc"
                }
                return color_map.get(str(color), f"code {color}")

            # 2. INTENT "set_led" → envoyer une commande et mettre à jour l’état
            if action == "set_led":
                # 🔁 Résolution des paramètres
                params = intent.get("params", {})
                resolved_params = {}

                for key, value in params.items():
                    if value == "{color}":
                        resolved_params[key] = self.last_detected_params.get("color", 0)
                    else:
                        resolved_params[key] = value

                # 🧠 Mise à jour de l'état
                color_code = resolved_params.get("color", 0)
                self.robot_state["nose_color"] = str(color_code)

                # 2.1 Publication ROS Topic
                if intent.get("type") == "topic":
                    from rosidl_runtime_py.utilities import get_message
                    msg_class = get_message(intent["msg_type"])
                    msg = msg_class(**resolved_params)
                    pub = self.create_publisher(msg_class, intent["target"], 10)
                    pub.publish(msg)
                    self.get_logger().info(f"✅ Topic publié sur {intent['target']} avec {resolved_params}")
                    return None

            elif action == "test_leds":
                # 2.2 Appel ROS Service
                if intent.get("type") == "service":
                    from rosidl_runtime_py.utilities import get_service
                    srv_class = get_service(intent["msg_type"])
                    client = self.create_client(srv_class, intent["target"])
                    if client.wait_for_service(timeout_sec=2.0):
                        request = srv_class.Request()
                        client.call_async(request)
                        self.get_logger().info(f"✅ Service appelé {intent['target']}")
                    else:
                        self.get_logger().warn(f"❌ Service {intent['target']} non disponible.")
                    return None

        except Exception as e:
            self.get_logger().error(f"❌ Erreur exécution intent : {e}")
            return None



    # def listen_callback(self, msg: ListenResult):

    #     sentence = msg.sentence.lower().strip()
    #     confidence = msg.confidence
    #     self.get_logger().info(sentence)

    #     if not self.index:
    #         self.get_logger().warn("Aucun index chargé, vectorisez d'abord.")
    #         return

    #     input = self.embed_tokenizer("query: " + sentence, return_tensors="pt", truncation=True, padding=True)
    #     with torch.no_grad():
    #         qvec = self.embed_model(**input).last_hidden_state[:, 0].numpy()

    #     _, indices = self.index.search(qvec, 1)
    #     if len(indices[0]) == 0:
    #         self.get_logger().warn("Aucune réponse trouvée dans l'index.")
    #         return

    #     best_item = self.qa_pairs[indices[0][0]]
    #     base_answer = best_item['answer']

    #     # 📦 Exécuter l'intent si présent
    #     intent = best_item.get("intent")
    #     if intent:
    #         self.execute_intent(intent)
    #     self.get_logger().info(base_answer)

    #     # prompt = f"Réécris cette phrase en français avec une formulation différente, sans changer le sens : {base_answer}"
    #     # gen_input = self.gen_tokenizer(prompt, return_tensors="pt")
    #     # with torch.no_grad():
    #     #     output = self.gen_model.generate(**gen_input, max_new_tokens=60, do_sample=True, top_p=0.9, temperature=0.7)

    #     # reformulated = self.gen_tokenizer.decode(output[0], skip_special_tokens=True)
    #     # final_text = reformulated.replace(prompt, '').strip(': .\n')

    #     # self.get_logger().info(f"Q: {sentence}\nA: {final_text}")

    #     # 🎤 Parler
    #     if self.speaker_client.wait_for_service(timeout_sec=1.0):
    #         request = Text2Speach.Request()
    #         request.sentence = base_answer
    #         self.speaker_client.call_async(request)
    #     else:
    #         self.get_logger().warn("Service /qbo_driver/say_to_TTS non disponible.")

    def listen_callback(self, msg: ListenResult):
        sentence = msg.sentence.lower().strip()
        confidence = msg.confidence
        self.get_logger().info(f"🗣️ Reçu : {sentence} (confiance: {confidence:.2f})")

        if not self.index:
            self.get_logger().warn("Aucun index chargé, vectorisez d'abord.")
            return

        # 🔍 Extraire paramètres de la question (ex: couleur)
        color_keywords = {
            "rouge": 1, "bleu": 2, "violet": 3, "vert": 4,
            "jaune": 5, "magenta": 6, "blanc": 7
        }
        action_words = ["allume", "allumer", "mets", "mettre", "change", "changer"]

        self.last_detected_params = {}
        for name, code in color_keywords.items():
            if name in sentence:
                self.last_detected_params["color"] = code
                self.last_detected_params["color_name"] = name
                break
        self.get_logger().info(f"🔧 Paramètres détectés : {self.last_detected_params}")
        has_color = "color" in self.last_detected_params
        has_action_word = any(word in sentence for word in action_words)

        # 🔎 Recherche vectorielle
        inputs = self.embed_tokenizer("query: " + sentence, return_tensors="pt", truncation=True, padding=True)
        with torch.no_grad():
            qvec = self.embed_model(**inputs).last_hidden_state[:, 0].numpy()
        distances, indices = self.index.search(qvec, 1)


        # Récupérer la paire question/réponse/intention
        try:
            best_item = self.qa_pairs[indices[0][0]]
            confidence_score = 1.0 - distances[0][0] / 10.0  # Ajuste ce seuil selon ton dataset
            self.get_logger().info(f"❓ Question la plus proche : {best_item['question']}")
            self.get_logger().info(f"🔎 Score de confiance FAISS : {confidence_score:.2f}")
        except IndexError:
            self.get_logger().warn("Aucune correspondance trouvée dans la base.")
            return


        # 🔁 Exécuter l'intention si présente
        intent = best_item.get("intent", None)
        # 🔒 Correction sémantique simple
        # if intent and intent.get("action") == "get_led" and has_color and has_action_word:
        #     self.get_logger().info("🔁 Forçage en intent set_led grâce au contexte phrase + couleur")

        #     # on construit un intent set_led à la volée
        #     intent = {
        #         "type": "topic",
        #         "target": "/qbo_arduqbo/nose_ctrl/cmd_nose",
        #         "msg_type": "qbo_msgs/msg/Nose",
        #         "action": "set_led",
        #         "params": { "color": "{color}" }
        #     }

        #     # réponse générique si besoin
        #     best_item["answer"] = ["d'accord, je passe mon nez en {color}."]
        result = self.execute_intent(intent) if intent else None

        # 🎤 Générer la réponse finale
        answers = best_item['answer']
        if isinstance(answers, str):
            answers = [answers]
        answer_template = random.choice(answers)

        final_text = answer_template

        if "{color}" in final_text:
            color_name = self.last_detected_params.get("color_name")

            # cas lecture d'état (get_led)
            if result is not None:
                final_text = final_text.replace("{color}", result)

            # cas ordre utilisateur (set_led)
            elif color_name is not None:
                final_text = final_text.replace("{color}", color_name)

            else:
                final_text = final_text.replace("{color}", "inconnue")

        # 🧩 Interpolation dynamique (ex: {color})
        if "{color}" in final_text:
            color_val = self.last_detected_params.get("color", None)
            if isinstance(result, str):  # get_led a retourné un nom de couleur
                final_text = final_text.replace("{color}", result)
            elif color_val is not None:
                color_name = next((k for k, v in color_keywords.items() if v == color_val), "inconnu")
                final_text = final_text.replace("{color}", color_name)

        # 📢 Envoi au TTS
        self.get_logger().info(f"💬 Réponse : {final_text}")
        if self.speaker_client.wait_for_service(timeout_sec=1.0):
            request = Text2Speach.Request()
            request.sentence = final_text
            self.speaker_client.call_async(request)
        else:
            self.get_logger().warn("Service /qbo_driver/say_to_TTS non disponible.")

def main(args=None):
    rclpy.init(args=args)
    node = AIMLNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⏹️ Arrêt demandé...")
    finally:
        node.stop_evt.set()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
