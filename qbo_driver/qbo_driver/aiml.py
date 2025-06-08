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
        self.speaker_client = self.create_client(Text2Speach, '/qbo_driver/piper2wave_say')

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

    def load_latest_index(self):
        try:
            files = [f for f in os.listdir(LLM_DIR) if f.endswith('.faiss')]
            if not files:
                self.get_logger().warn("Aucun index FAISS trouvé dans config/LLM/")
                return
            files.sort(reverse=True)
            latest_faiss = files[0]
            json_name = latest_faiss.replace('_index.faiss', '_pairs.json')

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
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"Erreur vectorisation : {e}")

        return response

    def listen_callback(self, msg: ListenResult):

        sentence = msg.sentence.lower().strip()
        confidence = msg.confidence
        self.get_logger().info(sentence)

        if not self.index:
            self.get_logger().warn("Aucun index chargé, vectorisez d'abord.")
            return

        input = self.embed_tokenizer("query: " + sentence, return_tensors="pt", truncation=True, padding=True)
        with torch.no_grad():
            qvec = self.embed_model(**input).last_hidden_state[:, 0].numpy()

        _, indices = self.index.search(qvec, 1)
        best_item = self.qa_pairs[indices[0][0]]
        base_answer = best_item['answer']
        self.get_logger().info(base_answer)

        # prompt = f"Réécris cette phrase en français avec une formulation différente, sans changer le sens : {base_answer}"
        # gen_input = self.gen_tokenizer(prompt, return_tensors="pt")
        # with torch.no_grad():
        #     output = self.gen_model.generate(**gen_input, max_new_tokens=60, do_sample=True, top_p=0.9, temperature=0.7)

        # reformulated = self.gen_tokenizer.decode(output[0], skip_special_tokens=True)
        # final_text = reformulated.replace(prompt, '').strip(': .\n')

        # self.get_logger().info(f"Q: {sentence}\nA: {final_text}")

        if self.speaker_client.wait_for_service(timeout_sec=1.0):
            request = Text2Speach.Request()
            request.sentence = base_answer
            self.speaker_client.call_async(request)
        else:
            self.get_logger().warn("Service /qbo_driver/piper2wave_say non disponible.")


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
