import os
import json
import faiss
import torch
from datetime import datetime
from transformers import AutoTokenizer, AutoModel

from qbo_driver.qbo_aiml.constants import FAISS_CONFIDENCE_THRESHOLD



EMBED_MODEL_NAME = "intfloat/e5-small-v2"

'''
QALoader : classe de gestion du chargement et de la recherche dans l'index de questions-réponses pour l'AIMLNode.
- load_latest_index() : charge le dernier index FAISS et son JSON associé.
- rebuild_index() : reconstruit l'index à partir des fichiers JSON de data_pairs, sauvegarde le nouvel index et le JSON.
- search(sentence) : recherche la question la plus proche de la phrase donnée, retourne la paire question-réponse et une confiance basée sur la distance.

🔥 Ce que cette version apporte

    ✔ Modèle chargé une seule fois
    ✔ Gestion erreurs robuste
    ✔ Vérification cohérence index/json
    ✔ Méthode search() propre
    ✔ Méthode rebuild_index() propre
    ✔ Indépendant de ROS

🧠 Évolutions futures possibles
    - passer en cosine similarity (IndexFlatIP)
    - normaliser les embeddings
    - ajouter seuil dynamique
    - ajouter top_k > 1 pour reranking
    - ajouter fallback contextuel

'''


class QALoader:

    def __init__(self, llm_dir, data_dir, logger):
        self.llm_dir = llm_dir
        self.data_dir = data_dir
        self.logger = logger

        self.index = None
        self.qa_pairs = []

        # 🔹 Chargement modèle embedding
        self.logger.info("🔄 Chargement modèle embedding...")
        self.tokenizer = AutoTokenizer.from_pretrained(EMBED_MODEL_NAME)
        self.model = AutoModel.from_pretrained(EMBED_MODEL_NAME).eval()

        # 🔹 Chargement index existant
        self.load_latest_index()

    # ========================================
    # CHARGEMENT INDEX EXISTANT
    # ========================================
    def load_latest_index(self):

        try:
            files = [f for f in os.listdir(self.llm_dir) if f.endswith('.faiss')]
            if not files:
                self.logger.warn("⚠ Aucun index FAISS trouvé.")
                return

            files.sort(reverse=True)
            latest_faiss = files[0]
            json_name = latest_faiss.replace('.faiss', '.json')

            faiss_path = os.path.join(self.llm_dir, latest_faiss)
            json_path = os.path.join(self.llm_dir, json_name)

            if not os.path.exists(json_path):
                self.logger.error("❌ JSON associé manquant.")
                return

            self.index = faiss.read_index(faiss_path)

            with open(json_path, 'r', encoding='utf-8') as f:
                self.qa_pairs = json.load(f)

            if self.index.ntotal != len(self.qa_pairs):
                self.logger.error("❌ Mismatch index / JSON.")
                self.index = None
                self.qa_pairs = []
                return

            self.logger.info(f"✅ Index chargé : {latest_faiss}")

        except Exception as e:
            self.logger.error(f"❌ Erreur chargement index : {e}")

    # ========================================
    # REBUILD INDEX
    # ========================================
    def rebuild_index(self):

        try:
            all_data = []

            for fname in os.listdir(self.data_dir):
                if fname.endswith(".json"):
                    with open(os.path.join(self.data_dir, fname), 'r', encoding='utf-8') as f:
                        all_data.extend(json.load(f))

            if not all_data:
                return False, "Aucune donnée trouvée."

            texts = [
                f"{item['question']} {item.get('answer', '')}"
                for item in all_data
            ]

            inputs = self.tokenizer(
                ["passage: " + t for t in texts],
                padding=True,
                truncation=True,
                return_tensors="pt"
            )

            with torch.no_grad():
                vectors = self.model(**inputs).last_hidden_state[:, 0].numpy()

            index = faiss.IndexFlatL2(vectors.shape[1])
            index.add(vectors)

            now = datetime.now().strftime("%Y_%m_%d_%H_%M")
            index_file = f"index_{now}.faiss"
            json_file = f"index_{now}.json"

            faiss.write_index(index, os.path.join(self.llm_dir, index_file))

            with open(os.path.join(self.llm_dir, json_file), 'w', encoding='utf-8') as f:
                json.dump(all_data, f, ensure_ascii=False, indent=2)

            self.logger.info(f"✅ Index sauvegardé : {index_file}")

            # recharger
            self.load_latest_index()

            return True, f"Index créé avec {len(all_data)} entrées."

        except Exception as e:
            self.logger.error(f"❌ Erreur rebuild : {e}")
            return False, str(e)

    # ========================================
    # SEARCH
    # ========================================
    def search(self, sentence):

        if not self.index:
            self.logger.warn("⚠ Aucun index chargé.")
            return None, 0.0

        inputs = self.tokenizer(
            "query: " + sentence,
            return_tensors="pt",
            truncation=True,
            padding=True
        )

        with torch.no_grad():
            qvec = self.model(**inputs).last_hidden_state[:, 0].numpy()

        distances, indices = self.index.search(qvec, 1)

        idx = indices[0][0]

        if idx >= len(self.qa_pairs):
            return None, 0.0

        best_item = self.qa_pairs[idx]

        # Normalisation distance → pseudo confidence
        raw_distance = distances[0][0]
        confidence = max(0.0, 1.0 - raw_distance / 10.0)

        return best_item, confidence
