import os
import json
import faiss
import torch
from datetime import datetime
import numpy as np

from transformers import AutoTokenizer, AutoModel

from qbo_driver.qbo_aiml.constants import FAISS_CONFIDENCE_THRESHOLD

class QALoader:

    def __init__(self, model_name, logger, data_dir, index_dir):
        self.logger = logger
        self.data_dir = data_dir
        self.index_dir = index_dir

        # Embedding (optionnel)
        self.tokenizer = None
        self.model = None

        if model_name:
            self.logger.info(f"🔄 Chargement embedding model: {model_name}")
            self.tokenizer = AutoTokenizer.from_pretrained(model_name)
            self.model = AutoModel.from_pretrained(model_name).eval()
            self.logger.info("✅ Embedding model prêt.")
        else:
            self.logger.info("🔁 Embedding non chargé (partage attendu).")

        # FAISS / data
        self.index = None
        self.entries = []
        self.variant_to_entry = []
        self.qa_pairs = []

    # ========================================
    # LOAD EXISTING INDEX
    # ========================================

    def share_embedding(self, other_loader):
        self.tokenizer = other_loader.tokenizer
        self.model = other_loader.model

    def load_latest_index(self, prefix="index"):

        try:
            files = [f for f in os.listdir(self.index_dir)
                    if f.startswith(prefix + "_") and f.endswith(".faiss")]
            if not files:
                self.logger.warn("⚠ Aucun index FAISS trouvé.")
                return

            files.sort(reverse=True)
            latest_faiss = files[0]
            json_name = latest_faiss.replace('.faiss', '.json')

            faiss_path = os.path.join(self.index_dir, latest_faiss)
            json_path = os.path.join(self.index_dir, json_name)

            if not os.path.exists(json_path):
                self.logger.error("❌ JSON associé manquant.")
                return

            self.index = faiss.read_index(faiss_path)

            with open(json_path, 'r', encoding='utf-8') as f:
                self.qa_pairs = json.load(f)

            # 🔹 Vérification structure minimale
            if not isinstance(self.qa_pairs, list):
                self.logger.error("❌ JSON index invalide (pas une liste).")
                self.index = None
                self.qa_pairs = []
                return

            if self.index.ntotal != len(self.qa_pairs):
                self.logger.error("❌ Mismatch index / JSON.")
                self.index = None
                self.qa_pairs = []
                return

            self.logger.info(f"📦 {len(self.qa_pairs)} variantes chargées")
            # self.logger.info(f"🔎 Index type: {type(self.index)} | metric: {self.index.metric_type}")
            self.logger.info(f"✅ Index chargé : {latest_faiss}")

        except Exception as e:
            self.logger.error(f"❌ Erreur chargement index : {e}")

    # ========================================
    # SEARCH (unique, propre)
    # ========================================
    def search(self, sentence):

        if self.model is None or self.tokenizer is None:
            self.logger.error("❌ Embedding non initialisé. Appelle share_embedding() ou charge un model_name.")
            return None, 0.0

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

        # normalisation requête
        qvec = qvec / np.linalg.norm(qvec, axis=1, keepdims=True)
        # print("Norm min/max:",
        #     np.min(np.linalg.norm(qvec, axis=1)),
        #     np.max(np.linalg.norm(qvec, axis=1)))

        scores, indices = self.index.search(qvec, 1)

        idx = indices[0][0]

        if idx >= len(self.qa_pairs):
            return None, 0.0

        best_item = self.qa_pairs[idx]["entry"]

        raw_score = scores[0][0]   # 👈 ICI
        confidence = float(raw_score)

        self.logger.info(
            f"🔎 RAG match: {best_item.get('question', 'N/A')} "
            f"(score {confidence:.2f})"
        )

        return best_item, confidence

    def search_topk(self, sentence, k=5):

        if self.model is None or self.tokenizer is None:
            self.logger.error("❌ Embedding non initialisé. Appelle share_embedding() ou charge un model_name.")
            return []

        if not self.index:
            self.logger.warn("⚠ Aucun index chargé.")
            return []

        inputs = self.tokenizer(
            "query: " + sentence,
            return_tensors="pt",
            truncation=True,
            padding=True
        )

        with torch.no_grad():
            qvec = self.model(**inputs).last_hidden_state[:, 0].numpy()

        # normalisation requête
        qvec = qvec / np.linalg.norm(qvec, axis=1, keepdims=True)
        # print("Norm min/max:",
        #     np.min(np.linalg.norm(qvec, axis=1)),
        #     np.max(np.linalg.norm(qvec, axis=1)))

        scores, indices = self.index.search(qvec, k)

        results = []

        for idx, score in zip(indices[0], scores[0]):
            if idx < 0:
                continue
            if idx >= len(self.qa_pairs):
                continue

            results.append({
                "item": self.qa_pairs[idx]["entry"],
                "score": float(score)
            })

        return results

    # ========================================
    # REBUILD INDEX
    # ========================================
    def rebuild_index(self, prefix="index", filter_fn=None):

        if not self.data_dir:
            return False, "DATA_DIR non défini."

        try:
            all_entries = []

            # 🔹 Chargement fichiers JSON
            for fname in os.listdir(self.data_dir):
                if fname.endswith(".json"):

                    path = os.path.join(self.data_dir, fname)

                    with open(path, 'r', encoding='utf-8') as f:
                        data = json.load(f)

                    if isinstance(data, list):
                        all_entries.extend(data)
                        if filter_fn is not None:
                            all_entries = [e for e in all_entries if filter_fn(e)]
                    else:
                        self.logger.warn(
                            f"⚠ Fichier ignoré (format invalide) : {fname}"
                        )

            if not all_entries:
                return False, "Aucune donnée trouvée."

            # =====================================================
            # 🔹 Dépliage des question_variants
            # =====================================================

            self.qa_pairs = []          # utilisé pour l'index
            self.qa_entries = all_entries  # structure complète

            for entry in all_entries:

                # Nouveau format
                if "question_variants" in entry:
                    variants = entry.get("question_variants", [])

                # Ancien format (compatibilité)
                elif "question" in entry:
                    variants = [entry["question"]]

                else:
                    continue

                for variant in variants:
                    self.qa_pairs.append({
                        "question": variant,
                        "entry": entry
                    })

            if not self.qa_pairs:
                return False, "Aucune question exploitable."

            # =====================================================
            # 🔹 Embedding uniquement sur les questions
            # =====================================================

            sentences = [
                "passage: " + item["question"]
                for item in self.qa_pairs
            ]

            inputs = self.tokenizer(
                sentences,
                padding=True,
                truncation=True,
                return_tensors="pt"
            )

            with torch.no_grad():
                vectors = self.model(**inputs).last_hidden_state[:, 0].numpy()

            # 🔹 Normalisation cosine
            vectors = vectors / np.linalg.norm(vectors, axis=1, keepdims=True)

            # 🔹 Index cosine (Inner Product)
            index = faiss.IndexFlatIP(vectors.shape[1])
            index.add(vectors)

            # =====================================================
            # 🔹 Sauvegarde index + JSON source
            # =====================================================

            now = datetime.now().strftime("%Y_%m_%d_%H_%M")
            index_file = f"{prefix}_{now}.faiss"
            variants_file = f"{prefix}_{now}.json"
            entries_file = f"{prefix}_entries_{now}.json"

            faiss.write_index(index, os.path.join(self.index_dir, index_file))

            with open(os.path.join(self.index_dir, variants_file), "w", encoding="utf-8") as f:
                json.dump(self.qa_pairs, f, ensure_ascii=False, indent=2)

            with open(os.path.join(self.index_dir, entries_file), "w", encoding="utf-8") as f:
                json.dump(all_entries, f, ensure_ascii=False, indent=2)

            self.logger.info(f"✅ Index sauvegardé : {index_file}")
            self.logger.info(f"📦 {len(self.qa_pairs)} variantes indexées")

            self.load_latest_index(prefix=prefix)

            return True, f"Index créé avec {len(self.qa_pairs)} variantes."

        except Exception as e:
            self.logger.error(f"❌ Erreur rebuild : {e}")
            return False, str(e)
