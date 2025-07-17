#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import onnxruntime as ort
import numpy as np
import cv2

import faiss

from huggingface_hub import hf_hub_download

from qbo_msgs.srv import RecognizeFace
from cv_bridge import CvBridge
import rclpy
import os
import cv2
import time

class ArcFaceEmbedder:
    def __init__(self, model_path):
        self.session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
        self.input_name = self.session.get_inputs()[0].name

    def get_embedding(self, image: np.ndarray) -> np.ndarray:
        # Image: RGB 112x112 float32 normalized
        img = cv2.resize(image, (112, 112))
        img = img.astype(np.float32)
        img = (img - 127.5) / 128.0
        img = np.transpose(img, (2, 0, 1))  # HWC → CHW
        img = np.expand_dims(img, axis=0)   # Add batch dim
        return self.session.run(None, {self.input_name: img})[0][0]


class FaceIndex:
    def __init__(self, dim=512, threshold=0.6):
        print("🟡 FAISS index initialisé, aucun visage pour l’instant.")
        self.index = faiss.IndexFlatL2(dim)
        self.embeddings = []  # Pour garder les labels associés
        self.labels = []
        self.threshold = threshold

    def add(self, embedding: np.ndarray, label: str):
        self.initialized = True
        self.index.add(np.array([embedding], dtype='float32'))
        self.labels.append(label)

    def search(self, embedding: np.ndarray):
        if self.index.ntotal == 0 or len(self.labels) == 0:
            print("ℹ️ Aucun visage enregistré dans l’index FAISS.")
            return "unknown", False

        # ⚠️ On ne fait la recherche qu'après avoir vérifié qu'on a bien des données
        embedding_np = np.array([embedding], dtype='float32')
        D, I = self.index.search(embedding_np, 1)

        print(f"🔍 Recherche FAISS - index.ntotal = {self.index.ntotal}, labels = {len(self.labels)}")
        print(f"    Résultat FAISS: I={I}, D={D}")

        if I[0][0] >= len(self.labels):
            print(f"⚠️ Incohérence FAISS: index {I[0][0]} hors limites.")
            return "unknown", False

        if D[0][0] < self.threshold ** 2:
            return self.labels[I[0][0]], True

        return "unknown", False



class FaceRecognizerNode(Node):
    def __init__(self):
        super().__init__('qbo_face_recognizer')
            print("🟢 [INIT] Node created")

            self.initialized = False

            self.declare_parameter('model_path', '')
            self.declare_parameter('path_faces_learn', 'faces/learn')
            self.declare_parameter('threshold', 0.6)
            print("🟢 [INIT] Parameters declared")

            local_model_dir = os.path.join(os.path.expanduser("~"), ".qbo_models")
            os.makedirs(local_model_dir, exist_ok=True)
            print("🟢 [INIT] Model cache dir ensured")

            model_path_param = self.get_parameter('model_path').get_parameter_value().string_value
            if model_path_param:
                model_path = model_path_param
                print(f"✅ [INIT] Model path from param: {model_path}")
            else:
                print("📥 [INIT] Downloading model from HuggingFace...")
                model_path = hf_hub_download(
                    repo_id="deepinsight/insightface-model-zoo",
                    filename="recognition/arcface/glint360k_r50/model.onnx",
                    cache_dir=local_model_dir
                )
                print(f"✅ [INIT] Model downloaded: {model_path}")

            print("🧠 [INIT] Creating ArcFaceEmbedder...")
            self.arcface = ArcFaceEmbedder(model_path)
            print("✅ [INIT] ArcFaceEmbedder ready")

            self.path_faces_learn = self.get_parameter('path_faces_learn').value
            self.threshold = self.get_parameter('threshold').value
            self.bridge = CvBridge()
            self.faiss_index = FaceIndex(threshold=self.threshold)
            print("✅ [INIT] FAISS index initialized")

            self.last_save_time = 0
            self.srv = self.create_service(
                RecognizeFace,
                'qbo_face_recognizer/recognize_face',
                self.handle_recognize
            )
            print("✅ [INIT] ROS2 service registered")


    def handle_recognize(self, request, response):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(request.face, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f"Erreur conversion image : {e}")
            response.recognized = False
            response.name = "invalid_image"
            return response

        # Convert to RGB
        rgb = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
        emb = self.arcface.get_embedding(rgb)

        name, recognized = self.faiss_index.search(emb)
        response.recognized = recognized
        response.name = name

        if recognized:
            self.get_logger().info(f"🧠 Visage reconnu : {name}")
        else:
            now = time.time()
            if now - self.last_save_time < 2.0:
                self.get_logger().warn("⏱️ Trop tôt pour enregistrer un nouveau visage.")
                return response

            timestamp = str(int(now * 1e6))
            save_dir = os.path.join(self.path_faces_learn, timestamp)
            os.makedirs(save_dir, exist_ok=True)

            cv2.imwrite(os.path.join(save_dir, f"face_{timestamp}.jpg"), cv_image)
            flipped = cv2.flip(cv_image, 1)
            cv2.imwrite(os.path.join(save_dir, f"face_{timestamp}_flip.jpg"), flipped)

            self.last_save_time = now
            self.get_logger().warn(f"🤔 Visage inconnu enregistré dans {save_dir}")

        return response

    def main(args=None):
        rclpy.init(args=args)
        node = FaceRecognizerNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()


