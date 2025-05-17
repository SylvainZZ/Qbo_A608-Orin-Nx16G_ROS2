#!/usr/bin/env python3
"""
Version ROS2 (Humble) du nœud d'écoute Qbo.
— Convertit l'ancien script ROS1 (rospy) vers rclpy.
— Conçu pour tourner sur Jetson Orin NX16GB / Ubuntu 22.04 / JetPack 6.2.
"""

import os
import queue
from collections import deque

import numpy as np
import sounddevice as sd
import webrtcvad

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import String
from faster_whisper import WhisperModel


class ListenNode(Node):
    """Nœud ROS2 qui écoute le micro, détecte la parole et publie la transcription."""

    # ---------- Callbacks utilitaires et autres ----------

    def _sd_callback(self, indata, frames, time_info, status):
        """Appelé par sounddevice pour chaque bloc audio brut."""
        if status:
            self.get_logger().warning(f"Problème audio : {status}")
        # On empile les octets pour le traitement principal
        self.buffer_queue.put(bytes(indata))

    @staticmethod
    def _preprocess_audio(raw_audio):
        """Convertit les octets PAM en float32[-1,1]."""
        np_audio = np.frombuffer(raw_audio, dtype=np.int16).astype(np.float32) / 32768.0
        return np_audio  # le gain éventuel est appliqué ensuite si besoin

    def _find_device_index(self, wanted):
        for idx, dev in enumerate(sd.query_devices()):
            if wanted.lower() in dev['name'].lower():
                return idx
        self.get_logger().warning(f"⚠️  Device '{wanted}' not found, fallback default")
        return None

    # ---------- Construction du nœud ----------

    def __init__(self):
        super().__init__('qbo_listen')

        # ───── Paramètre de langue système ─────
        self.declare_parameter('system_lang', 'fr')
        self.lang = self.get_parameter('system_lang').get_parameter_value().string_value

        # ───── Publishers ─────
        self.system_lang_pub = self.create_publisher(String, '/system_lang', 1)
        self.listen_pub = self.create_publisher(String, '/listen', 10)
        self.system_lang_pub.publish(String(data=self.lang))

        # ───── Audio / VAD ─────
        self.CHANNELS = 1
        self.DURATION = 0.01  # 20 ms frame
        self.SAMPLE_RATE = 16_000
        self.VAD_SENSITIVITY = 1
        self.SILENCE_THRESHOLD = 10  # 0,1 s de silence = coupure
        # self.device_index = 0  # À adapter : "sd.query_devices()" pour la liste
        wanted_name = self.get_parameter('Jabra SPEAK 510 USB').value   # ex. "USB"
        self.device_index = self._find_device_index(wanted_name)

        self.vad = webrtcvad.Vad(self.VAD_SENSITIVITY)
        self.buffer_queue: "queue.Queue[bytes]" = queue.Queue()

        # ───── Chargement de Whisper ─────
        self.get_logger().info("Chargement du modèle Whisper (small / float16)…")
        # self.voice_model = WhisperModel(
        #     "small", device="cpu", compute_type="int8", cpu_threads=os.cpu_count()
        # )
        # --- inference (GPU, FP16) ---
        self.voice_model = WhisperModel(
            "small",
            device="cuda",
            device_index=0,
            compute_type="float16",
            # inter_threads   : nombre de lots concurrents (1 suffit)
            # inter_threads=1,
            # intra_threads   : threads CPU par lot (0 = auto). Sur Jetson, peu d’impact si on est en CUDA.
            # intra_threads=0,
        )
        # self.voice_model = WhisperModel(
        #     "small",
        #     device="cuda",
        #     compute_type="float16",
        #     cpu_threads=1,
        #     num_workers=1,
        #     # gpu_conv1d=False,          # ← désactive Conv1D GPU
        #     # vad_filter=True,
        #     # vad_parameters={"min_silence_duration_ms": 150},
        # )

        self.get_logger().info("Modèle Whisper chargé !")

        # Lancement direct de la boucle d'écoute (bloquante)
        self._listen_loop()

    # ---------- Boucle d'écoute principale ----------

    def _listen_loop(self):
        self.get_logger().info("En écoute … (Ctrl-C pour quitter)")
        while rclpy.ok():
            audio_data = deque()
            silent_frames = 0
            speaking = False

            try:
                with sd.RawInputStream(
                    samplerate=self.SAMPLE_RATE,
                    blocksize=int(self.SAMPLE_RATE * self.DURATION),
                    device=self.device_index,
                    dtype='int16',
                    channels=self.CHANNELS,
                    callback=self._sd_callback,
                ):
                    while rclpy.ok():
                        try:
                            frame = self.buffer_queue.get(timeout=1.0)
                        except queue.Empty:
                            continue

                        if len(frame) < int(self.SAMPLE_RATE * self.DURATION * 2):
                            continue  # frame incomplète

                        if self.vad.is_speech(frame, self.SAMPLE_RATE):
                            audio_data.append(frame)
                            silent_frames = 0
                            speaking = True
                        elif speaking:
                            silent_frames += 1
                            if silent_frames > self.SILENCE_THRESHOLD:
                                break  # fin de la phrase

                if not audio_data:
                    self.get_logger().debug("Aucun son détecté, on recommence…")
                    continue

                raw_audio = b"".join(audio_data)
                np_audio = self._preprocess_audio(raw_audio)

                try:
                    segments, _ = self.voice_model.transcribe(
                        np_audio,
                        language=self.lang,
                        beam_size=1,
                        vad_filter=True,
                        vad_parameters={"min_silence_duration_ms": 150},
                    )
                    text = " ".join(segment.text for segment in segments).strip()

                    if text:
                        self.get_logger().info(f"Transcription : {text}")
                        self.listen_pub.publish(String(data=text))
                    else:
                        self.get_logger().debug("Whisper n'a rien renvoyé, on recommence…")
                except Exception as exc:
                    self.get_logger().error(f"Erreur Whisper : {exc}")

            except KeyboardInterrupt:
                self.get_logger().info("Arrêt demandé → sortie propre.")
                break
            except Exception as exc:
                self.get_logger().error(f"Erreur dans la boucle d'écoute : {exc}")

        # Fin : on détruit proprement le nœud avant la fermeture
        self.destroy_node()


# ---------- Fonction main ----------

def main():
    rclpy.init()
    node = ListenNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
