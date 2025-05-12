#!/usr/bin/env python3
"""
ROS 2 Humble - Qbo Listen Node (v2)
----------------------------------
• Détection automatique du micro par *nom* (paramètre `audio_device_name`).
• S'adapte à la *fréquence native* de la carte son et ré-échantillonne en 16kHz (faster-whisper + VAD).
• Boucle d'écoute dans un *thread daemon* avec arrêt propre via `threading.Event`.
• Compatibilité Jetson Orin NX 16 GB -- CTranslate2 GPU + cuDNN.
"""

from __future__ import annotations

import os, queue, threading, time
import math
from collections import deque
from typing import Optional

import numpy as np
from scipy.signal import resample_poly  # pip install scipy (lightweight)
import sounddevice as sd
import webrtcvad

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from qbo_msgs.msg import ListenResult
from faster_whisper import WhisperModel


def find_device(name_hint: str) -> tuple[Optional[int], int]:
    """Retourne (index, sample_rate_natif) du premier device d'entrée
    dont *name_hint* (insensible à la casse) est contenu dans le nom."""
    name_hint = name_hint.lower()
    for idx, dev in enumerate(sd.query_devices()):
        if dev["max_input_channels"] < 1:
            continue
        if name_hint in dev["name"].lower():
            return idx, int(dev["default_samplerate"])
    # aucun device correspondant ➜ prendre le défaut système
    dev = sd.query_devices(kind="input")
    return None, int(dev["default_samplerate"])


class ListenNode(Node):
    CHANNELS = 1
    TARGET_RATE = 16_000            # ce que veulent VAD + Whisper (Hz)
    FRAME_MS = 10                    # frame audio = 10 ms ➜ 160 samples @16 kHz
    VAD_SENSITIVITY = 1             # 0 = moins agressif, 3 = plus
    SILENCE_THRESHOLD = 10          # 10 × 10 ms = 100 ms de silence → fin phrase

    def __init__(self):
        super().__init__("qbo_listen")

        # ── Paramètres ROS 2 ───────────────────────────────────────────
        self.declare_parameter("audio_device_name", "usb")
        self.declare_parameter("system_lang", "fr")
        self.declare_parameter("whisper_model", "small")
        self.lang = self.get_parameter("system_lang").get_parameter_value().string_value
        device_hint = self.get_parameter("audio_device_name").get_parameter_value().string_value
        model_size  = self.get_parameter("whisper_model").get_parameter_value().string_value

        # ── Sélection du périphérique audio ───────────────────────────
        self.device_index, self.input_rate = find_device(device_hint)
        self.get_logger().info(f"🎤 Device index={self.device_index}, rate={self.input_rate} Hz")

        self.blocksize = int(self.input_rate * self.FRAME_MS / 1000)
        self.vad = webrtcvad.Vad(self.VAD_SENSITIVITY)
        self.buffer_queue: queue.Queue[bytes] = queue.Queue()

        # ── Publishers ────────────────────────────────────────────────
        self.system_lang_pub = self.create_publisher(String, "/system_lang", 1)
        self.result_pub = self.create_publisher(ListenResult, "/listen", 10)
        self.system_lang_pub.publish(String(data=self.lang))

        # ── Whisper initialisation ───────────────────────────────────
        self.get_logger().info("🔄 Chargement du modèle Whisper…")
        self.voice_model = WhisperModel(
            model_size,
            device="cuda",            # GPU Orin
            compute_type="float16",  # cuDNN activé

        )
        self.get_logger().info("✅ Modèle Whisper prêt.")

        # ── Thread d'écoute ──────────────────────────────────────────
        self.stop_evt = threading.Event()
        self.audio_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.audio_thread.start()

    # ─────────────────────────────────────────────────────────────────
    # Audio utils
    def _callback(self, indata, frames, t, status):
        if status:
            self.get_logger().warning(f"Audio status : {status}")
        self.buffer_queue.put(bytes(indata))

    def _resample_to_target(self, pcm: np.ndarray) -> np.ndarray:
        if self.input_rate == self.TARGET_RATE:
            return pcm
        return resample_poly(pcm, self.TARGET_RATE, self.input_rate)

    def _preprocess_block(self, raw: bytes) -> np.ndarray:
        pcm = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
        pcm = self._resample_to_target(pcm)
        return pcm

    def _confidence(self, segments) -> float:
        if not segments:
            self.get_logger().info("Attention segments vide.")
            return 0.0
        for i, seg in enumerate(segments):
            self.get_logger().info(
                f"[seg {i}] avg_logprob={seg.avg_logprob:.3f} "
                f"no_speech_prob={seg.no_speech_prob:.3f}")
        if not segments:
            return 0.0

        use_avg = all(math.isfinite(s.avg_logprob) and s.avg_logprob > -10
                      for s in segments)

        total, tokens = 0.0, 0
        for seg in segments:
            ntok = max(1, len(seg.text.strip().split()))
            score = (math.exp(seg.avg_logprob)
                     if use_avg else (1.0 - seg.no_speech_prob))
            total  += score * ntok
            tokens += ntok

        return total / tokens if tokens else 0.0


    # ─────────────────────────────────────────────────────────────────
    def _listen_loop(self):
        self.get_logger().info("🎙️ En écoute … (Ctrl-C pour quitter)")
        while not self.stop_evt.is_set() and rclpy.ok():
            audio_frames = deque()
            silent_frames = 0
            speaking = False
            try:
                with sd.RawInputStream(
                    samplerate=self.input_rate,
                    blocksize=self.blocksize,
                    device=self.device_index,
                    dtype="int16",
                    channels=self.CHANNELS,
                    callback=self._callback,
                ):
                    while not self.stop_evt.is_set():
                        try:
                            frame = self.buffer_queue.get(timeout=0.5)
                        except queue.Empty:
                            continue
                        if len(frame) < self.blocksize * 2:
                            continue
                        try:
                            is_speech = self.vad.is_speech(frame, self.TARGET_RATE)
                        except Exception as e:
                            self.get_logger().warning(f"VAD error: {e}")
                            continue
                        if is_speech:
                            audio_frames.append(frame)
                            silent_frames = 0
                            speaking = True
                        elif speaking:
                            silent_frames += 1
                            if silent_frames > self.SILENCE_THRESHOLD:
                                break  # fin de phrase
            except Exception as e:
                self.get_logger().error(f"Stream error: {e}")
                time.sleep(1)
                continue

            if not audio_frames:
                continue

            raw_audio = b"".join(audio_frames)
            pcm = self._preprocess_block(raw_audio)

            try:
                segments, _ = self.voice_model.transcribe(
                    pcm,
                    language=self.lang,
                    beam_size=1,
                    vad_filter=True,
                    vad_parameters={"min_silence_duration_ms": 150},
                )
                text = " ".join(s.text for s in segments).strip()

                if text:
                    conf = self._confidence(segments)
                    msg  = ListenResult(text=text, confidence=conf)
                    self.result_pub.publish(msg)
                    self.get_logger().info(f"📝 {text}  (conf={conf:.3f})")
            except Exception as e:
                self.get_logger().error(f"Whisper error: {e}")

    # ─────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ListenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⏹️ Arrêt demandé → sortie propre.")
    finally:
        node.stop_evt.set()
        node.audio_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
