#!/usr/bin/env python3
"""
ROS 2 Node: Coqui TTS (YourTTS) GPU, service compatible Text2Speach.srv
Backend audio: SoX play uniquement (pas de sounddevice / pas de stream).

- 16 kHz fixe (Jabra)
- sortie stéréo (duplication mono)
- /tts_active publié pendant lecture
- pas de mute micro / AEC/NR (géré par Jabra)
"""

import os
import time
import queue
import threading
import subprocess
import unicodedata
import re

import numpy as np
from scipy.signal import resample

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from qbo_msgs.srv import Text2Speach

from TTS.api import TTS

FIXED_SR = 16000  # Jabra: 16k uniquement


ACRONYM_LEXICON = {
    "QBO": "ku bé o",
    "KNX": "ka en x",
    "ROS2": "R O S deux",
    "ROS": "R O S",
    "GPU": "gé pé u",
    "CPU": "cé pé u",
    "USB": "u ès bé",
    "I2C": "i deux c",
    "UART": "u a r t",
    "IMU": "i m u",
    "SLAM": "s l a m",
    "LIDAR": "li dar",
}

WORD_LEXICON = {
    "JetPack": "Jet pack",
    "NVIDIA": "En vidia",
}


def normalize_text_for_tts(text: str) -> str:
    t = " ".join(text.strip().split())
    for k, v in ACRONYM_LEXICON.items():
        t = re.sub(rf"\b{k}\b", v, t)
    for k, v in WORD_LEXICON.items():
        t = re.sub(rf"\b{k}\b", v, t)
    return t


def apply_fade(wav: np.ndarray, sr: int, fade_ms: float) -> np.ndarray:
    wav = np.asarray(wav, dtype=np.float32).reshape(-1)
    n = int(sr * fade_ms / 1000.0)
    if n <= 1 or wav.size < 2 * n:
        return wav
    fade_in = np.linspace(0.0, 1.0, n, dtype=np.float32)
    fade_out = np.linspace(1.0, 0.0, n, dtype=np.float32)
    wav[:n] *= fade_in
    wav[-n:] *= fade_out
    return wav


def get_tts_output_sr(tts_obj, default=22050) -> int:
    for attr_path in [
        ("synthesizer", "output_sample_rate"),
        ("synthesizer", "sample_rate"),
        ("synthesizer", "config", "audio", "sample_rate"),
        ("synthesizer", "tts_config", "audio", "sample_rate"),
        ("config", "audio", "sample_rate"),
    ]:
        try:
            obj = tts_obj
            for a in attr_path:
                obj = getattr(obj, a)
            sr = int(obj)
            if 8000 <= sr <= 48000:
                return sr
        except Exception:
            pass
    return int(default)


def sox_tempo_raw(wav: np.ndarray, sr_in: int, rate: float, sr_out: int | None = None) -> tuple[np.ndarray, int]:
    """
    Time-stretch via sox tempo -s, puis resample explicite si demandé.
    I/O en raw float32 mono.
    """
    wav = np.asarray(wav, dtype=np.float32).reshape(-1)
    out_sr = sr_in if sr_out is None else int(sr_out)

    if abs(rate - 1.0) < 1e-3 and out_sr == sr_in:
        return wav, sr_in

    cmd = [
        "sox", "-q",
        "-t", "raw",
        "-r", str(sr_in),
        "-e", "floating-point",
        "-b", "32",
        "-c", "1",
        "-",
        "-t", "raw",
        "-r", str(out_sr),
        "-e", "floating-point",
        "-b", "32",
        "-c", "1",
        "-",
        "tempo", "-s", str(rate),
    ]
    if out_sr != sr_in:
        cmd += ["rate", "-v", str(out_sr)]

    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    out, err = p.communicate(input=wav.tobytes())
    if p.returncode != 0:
        raise RuntimeError(f"sox tempo failed: {err.decode('utf-8', errors='ignore')}")
    return np.frombuffer(out, dtype=np.float32), out_sr


def play_with_sox_raw(stereo_i16: np.ndarray, sr: int):
    """
    Playback via SoX 'play' en raw PCM int16 stéréo via stdin.
    Utilise le périphérique par défaut (donc ton pactl set-default-sink).
    """
    cmd = [
        "play", "-q",
        "-t", "raw",
        "-r", str(sr),
        "-e", "signed-integer",
        "-b", "16",
        "-c", "2",
        "-",  # stdin
    ]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        p.stdin.write(stereo_i16.tobytes())
        p.stdin.close()
        p.wait()
    finally:
        try:
            p.kill()
        except Exception:
            pass


class QboTalkCoquiNode(Node):
    def __init__(self):
        super().__init__("qbo_talk_coqui")

        # ---- Audio (simplifié) ----
        self.declare_parameter("sample_rate", FIXED_SR)  # immuable
        self.declare_parameter("output_channels", 2)
        self.declare_parameter("audio_playback_volume", 80)  # 0..150

        sr = int(self.get_parameter("sample_rate").value)
        if sr != FIXED_SR:
            self.get_logger().warning(f"sample_rate demandé={sr} mais Jabra impose {FIXED_SR}. Forçage.")
        self.stream_sr = FIXED_SR

        ch = int(self.get_parameter("output_channels").value)
        if ch != 2:
            self.get_logger().warning("output_channels attendu=2. Forçage à 2.")
        self.stream_channels = 2

        self.volume = int(self.get_parameter("audio_playback_volume").value)

        # Anti-pop (à ajuster si besoin)
        self.fade_ms = 20.0
        self.pad_ms = 10.0

        # Normalisation/limiter doux
        self.declare_parameter("target_rms", 0.08)
        self.declare_parameter("soft_limit", 2.0)
        self.target_rms = float(self.get_parameter("target_rms").value)
        self.soft_limit = float(self.get_parameter("soft_limit").value)

        # ---- Langue / compat ----
        self.declare_parameter("default_lang", "fr")
        self.lang = self.get_parameter("default_lang").value

        # ---- Coqui ----
        self.declare_parameter("tts_model_name", "tts_models/multilingual/multi-dataset/your_tts")
        self.declare_parameter("tts_speaker", "male-en-2")
        self.declare_parameter("tts_language_fr", "fr-fr")
        self.declare_parameter("tts_language_en", "en")
        self.declare_parameter("use_gpu", True)
        self.declare_parameter("warmup_text", "Bonjour.")
        self.declare_parameter("queue_maxsize", 10)
        self.declare_parameter("tts_speaker_wav", "/home/qbo-v2/venvs/tts/qbo_fr_ref_22050.wav")
        self.declare_parameter("speech_rate", 0.80)

        self.model_name = self.get_parameter("tts_model_name").value
        self.speaker = self.get_parameter("tts_speaker").value
        self.lang_fr = self.get_parameter("tts_language_fr").value
        self.lang_en = self.get_parameter("tts_language_en").value
        self.use_gpu = bool(self.get_parameter("use_gpu").value)
        self.warmup_text = self.get_parameter("warmup_text").value
        self.queue_maxsize = int(self.get_parameter("queue_maxsize").value)
        self.speech_rate = float(self.get_parameter("speech_rate").value)

        self.speaker_wav = self.get_parameter("tts_speaker_wav").value.strip()
        if not self.speaker_wav:
            self.speaker_wav = None

        self.get_logger().info(f"speaker_wav exists = {os.path.exists(self.speaker_wav) if self.speaker_wav else False}")
        self.get_logger().info("🔊 Backend audio: SoX play (raw stdin), pas de stream ouvert")

        # ---- ROS /tts_active ----
        self.tts_active_pub = self.create_publisher(Bool, "/tts_active", 10)
        self._publish_tts_active(False)

        # ---- Load Coqui ----
        self.get_logger().info(f"Loading Coqui model: {self.model_name} (gpu={self.use_gpu}) ...")
        t0 = time.time()
        self.tts = TTS(model_name=self.model_name, progress_bar=False)
        self.tts.to("cuda" if self.use_gpu else "cpu")
        self.get_logger().info(f"Model loaded in {time.time() - t0:.2f}s")
        self.get_logger().info(f"Available languages: {getattr(self.tts, 'languages', None)}")
        self.get_logger().info(f"Available speakers: {getattr(self.tts, 'speakers', None)}")

        # Warmup
        try:
            self.get_logger().info("Warmup...")
            _ = self.tts.tts(self.warmup_text, language=self._coqui_language(), speaker=self.speaker)
            self.get_logger().info("Warmup done ✅")
        except Exception as e:
            self.get_logger().warning(f"Warmup failed: {e}")

        # Cache embedding (si speaker_wav)
        self._precompute_speaker_embedding()

        # ---- Queue + worker ----
        self.q = queue.Queue(maxsize=max(1, self.queue_maxsize))
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        # ---- ROS interfaces ----
        self.create_service(Text2Speach, "/qbo_driver/say_to_TTS", self.say_callback)
        self.create_subscription(String, "/system_lang", self.lang_callback, 10)
        self.create_subscription(String, "/system_out_volume", self.out_volume_callback, 10)

        self.get_logger().info(f"Langue initiale : {self.lang}")

    def _publish_tts_active(self, active: bool):
        msg = Bool()
        msg.data = bool(active)
        self.tts_active_pub.publish(msg)

    def _coqui_language(self):
        if (self.lang or "").lower().startswith("en"):
            return self.lang_en
        return self.lang_fr

    def _volume_gain(self) -> float:
        v = max(0, min(150, int(self.volume)))
        return float(v) / 100.0

    def _precompute_speaker_embedding(self):
        if not self.speaker_wav:
            self.get_logger().info("No speaker_wav -> pas de cache speaker embedding.")
            return

        try:
            sm = self.tts.synthesizer.tts_model.speaker_manager
        except Exception:
            self.get_logger().warning("speaker_manager introuvable -> cache embedding non supporté sur ce modèle.")
            return

        compute_fn_name = None
        for name in ("compute_embedding_from_clip", "compute_embedding", "get_embedding"):
            if hasattr(sm, name):
                compute_fn_name = name
                break
        if compute_fn_name is None:
            self.get_logger().warning("Aucune fonction compute embedding trouvée -> cache désactivé.")
            return

        compute_fn = getattr(sm, compute_fn_name)
        cache_key = os.path.abspath(self.speaker_wav)

        self.get_logger().info("Precomputing speaker embedding (cache)...")
        t0 = time.time()
        emb = compute_fn(cache_key)
        self.get_logger().info(f"Speaker embedding cached ✅ ({time.time()-t0:.2f}s)")

        orig_fn = compute_fn

        def cached_compute(path, *args, **kwargs):
            try:
                p = os.path.abspath(path)
                if p == cache_key:
                    return emb
            except Exception:
                pass
            return orig_fn(path, *args, **kwargs)

        setattr(sm, compute_fn_name, cached_compute)
        self.get_logger().info("Speaker embedding hook installed ✅ (no recompute for cached wav)")

    def _play_audio(self, wav: np.ndarray, sr: int):
        wav = np.asarray(wav, dtype=np.float32).reshape(-1)

        # resample vers 16k si besoin
        if sr != self.stream_sr:
            n = int(len(wav) * self.stream_sr / sr)
            wav = resample(wav, n).astype(np.float32)

        # gain volume
        wav = wav * self._volume_gain()

        # normalisation RMS
        rms = float(np.sqrt(np.mean(wav * wav))) if wav.size else 0.0
        if rms > 1e-6:
            wav = wav * (self.target_rms / rms)

        # limiteur doux
        k = self.soft_limit
        wav = np.tanh(wav * k) / np.tanh(k)

        # pad + fade anti-pop
        pad_n = int(self.stream_sr * self.pad_ms / 1000.0)
        if pad_n > 0:
            wav = np.concatenate([np.zeros(pad_n, np.float32), wav, np.zeros(pad_n, np.float32)])
        wav = apply_fade(wav, self.stream_sr, self.fade_ms)

        # mono -> stéréo
        stereo_f32 = np.column_stack([wav, wav]).astype(np.float32, copy=False)

        # float32 -> int16 PCM
        stereo_i16 = np.clip(stereo_f32 * 32767.0, -32768, 32767).astype(np.int16, copy=False)

        # playback via SoX
        play_with_sox_raw(stereo_i16, self.stream_sr)

    def say_callback(self, request, response):
        text = normalize_text_for_tts(unicodedata.normalize("NFC", (request.sentence or "").strip()))
        if not text:
            response.success = False
            return response

        try:
            self.q.put_nowait(text)
            response.success = True
        except queue.Full:
            self.get_logger().warning("Queue TTS pleine, requête ignorée.")
            response.success = False
        return response

    def lang_callback(self, msg: String):
        self.lang = (msg.data or "").strip().lower()
        self.get_logger().info(f"Langue changée dynamiquement : {self.lang} -> {self._coqui_language()}")

    def out_volume_callback(self, msg: String):
        try:
            vol = int((msg.data or "").strip().replace("%", ""))
            if 0 <= vol <= 150:
                self.volume = vol
                self.get_logger().info(f"🔊 Volume (software) ajusté à {self.volume}% (gain={self._volume_gain():.2f})")
            else:
                self.get_logger().warning(f"Valeur de volume hors plage : {msg.data}")
        except ValueError:
            self.get_logger().warning(f"Volume invalide reçu : {msg.data}")

    def _worker_loop(self):
        while rclpy.ok():
            try:
                text = self.q.get(timeout=0.2)
            except queue.Empty:
                continue

            self._publish_tts_active(True)

            try:
                t0 = time.time()

                kwargs = {"language": self._coqui_language()}
                if self.speaker_wav:
                    kwargs["speaker_wav"] = self.speaker_wav
                else:
                    kwargs["speaker"] = self.speaker

                wav = self.tts.tts(text, **kwargs)
                t_gen = time.time()

                sr = get_tts_output_sr(self.tts, default=22050)
                self.get_logger().info(f"[TTS] model_output_sr={sr} -> stream_sr={self.stream_sr}")

                # tempo + resample direct vers 16k
                if abs(self.speech_rate - 1.0) > 1e-3 or sr != self.stream_sr:
                    wav, sr2 = sox_tempo_raw(wav, sr_in=sr, rate=self.speech_rate, sr_out=self.stream_sr)
                else:
                    sr2 = sr

                t_sox = time.time()

                self._play_audio(wav, sr2)
                t_play = time.time()

                self.get_logger().info(
                    f"[TTS] gen={t_gen-t0:.3f}s sox={t_sox-t_gen:.3f}s play={t_play-t_sox:.3f}s total={t_play-t0:.3f}s"
                )

            except Exception as e:
                self.get_logger().error(f"Erreur synthèse/lecture : {e}")

            finally:
                self._publish_tts_active(False)
                self.q.task_done()

    def destroy_node(self):
        try:
            self._publish_tts_active(False)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = QboTalkCoquiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⏹️ Arrêt demandé → sortie propre.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
