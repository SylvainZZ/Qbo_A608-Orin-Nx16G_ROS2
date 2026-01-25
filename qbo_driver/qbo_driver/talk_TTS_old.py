#!/usr/bin/env python3
"""
ROS 2 Node: Coqui TTS (YourTTS) GPU, service compatible Text2Speach.srv
- Chargement unique du modèle (GPU)
- Warmup au démarrage
- Queue + worker (pas de chevauchement)
- Volume via pactl, mute micro pendant playback
- Lecture audio via sounddevice (float32)
"""
import os
import time
import queue
import threading
import subprocess
import unicodedata
import re

import numpy as np
import sounddevice as sd
from scipy.signal import resample

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from qbo_msgs.srv import Text2Speach

from TTS.api import TTS
import tempfile
import soundfile as sf

ACRONYM_LEXICON = {
    # Robot / projet
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
    # exemples
    "JetPack": "Jet pack",
    "NVIDIA": "En vidia",
}

def normalize_text_for_tts(text: str) -> str:
    # 1) espaces propres
    t = " ".join(text.strip().split())

    # 2) normalise les acronymes avec points: Q.B.O / Q. B. O.
    #    => on supprime les points entre lettres (mais on garde le mot)
    t = re.sub(r"\b([A-Z])\.\s*([A-Z])\.\s*([A-Z])\b", r"\1\2\3", t)

    # 3) applique WORD_LEXICON (insensible à la casse si tu veux)
    for k, v in WORD_LEXICON.items():
        t = re.sub(rf"\b{re.escape(k)}\b", v, t)

    # 4) applique ACRONYM_LEXICON (mot entier)
    #    On remplace d’abord les formes les plus longues pour éviter les collisions.
    for k in sorted(ACRONYM_LEXICON.keys(), key=len, reverse=True):
        v = ACRONYM_LEXICON[k]
        t = re.sub(rf"\b{re.escape(k)}\b", v, t)

    return t

def apply_fade(x: np.ndarray, sr: int, ms: float):
    n = int(sr * ms / 1000.0)
    if n <= 1 or len(x) < 2*n:
        return x
    fade = np.linspace(0.0, 1.0, n, dtype=np.float32)
    x[:n] *= fade
    x[-n:] *= fade[::-1]
    return x

def normalize_peak(x: np.ndarray, target_peak: float = 0.90) -> np.ndarray:
    x = x.astype(np.float32, copy=False)
    peak = float(np.max(np.abs(x))) if x.size else 0.0
    if peak < 1e-6:
        return x
    gain = target_peak / peak
    return x * gain

def find_output_device(name_hint: str):
    """Retourne le nom sounddevice du device de sortie correspondant au hint."""
    name_hint = (name_hint or "").lower()
    for dev in sd.query_devices():
        if dev.get("max_output_channels", 0) < 1:
            continue
        if name_hint in dev["name"].lower():
            return dev["name"]
    return None

def find_default_output():
    """Retourne (sounddevice_output_index, pulseaudio_sink)."""
    default_output = None
    pulse_sink = None

    try:
        default_output = sd.default.device[1]
        dev = sd.query_devices(default_output)
        print(f"[INFO] 🎧 Device par défaut (sounddevice): {dev['name']}")
    except Exception as e:
        print(f"[WARN] Échec détection sounddevice: {e}")

    try:
        pulse_sink = subprocess.check_output(["pactl", "get-default-sink"], text=True).strip()
        print(f"[INFO] 🔊 Sink PulseAudio par défaut : {pulse_sink}")
    except Exception as e:
        print(f"[WARN] Échec détection PulseAudio: {e}")

    return default_output, pulse_sink

def find_input_source_name(name_hint: str) -> str:
    result = subprocess.run(["pactl", "list", "short", "sources"], stdout=subprocess.PIPE, text=True)
    for line in result.stdout.splitlines():
        if (name_hint or "").lower() in line.lower():
            return line.split()[1]
    return "@DEFAULT_SOURCE@"

def sox_tempo_raw(wav: np.ndarray, sr_in: int, rate: float, sr_out: int | None = None) -> tuple[np.ndarray, int]:
    """
    Time-stretch via sox tempo -s, sans fichiers (stdin/stdout).
    wav: float32 mono
    sr_out: si fourni, sox resample aussi (utile pour aller directement à 48000)
    Retourne (wav_out, sr_effectif)
    """
    if abs(rate - 1.0) < 1e-3 and (sr_out is None or sr_out == sr_in):
        return np.asarray(wav, dtype=np.float32), sr_in

    wav = np.asarray(wav, dtype=np.float32).reshape(-1)
    out_sr = sr_in if sr_out is None else int(sr_out)

    cmd = [
        "sox", "-q",
        "-t", "raw",
        "-r", str(sr_in),
        "-e", "floating-point",
        "-b", "32",
        "-c", "1",
        "-",                         # stdin
        "-t", "raw",
        "-r", str(out_sr),
        "-e", "floating-point",
        "-b", "32",
        "-c", "1",
        "-",                         # stdout
        "tempo", "-s", str(rate),
    ]

    p = subprocess.run(cmd, input=wav.tobytes(), stdout=subprocess.PIPE, check=True)
    y = np.frombuffer(p.stdout, dtype=np.float32)
    return y, out_sr

class QboTalkCoquiNode(Node):
    def __init__(self):
        super().__init__("qbo_talk_coqui")

        # -------- Stream audio ----------
        self.declare_parameter("stream_samplerate", 48000)
        self.declare_parameter("stream_blocksize", 4096)
        self.declare_parameter("stream_channels", 1)

        self.stream_sr = int(self.get_parameter("stream_samplerate").value)
        self.stream_blocksize = int(self.get_parameter("stream_blocksize").value)
        self.stream_channels = int(self.get_parameter("stream_channels").value)

        # -------- Normalisation ----------
        self.declare_parameter("target_rms", 0.08)
        self.declare_parameter("soft_limit", 2.0)
        self.target_rms = float(self.get_parameter("target_rms").value)
        self.soft_limit = float(self.get_parameter("soft_limit").value)

        # -------- Params node (compat) ----------
        self.declare_parameter("audio_out_device_name", "ReSpeaker Lite")
        self.declare_parameter("audio_in_device_name", "default")
        self.declare_parameter("default_lang", "fr")
        self.declare_parameter("audio_playback_volume", 80)
        self.declare_parameter("mute_micro_during_playback", False)

        # -------- Params Coqui ----------
        self.declare_parameter("tts_model_name", "tts_models/multilingual/multi-dataset/your_tts")
        self.declare_parameter("tts_speaker", "male-en-2")
        self.declare_parameter("tts_language_fr", "fr-fr")
        self.declare_parameter("tts_language_en", "en")
        self.declare_parameter("use_gpu", True)
        self.declare_parameter("warmup_text", "Bonjour.")
        self.declare_parameter("queue_maxsize", 10)
        self.declare_parameter("tts_speaker_wav", "/home/qbo-v2/venvs/tts/qbo_fr_ref_22050.wav")
        self.declare_parameter("speech_rate", 0.80)

        # -------- Read params ----------
        self.device_name = self.get_parameter("audio_out_device_name").value
        self.input_name_hint = self.get_parameter("audio_in_device_name").value
        self.lang = self.get_parameter("default_lang").value
        self.volume = int(self.get_parameter("audio_playback_volume").value)
        self.mute_micro = bool(self.get_parameter("mute_micro_during_playback").value)

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

        # Anti-pop
        self.fade_ms = 12.0
        self.pad_ms = 3.0

        # -------- Audio devices ----------
        if self.device_name == "default":
            self.audio_device, self.pulseaudio_sink = find_default_output()
        else:
            self.audio_device = find_output_device(self.device_name)
            try:
                self.pulseaudio_sink = subprocess.check_output(["pactl", "get-default-sink"], text=True).strip()
            except Exception:
                self.pulseaudio_sink = None

        self.input_source = find_input_source_name(self.input_name_hint)

        # Apply initial volume
        self._apply_volume()

        # --------- Ouvre un stream persistant ---------
        try:
            self.stream = sd.OutputStream(
                device=self.audio_device,
                samplerate=self.stream_sr,
                channels=self.stream_channels,
                dtype="float32",
                blocksize=self.stream_blocksize,
                latency="high",
            )
            self.stream.start()
            # Petit silence pour stabiliser ("réveil")
            self.stream.write(np.zeros(int(self.stream_sr * 0.05), dtype=np.float32))
            self.get_logger().info(f"OutputStream ouvert: sr={self.stream_sr} device={self.audio_device}")
        except Exception as e:
            self.get_logger().error(f"Impossible d'ouvrir OutputStream: {e}")
            self.stream = None

        # -------- Load Coqui model once ----------
        self.get_logger().info(f"Loading Coqui model: {self.model_name} (gpu={self.use_gpu}) ...")
        t0 = time.time()
        self.tts = TTS(model_name=self.model_name, progress_bar=False)
        self.tts.to("cuda" if self.use_gpu else "cpu")
        self.get_logger().info(f"Model loaded in {time.time() - t0:.2f}s")

        # Info model
        self.get_logger().info(f"Available languages: {getattr(self.tts, 'languages', None)}")
        self.get_logger().info(f"Available speakers: {getattr(self.tts, 'speakers', None)}")

        # -------- Cache speaker embedding (if speaker_wav) ----------
        self.speaker_embedding = None
        if self.speaker_wav:
            self._precompute_speaker_embedding()

        # Warmup (important for GPU)
        try:
            self.get_logger().info("Warmup...")
            kwargs = {"language": self._coqui_language()}
            if self.speaker_wav:
                kwargs["speaker_wav"] = self.speaker_wav
            else:
                kwargs["speaker"] = self.speaker

            _ = self.tts.tts(self.warmup_text, **kwargs)

            self.get_logger().info("Warmup done.")
        except Exception as e:
            self.get_logger().warn(f"Warmup failed (continuing): {e}")

        # -------- Queue + worker ----------
        self.q = queue.Queue(maxsize=max(1, self.queue_maxsize))
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        # -------- ROS interfaces ----------
        self.create_service(Text2Speach, "/qbo_driver/say_to_TTS", self.say_callback)
        self.create_subscription(String, "/system_lang", self.lang_callback, 10)
        self.create_subscription(String, "/system_out_volume", self.out_volume_callback, 10)

        self.get_logger().info(f"Langue initiale : {self.lang}")
        self.get_logger().info(f"Sortie audio : {self.audio_device}")
        self.get_logger().info(f"Entrée audio : {self.input_source}")
        self.get_logger().info(f"Volume de lecture : {self.volume}%")
        self.get_logger().info(f"Micro coupé pendant playback : {self.mute_micro}")

    # ---------- Helpers ----------
    def _apply_volume(self):
        if self.pulseaudio_sink:
            try:
                subprocess.run(["pactl", "set-sink-volume", self.pulseaudio_sink, f"{self.volume}%"], check=True)
            except Exception as e:
                self.get_logger().warn(f"Volume non appliqué : {e}")

    def _precompute_speaker_embedding(self):
        try:
            self.get_logger().info("Precomputing speaker embedding (cache)...")

            # Trouver le speaker_manager (YourTTS)
            sm = getattr(self.tts.synthesizer, "speaker_manager", None)
            if sm is None:
                sm = getattr(getattr(self.tts.synthesizer, "tts_model", None), "speaker_manager", None)

            if sm is None:
                raise RuntimeError("speaker_manager introuvable dans synthesizer")

            if not hasattr(sm, "compute_embedding_from_clip"):
                raise RuntimeError("speaker_manager n'a pas compute_embedding_from_clip()")

            # 1) Calculer embedding une fois
            self.speaker_embedding = sm.compute_embedding_from_clip(self.speaker_wav)
            self.get_logger().info("Speaker embedding cached ✅")

            # 2) Monkeypatch : si même speaker_wav -> renvoyer le cache (au lieu de recalculer)
            orig_fn = sm.compute_embedding_from_clip

            cached_path = self.speaker_wav
            cached_emb = self.speaker_embedding

            def patched_compute_embedding_from_clip(path, *args, **kwargs):
                try:
                    if path == cached_path:
                        return cached_emb
                except Exception:
                    pass
                return orig_fn(path, *args, **kwargs)

            sm.compute_embedding_from_clip = patched_compute_embedding_from_clip
            self.get_logger().info("Speaker embedding hook installed ✅ (no recompute for cached wav)")

        except Exception as e:
            self.get_logger().warn(f"Speaker embedding cache/hook failed: {e}")
            self.speaker_embedding = None

    def _mute_mic(self, mute: bool):
        if not self.mute_micro:
            return
        try:
            subprocess.run(["pactl", "set-source-mute", self.input_source, "1" if mute else "0"], check=True)
        except Exception as e:
            self.get_logger().warn(f"{'Mute' if mute else 'Unmute'} micro impossible : {e}")

    def _coqui_language(self) -> str:
        l = (self.lang or "fr").strip().lower()
        if l.startswith("fr"):
            return self.lang_fr
        if l.startswith("en"):
            return self.lang_en
        # pas d'es dans ton your_tts -> fallback fr-fr
        return self.lang_fr

    def _play_audio(self, wav: np.ndarray, sr: int):
        wav = np.asarray(wav).squeeze()
        if wav.ndim != 1:
            wav = wav.reshape(-1, 1)
        wav = wav.astype(np.float32, copy=False)

        # self.get_logger().info(f"DBG target_rms={self.target_rms} soft_limit={self.soft_limit}")

        # Si pas de stream persistant, fallback (au cas où)
        if self.stream is None:
            sd.play(wav, samplerate=sr, device=self.audio_device)
            sd.wait()
            return

        target_sr = self.stream_sr

        # Resample vers le SR du stream
        if sr != target_sr:
            num_samples = int(len(wav) * target_sr / sr)
            wav = resample(wav, num_samples).astype(np.float32)

        # Normalisation RMS (volume perçu)
        rms = float(np.sqrt(np.mean(wav * wav))) if wav.size else 0.0
        if rms > 1e-6:
            wav = wav * (self.target_rms / rms)

        # Limiteur doux pour éviter le clipping dur (monte le volume sans grésillement)
        k = self.soft_limit
        wav = np.tanh(wav * k) / np.tanh(k)

        # Fade + padding léger
        pad_ms = getattr(self, "pad_ms", 3.0)
        pad_n = int(target_sr * pad_ms / 1000.0)
        if pad_n > 0:
            wav = np.concatenate([np.zeros(pad_n, np.float32), wav, np.zeros(pad_n, np.float32)])
        wav = apply_fade(wav, target_sr, self.fade_ms)

        # Écriture dans le stream
        self.stream.write(wav)

    # ---------- ROS callbacks ----------
    def say_callback(self, request, response):
        text = normalize_text_for_tts(unicodedata.normalize("NFC", (request.sentence or "").strip()))
        #text = unicodedata.normalize("NFC", (request.sentence or "").strip())
        if not text:
            response.success = False
            return response

        try:
            self.q.put_nowait(text)
            response.success = True
        except queue.Full:
            self.get_logger().warn("Queue TTS pleine, requête ignorée.")
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
                self._apply_volume()
                self.get_logger().info(f"🔊 Volume ajusté à {self.volume}%")
            else:
                self.get_logger().warn(f"Valeur de volume hors plage : {msg.data}")
        except ValueError:
            self.get_logger().warn(f"Volume invalide reçu : {msg.data}")

    # ---------- Worker loop ----------
    def _worker_loop(self):
        while rclpy.ok():
            try:
                text = self.q.get(timeout=0.2)
            except queue.Empty:
                continue

            # ---- Log what we'll use (speaker vs speaker_wav vs embedding) ----
            # lang = self._coqui_language()

            # if getattr(self, "speaker_embedding", None) is not None:
            #     self.get_logger().info(
            #         f"[TTS] '{text}' (lang={lang} speaker_embedding=cached gpu={self.use_gpu})"
            #     )
            # elif self.speaker_wav:
            #     self.get_logger().info(
            #         f"[TTS] '{text}' (lang={lang} speaker_wav={self.speaker_wav} gpu={self.use_gpu})"
            #     )
            # else:
            #     self.get_logger().info(
            #         f"[TTS] '{text}' (lang={lang} speaker={self.speaker} gpu={self.use_gpu})"
            #     )

            # Volume + mute mic
            self._apply_volume()
            self._mute_mic(True)

            try:
                t0 = time.time()

                # # ---- 1) Generate wav (try embedding first) ----
                # kwargs = {"language": lang}

                # wav = None
                # emb = getattr(self, "speaker_embedding", None)

                # if emb is not None:
                #     # Selon versions, le nom de l'argument change. On essaie 2 variantes.
                #     try:
                #         wav = self.tts.tts(text, speaker_embedding=emb, **kwargs)
                #     except TypeError:
                #         try:
                #             wav = self.tts.tts(text, speaker_embeddings=emb, **kwargs)
                #         except TypeError:
                #             wav = None  # fallback après

                # if wav is None:
                #     if self.speaker_wav:
                #         wav = self.tts.tts(text, speaker_wav=self.speaker_wav, **kwargs)
                #     else:
                #         wav = self.tts.tts(text, speaker=self.speaker, **kwargs)
                kwargs = {"language": self._coqui_language()}
                if self.speaker_wav:
                    kwargs["speaker_wav"] = self.speaker_wav
                else:
                    kwargs["speaker"] = self.speaker

                wav = self.tts.tts(text, **kwargs)


                t_gen = time.time()

                # ---- 2) Time-stretch + resample to stream sr (SoX raw) ----
                sr = getattr(self.tts.synthesizer, "output_sample_rate", 22050)

                if abs(self.speech_rate - 1.0) > 1e-3 or sr != self.stream_sr:
                    wav, sr2 = sox_tempo_raw(wav, sr_in=sr, rate=self.speech_rate, sr_out=self.stream_sr)
                else:
                    sr2 = sr

                t_sox = time.time()

                # ---- 3) Play ----
                self._play_audio(wav, sr2)

                t_play = time.time()

                self.get_logger().info(
                    f"[TTS] gen={t_gen-t0:.3f}s sox={t_sox-t_gen:.3f}s play={t_play-t_sox:.3f}s total={t_play-t0:.3f}s"
                )

            except Exception as e:
                self.get_logger().error(f"Erreur synthèse/lecture : {e}")

            finally:
                self._mute_mic(False)
                self.q.task_done()

def main():
    rclpy.init()
    node = QboTalkCoquiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⏹️ Arrêt demandé → sortie propre.")
    finally:
        try:
            if hasattr(node, "stream") and node.stream is not None:
                node.stream.stop()
                node.stream.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
