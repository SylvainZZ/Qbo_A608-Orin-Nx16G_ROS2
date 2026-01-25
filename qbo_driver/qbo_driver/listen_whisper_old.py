import os
import queue
import threading
import time
from collections import deque
import numpy as np
import sounddevice as sd
import webrtcvad
from scipy.io.wavfile import write
from scipy.signal import resample_poly
from faster_whisper.transcribe import WhisperModel

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from qbo_msgs.msg import ListenResult

VAD_RATES = (16000, 48000, 32000, 8000)

def _resample_to(src: np.ndarray, fs_in: int, fs_out: int) -> np.ndarray:
    if fs_in == fs_out:
        return src
    return resample_poly(src, fs_out, fs_in)

def list_input_devices(logger):
    devs = sd.query_devices()
    logger.info("=== Input devices ===")
    for i, d in enumerate(devs):
        if d["max_input_channels"] > 0:
            logger.info(f"{i:2d}: {d['name']}  api={sd.query_hostapis()[d['hostapi']]['name']}  rate={int(d.get('default_samplerate',0))}")

def find_device(name_hint: str):
    """Préfère ALSA et retourne (index, samplerate par défaut du device)."""
    name_hint = name_hint.lower()
    devs = sd.query_devices()
    hostapis = sd.query_hostapis()
    candidates = []
    for i, d in enumerate(devs):
        if d["max_input_channels"] < 1:
            continue
        api = hostapis[d["hostapi"]]["name"]
        if name_hint in d["name"].lower():
            candidates.append((api, i, int(d.get("default_samplerate", 16000))))
    # ordre de préférence: ALSA > JACK > PulseAudio > le reste
    for pref in ("ALSA", "JACK", "PulseAudio"):
        for api, idx, rate in candidates:
            if api == pref:
                return idx, rate
    # fallback: premier input
    d = sd.query_devices(kind="input")
    return None, int(d.get("default_samplerate", 16000))

class ListenNode(Node):
    CHANNELS = 1
    TARGET_RATE = 16_000
    FRAME_MS = 10
    VAD_SENSITIVITY = 2          # ← défini ici
    SILENCE_THRESHOLD = 10

    def __init__(self):
        super().__init__("qbo_listen")

        self.declare_parameter("audio_in_device_name", "hw:0,0")
        # Paramètre pour volume micro
        self.declare_parameter("mic_volume_percent", 50)
        mic_volume = self.get_parameter("mic_volume_percent").get_parameter_value().integer_value

        self.declare_parameter("system_lang", "fr")
        self.declare_parameter("whisper_model", "small")

        self.lang = self.get_parameter("system_lang").get_parameter_value().string_value
        device_hint = self.get_parameter("audio_in_device_name").get_parameter_value().string_value
        model_size = self.get_parameter("whisper_model").get_parameter_value().string_value

        self.result_pub = self.create_publisher(ListenResult, "/listen", 10)

        # lister les devices pour t'assurer du bon nom à donner (ex: "APE", "wm8960", "analog", etc.)
        list_input_devices(self.get_logger())

        self.device_index, self.input_rate = find_device(device_hint)
        self.get_logger().info(f"🎤 Device index={self.device_index}, native_rate={self.input_rate} Hz")

        # choisir un taux compatible VAD (essaie 16k, sinon 48k, sinon 32k, sinon 8k)
        self.stream_rate = None
        for r in VAD_RATES:
            try:
                with sd.RawInputStream(samplerate=r, channels=1, dtype="int16",
                                    device=self.device_index, blocksize=int(r*0.01)):
                    self.stream_rate = r
                    break
            except Exception:
                continue
        if self.stream_rate is None:
            # dernier recours: on essaie la fréquence native (mais VAD pourrait râler)
            self.stream_rate = self.input_rate
        self.blocksize = int(self.stream_rate * self.FRAME_MS / 1000)
        self.get_logger().info(f"🎚️  Stream samplerate={self.stream_rate} Hz (VAD-compatible)")

        self.vad = webrtcvad.Vad(self.VAD_SENSITIVITY)
        self.buffer_queue = queue.Queue()

        # Ajuster le volume du micro via pactl
        try:
            import subprocess
            source_name = "alsa_input.platform-sound.analog-stereo"
            subprocess.run(["pactl", "set-source-volume", source_name, f"{mic_volume}%"], check=True)
            self.get_logger().info(f"🔊 Volume micro réglé à {mic_volume}% via PulseAudio")
        except Exception as e:
            self.get_logger().warning(f"⚠️ Impossible de régler le volume du micro : {e}")

        self.get_logger().info("🔄 Chargement du modèle Whisper...")
        # self.voice_model = WhisperModel(model_size, device="cuda", compute_type="int8_float16")
        self.voice_model = WhisperModel(
            model_size,
            device="cuda",
            compute_type="int8_float16",   # + rapide / - VRAM
        )
        self.get_logger().info("✅ Modèle Whisper prêt.")

        self.stop_evt = threading.Event()
        self.audio_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.audio_thread.start()

    def _callback(self, indata, frames, t, status):
        if status:
            self.get_logger().warning(f"Audio status: {status}")
        # indata est int16 mono → bytes() direct
        self.buffer_queue.put(bytes(indata))

    def _listen_loop(self):
        self.get_logger().info("🎹 En écoute avec VAD temps-réel...")

        vad = webrtcvad.Vad(2)
        buffer_queue = queue.Queue()

        def audio_callback(indata, frames, time, status):
            if status:
                self.get_logger().warning(f"Audio status: {status}")
            buffer_queue.put(indata.copy())

        # with sd.InputStream(samplerate=self.TARGET_RATE,
        #                     blocksize=self.BLOCKSIZE,
        #                     dtype='float32',
        #                     channels=self.CHANNELS,
        #                     callback=audio_callback,
        #                     device=self.device_index):
        with sd.RawInputStream(
            samplerate=self.stream_rate,
            blocksize=self.blocksize,
            device=self.device_index,
            dtype="int16",
            channels=self.CHANNELS,
            callback=self._callback,
        ):

            while not self.stop_evt.is_set():
                audio_frames = deque()
                silent_chunks = 0
                speaking = False

                while not self.stop_evt.is_set():
                    try:
                        frame = self.buffer_queue.get(timeout=1.0)  # ← bytes int16, 10ms
                    except queue.Empty:
                        continue

                    if len(frame) < self.blocksize * 2:   # 2 bytes/sample
                        continue

                    # VAD au TAUX DU FLUX
                    try:
                        is_speech = self.vad.is_speech(frame, self.input_rate)
                    except Exception as e:
                        self.get_logger().warning(f"VAD error: {e}")
                        continue

                    if is_speech:
                        audio_frames.append(frame)
                        silent_chunks = 0
                        speaking = True
                    elif speaking:
                        silent_chunks += 1
                        if silent_chunks > self.SILENCE_THRESHOLD:
                            break

                if not audio_frames:
                    continue
                raw = b"".join(audio_frames)  # bytes int16 @ self.stream_rate
                pcm = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
                pcm = _resample_to(pcm, self.stream_rate, self.TARGET_RATE)

                # audio_data = np.concatenate(audio_frames)
                # write("debug_float32.wav", self.TARGET_RATE, audio_data)

                # segments, _ = self.voice_model.transcribe(audio_data, language=self.lang, beam_size=1, vad_filter=True)
                segments, _ = self.voice_model.transcribe(
                    pcm,
                    language=self.lang,
                    beam_size=1,
                    vad_filter=False,   # ← webrtcvad déjà fait
                )
                text = " ".join([s.text for s in segments]).strip()

                if text:
                    msg = ListenResult(sentence=text, confidence=0.0)
                    self.result_pub.publish(msg)
                    self.get_logger().info(f"📝 {text}")

    def destroy_node(self):
        self.stop_evt.set()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ListenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⏹️ Arrêt demandé...")
    finally:
        node.stop_evt.set()
        node.audio_thread.join()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
