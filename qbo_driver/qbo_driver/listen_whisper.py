import os
import queue
import threading
import time
from collections import deque
import numpy as np
import sounddevice as sd
import webrtcvad
from scipy.io.wavfile import write
from faster_whisper import WhisperModel

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from qbo_msgs.msg import ListenResult

def find_device(name_hint: str):
    name_hint = name_hint.lower()
    for idx, dev in enumerate(sd.query_devices()):
        if dev["max_input_channels"] < 1:
            continue
        if name_hint in dev["name"].lower():
            rate = int(dev.get("default_samplerate", 16000))
            print(f"✨ Found '{dev['name']}' at index {idx}, rate={rate}")
            return idx, rate
    dev = sd.query_devices(kind="input")
    rate = int(dev.get("default_samplerate", 16000))
    return None, rate

class ListenNode(Node):
    TARGET_RATE = 16000
    FRAME_MS = 10
    CHANNELS = 1
    BLOCKSIZE = int(TARGET_RATE * FRAME_MS / 1000)
    SILENCE_THRESHOLD = 10

    def __init__(self):
        super().__init__("qbo_listen")

        self.declare_parameter("audio_in_device_name", "default")
        # Paramètre pour volume micro
        self.declare_parameter("mic_volume_percent", 70)
        mic_volume = self.get_parameter("mic_volume_percent").get_parameter_value().integer_value

        self.declare_parameter("system_lang", "fr")
        self.declare_parameter("whisper_model", "small")

        self.lang = self.get_parameter("system_lang").get_parameter_value().string_value
        device_hint = self.get_parameter("audio_in_device_name").get_parameter_value().string_value
        model_size = self.get_parameter("whisper_model").get_parameter_value().string_value

        self.device_index, _ = find_device(device_hint)

        self.result_pub = self.create_publisher(ListenResult, "/listen", 10)

        # Ajuster le volume du micro via pactl
        try:
            import subprocess
            source_name = "alsa_input.platform-sound.analog-stereo"
            subprocess.run(["pactl", "set-source-volume", source_name, f"{mic_volume}%"], check=True)
            self.get_logger().info(f"🔊 Volume micro réglé à {mic_volume}% via PulseAudio")
        except Exception as e:
            self.get_logger().warning(f"⚠️ Impossible de régler le volume du micro : {e}")

        self.get_logger().info("🔄 Chargement du modèle Whisper...")
        self.voice_model = WhisperModel(model_size, device="cuda", compute_type="int8_float16")
        self.get_logger().info("✅ Modèle Whisper prêt.")

        self.stop_evt = threading.Event()
        self.audio_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.audio_thread.start()

    def _listen_loop(self):
        self.get_logger().info("🎹 En écoute avec VAD temps-réel...")

        vad = webrtcvad.Vad(2)
        buffer_queue = queue.Queue()

        def audio_callback(indata, frames, time, status):
            if status:
                self.get_logger().warning(f"Audio status: {status}")
            buffer_queue.put(indata.copy())

        with sd.InputStream(samplerate=self.TARGET_RATE,
                            blocksize=self.BLOCKSIZE,
                            dtype='float32',
                            channels=self.CHANNELS,
                            callback=audio_callback,
                            device=self.device_index):

            while not self.stop_evt.is_set():
                audio_frames = deque()
                silent_chunks = 0
                speaking = False

                while not self.stop_evt.is_set():
                    try:
                        chunk = buffer_queue.get(timeout=1.0)
                    except queue.Empty:
                        continue

                    pcm_chunk = (chunk[:, 0] * 32768).astype(np.int16).tobytes()

                    is_speech = vad.is_speech(pcm_chunk, self.TARGET_RATE)

                    if is_speech:
                        audio_frames.append(chunk[:, 0])
                        silent_chunks = 0
                        speaking = True
                    elif speaking:
                        silent_chunks += 1
                        if silent_chunks > self.SILENCE_THRESHOLD:
                            break

                if not audio_frames:
                    continue

                audio_data = np.concatenate(audio_frames)
                # write("debug_float32.wav", self.TARGET_RATE, audio_data)

                segments, _ = self.voice_model.transcribe(audio_data, language=self.lang, beam_size=1, vad_filter=True)
                text = " ".join([s.text for s in segments]).strip()

                if text:
                    msg = ListenResult(sentence=text, confidence=0.5)
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
