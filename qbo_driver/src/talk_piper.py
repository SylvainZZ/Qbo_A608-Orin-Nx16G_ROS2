#!/usr/bin/env python3
"""
ROS 2 Node: Intégration directe de Piper TTS sans serveur HTTP.
- Utilise PiperVoice (piper-tts) avec chargement unique du modèle en mémoire.
- Gère les paramètres ROS : audio_out_device_name, audio_in_device_name, mute_micro_during_playback, system_lang, audio_playback_volume.
- Utilise un modèle différent par langue si souhaité.
- Effectue un resampling si le sample rate du modèle != sample rate du device.
"""
import os
import json
import subprocess
import sounddevice as sd
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from qbo_msgs.srv import Text2Speach
from piper import PiperVoice
import io
import wave
import numpy as np
from scipy.signal import resample

# Modèles disponibles par langue
VOICE_MODELS = {
    "fr": "/home/qbo-v2/piper/voices/fr/fr_FR-tom-medium.onnx",
    "en": "/home/qbo-v2/piper/voices/en/en_US-ryan-low.onnx",
    "es": "/home/qbo-v2/piper/voices/es/es_ES-sharon-low.onnx"
}

def find_output_device(name_hint: str) -> str:
    name_hint = name_hint.lower()
    for dev in sd.query_devices():
        if dev["max_output_channels"] < 1:
            continue
        if name_hint in dev["name"].lower():
            return dev["name"]  # pour sounddevice
    return None

def find_input_source_name(name_hint: str) -> str:
    result = subprocess.run(["pactl", "list", "short", "sources"], stdout=subprocess.PIPE, text=True)
    for line in result.stdout.splitlines():
        if name_hint.lower() in line.lower():
            return line.split()[1]
    return "@DEFAULT_SOURCE@"

class QboTalkNode(Node):
    def __init__(self):
        super().__init__('qbo_talk')

        self.declare_parameter("audio_out_device_name", "usb")
        self.declare_parameter("audio_in_device_name", "jabra")
        self.declare_parameter("mute_micro_during_playback", True)
        self.declare_parameter("system_lang", "fr")
        self.declare_parameter("audio_playback_volume", 100)

        self.lang = self.get_parameter("system_lang").get_parameter_value().string_value
        self.device_name = self.get_parameter("audio_out_device_name").get_parameter_value().string_value
        self.input_name_hint = self.get_parameter("audio_in_device_name").get_parameter_value().string_value
        self.mute_micro = self.get_parameter("mute_micro_during_playback").get_parameter_value().bool_value
        self.volume = self.get_parameter("audio_playback_volume").get_parameter_value().integer_value

        self.input_source = find_input_source_name(self.input_name_hint)
        self.audio_device = find_output_device(self.device_name)
        model_path = VOICE_MODELS.get(self.lang, VOICE_MODELS["fr"])
        self.voice = PiperVoice.load(model_path, use_cuda=False)

        self.get_logger().info(f"Langue initiale : {self.lang}")
        self.get_logger().info(f"Sortie audio : {self.audio_device}")
        self.get_logger().info(f"Entrée audio : {self.input_source}")
        self.get_logger().info(f"Mute micro actif : {self.mute_micro}")
        self.get_logger().info(f"Volume de lecture : {self.volume}%")

        self.create_service(Text2Speach, "/qbo_driver/piper2wave_say", self.say_callback)
        self.create_service(Text2Speach, "/qbo_driver/set_language", self.set_language_callback)
        self.create_subscription(String, "/system_lang", self.lang_callback, 10)

    def say_callback(self, request, response):
        text = request.sentence.strip()

        self.get_logger().info(f"[TTS] '{text}' via {self.audio_device} ({self.lang})")

        if self.mute_micro:
            try:
                subprocess.run(["pactl", "set-source-mute", self.input_source, "1"], check=True)
            except Exception as e:
                self.get_logger().warn(f"Mute micro impossible : {e}")

        try:
            subprocess.run(["pactl", "set-sink-volume", "@DEFAULT_SINK@", f"{self.volume}%"], check=True)
        except Exception as e:
            self.get_logger().warn(f"Volume non appliqué : {e}")

        try:
            buf = io.BytesIO()
            with wave.open(buf, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(self.voice.config.sample_rate)
                self.voice.synthesize(text, wav_file=wf)

            buf.seek(0)
            with wave.open(buf, 'rb') as wf:
                data = wf.readframes(wf.getnframes())
                samples = np.frombuffer(data, dtype=np.int16)
                model_sr = wf.getframerate()

                device_info = sd.query_devices(self.audio_device)
                device_sr = int(device_info["default_samplerate"])

                if device_sr < model_sr:
                    # Évite d'accélérer : on force un upsample vers 48000 Hz
                    self.get_logger().warn(f"Le sample rate du device est trop bas, forçage à 48000 Hz")
                    device_sr = 48000

                num_samples = int(len(samples) * device_sr / model_sr)
                samples = resample(samples, num_samples).astype(np.int16)

            sd.play(samples, samplerate=device_sr, device=self.audio_device)

            if hasattr(self.voice, "last_rtf"):
                self.get_logger().info(f"RTF mesuré ≈ {self.voice.last_rtf:.3f}")

        except Exception as e:
            self.get_logger().error(f"Erreur de synthèse ou lecture : {e}")
            response.success = False
            return response

        if self.mute_micro:
            try:
                subprocess.run(["pactl", "set-source-mute", self.input_source, "0"], check=True)
            except Exception as e:
                self.get_logger().warn(f"Unmute micro impossible : {e}")

        response.success = True
        return response

    def set_language_callback(self, request, response):
        response.success = self.set_language(request.sentence)
        return response

    def lang_callback(self, msg: String):
        self.set_language(msg.data)

    def set_language(self, lang: str) -> bool:
        if lang in VOICE_MODELS:
            self.lang = lang
            model_path = VOICE_MODELS[lang]
            self.voice = PiperVoice.load(model_path, use_cuda=True)
            self.get_logger().info(f"Langue changée : {lang}")
            return True
        else:
            self.get_logger().warn(f"Langue non supportée : {lang}")
            return False


def main():
    rclpy.init()
    node = QboTalkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
