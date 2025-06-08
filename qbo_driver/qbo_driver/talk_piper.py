#!/usr/bin/env python3
"""
ROS 2 Node: Intégration directe de Piper TTS sans serveur HTTP.
- Utilise PiperVoice (piper-tts) avec chargement unique du modèle en mémoire.
- Langue dynamique via topic /system_lang (synchronisation inter-nœuds)
- Volume dynamique via topic /out_volume
- Mute micro, resampling intelligent, audio en RAM
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
import unicodedata

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
            return dev["name"]
    return None

def find_default_output():
    """
    Récupère le périphérique de sortie audio par défaut.
    Retourne :
      - sd_index (int) pour `sounddevice`
      - pulse_sink (str) pour `pactl`
    """
    # Pour sounddevice
    try:
        default_output = sd.default.device[1]  # (input, output)
        dev = sd.query_devices(default_output)
        print(f"[INFO] 🎧 Device par défaut détecté (sounddevice): {dev['name']}")
    except Exception as e:
        print(f"[WARN] Échec détection sounddevice: {e}")
        default_output = None

    # Pour pactl
    try:
        pulse_sink = subprocess.check_output(
            ["pactl", "get-default-sink"], text=True
        ).strip()
        print(f"[INFO] 🔊 Sink PulseAudio par défaut : {pulse_sink}")
    except Exception as e:
        print(f"[WARN] Échec détection PulseAudio: {e}")
        pulse_sink = None

    return default_output, pulse_sink

def find_input_source_name(name_hint: str) -> str:
    result = subprocess.run(["pactl", "list", "short", "sources"], stdout=subprocess.PIPE, text=True)
    for line in result.stdout.splitlines():
        if name_hint.lower() in line.lower():
            return line.split()[1]
    return "@DEFAULT_SOURCE@"

def find_sink_name(name_hint: str) -> str:
    result = subprocess.run(["pactl", "list", "short", "sinks"], stdout=subprocess.PIPE, text=True)
    for line in result.stdout.splitlines():
        parts = line.split("\t")
        if name_hint.lower() in parts[1].lower():
            return parts[1]
    return "@DEFAULT_SINK@"

class QboTalkNode(Node):
    def __init__(self):
        super().__init__('qbo_talk')

        # Déclaration des paramètres
        self.declare_parameter("audio_out_device_name", "default")
        self.declare_parameter("audio_in_device_name", "default")
        self.declare_parameter("default_lang", "fr")
        self.declare_parameter("audio_playback_volume", 70)
        self.declare_parameter("mute_micro_during_playback", True)

        # Récupération des valeurs
        self.device_name = self.get_parameter("audio_out_device_name").value
        self.input_name_hint = self.get_parameter("audio_in_device_name").value
        self.lang = self.get_parameter("default_lang").value
        self.volume = int(self.get_parameter("audio_playback_volume").value)
        self.mute_micro = bool(self.get_parameter("mute_micro_during_playback").value)

        # Détection périphériques
        if self.device_name == "default":
            self.audio_device, self.pulseaudio_sink = find_default_output()
        else:
            self.audio_device, self.pulseaudio_sink = find_output_device(self.device_name)

        self.input_source = find_input_source_name(self.input_name_hint)

        # Réglage du volume via PulseAudio
        if self.pulseaudio_sink:
            try:
                subprocess.run(
                    ["pactl", "set-sink-volume", self.pulseaudio_sink, f"{self.volume}%"],
                    check=True
                )
            except subprocess.CalledProcessError as e:
                self.get_logger().warn(f"Volume non appliqué : {e}")

        # Chargement du modèle TTS
        try:
            self.voice = PiperVoice.load(VOICE_MODELS[self.lang], use_cuda=False)
        except Exception as e:
            self.get_logger().error(f"Erreur chargement voix '{self.lang}' : {e}")
            raise

        # Logs init
        self.get_logger().info(f"Langue initiale : {self.lang}")
        self.get_logger().info(f"Sortie audio : {self.audio_device}")
        self.get_logger().info(f"Entrée audio : {self.input_source}")
        self.get_logger().info(f"Volume de lecture : {self.volume}%")
        self.get_logger().info(f"Micro coupé pendant playback : {self.mute_micro}")

        # Services & Subscriptions
        self.create_service(Text2Speach, "/qbo_driver/piper2wave_say", self.say_callback)
        self.create_subscription(String, "/system_lang", self.lang_callback, 10)
        self.create_subscription(String, "/system_out_volume", self.out_volume_callback, 10)

    def say_callback(self, request, response):
        text = unicodedata.normalize("NFC", request.sentence.strip())

        self.get_logger().info(f"[TTS] '{text}' via {self.audio_device} ({self.lang})")

        try:
            subprocess.run(["pactl", "set-sink-volume", self.pulseaudio_sink, f"{self.volume}%"], check=True)
        except Exception as e:
            self.get_logger().warn(f"Volume non appliqué : {e}")

        if self.mute_micro:
            try:
                subprocess.run(["pactl", "set-source-mute", self.input_source, "1"], check=True)
            except Exception as e:
                self.get_logger().warn(f"Mute micro impossible : {e}")

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

                if model_sr != device_sr:
                    self.get_logger().warn(f"Sample rate incompatibles ({model_sr} → {device_sr}) → Resampling...")
                    if device_sr < model_sr:
                        self.get_logger().warn(f"Forçage à 48000 Hz pour lecture naturelle")
                        device_sr = 48000
                    num_samples = int(len(samples) * device_sr / model_sr)
                    samples = resample(samples, num_samples).astype(np.int16)

                sd.play(samples, samplerate=device_sr, device=self.audio_device)
                sd.wait()

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

    def lang_callback(self, msg: String):
        lang = msg.data.strip().lower()
        if lang in VOICE_MODELS:
            self.lang = lang
            self.voice = PiperVoice.load(VOICE_MODELS[lang], use_cuda=False)
            self.get_logger().info(f"Langue changée dynamiquement : {lang}")
        else:
            self.get_logger().warn(f"Langue inconnue : {lang}")

    def out_volume_callback(self, msg: String):
        try:
            vol = int(msg.data.strip().replace("%", ""))
            if 0 <= vol <= 150:
                self.volume = vol
                self.get_logger().info(f"🔊 Volume ajusté à {self.volume}%")
            else:
                self.get_logger().warn(f"Valeur de volume hors plage : {msg.data}")
        except ValueError:
            self.get_logger().warn(f"Volume invalide reçu : {msg.data}")


def main():
    rclpy.init()
    node = QboTalkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⏹️ Arrêt demandé → sortie propre.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
