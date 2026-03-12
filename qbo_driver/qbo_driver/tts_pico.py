#!/usr/bin/env python3
import os
import re
import json
import shutil
import subprocess
import tempfile
import threading
import queue
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool
from std_msgs.msg import String

try:
    from qbo_msgs.srv import Text2Speach  # <-- à adapter si nécessaire
except Exception:
    Text2Speach = None  # on gère l'erreur au runtime

@dataclass
class TTSRequestItem:
    text: str
    lang: str
    done_event: Optional[threading.Event] = None
    result_holder: Optional[dict] = None  # {"ok": bool, "msg": str}


class PicoTTSNode(Node):
    def __init__(self):
        super().__init__("talk_pico")

        if Text2Speach is None:
            raise RuntimeError(
                "Impossible d'importer le service Text2Speach. "
                "Modifiez l'import dans talk_pico.py pour pointer vers votre package."
            )

        # --------------------
        # Paramètres
        # --------------------
        self.declare_parameter("service_name", "/qbo_driver/say_to_TTS")
        self.declare_parameter("tts_active_topic", "/tts_active")

        self.declare_parameter("default_lang", "fr-FR")      # pico: fr-FR / en-US (selon installation)
        self.declare_parameter("allow_dynamic_lang", True)   # si le service a un champ langue (optionnel)

        self.declare_parameter("audio_backend", "pulse")     # "pulse" ou "alsa"
        self.declare_parameter("pulse_device", "")           # ex: "alsa_output.pci-0000_00_1f.3.analog-stereo"
        self.declare_parameter("force_default_pulse_sink", True)  # force pactl set-default-sink au demarrage
        self.declare_parameter("alsa_device", "default")     # ex: "hw:0,0" ou "default"

        self.declare_parameter("audio_playback_volume", 80)
        self.volume = int(self.get_parameter("audio_playback_volume").value)


        self.declare_parameter("sample_rate_hz", 16000)      # cible 16 kHz
        self.declare_parameter("force_mono", True)

        self.declare_parameter("block_on_call", True)        # service synchrone (attend fin lecture)
        self.declare_parameter("queue_size", 30)

        # 1️⃣ Déclarer le paramètre AVANT tout accès
        self.declare_parameter("pronunciation_file", "")

        # 2️⃣ Lire la valeur
        file_param = self.get_parameter("pronunciation_file").value

        self.pronunciation_map = {}

        if not file_param:
            self.get_logger().warn("Paramètre pronunciation_file non défini.")
        else:
            # 3️⃣ Résolution chemin package si relatif
            pkg_share = get_package_share_directory("qbo_driver")

            if not os.path.isabs(file_param):
                file_path = os.path.join(pkg_share, file_param)
            else:
                file_path = file_param

            # 4️⃣ Chargement JSON
            if os.path.exists(file_path):
                try:
                    with open(file_path, "r", encoding="utf-8") as f:
                        self.pronunciation_map = json.load(f)

                    self.get_logger().info(
                        f"Pronunciation map chargé: {len(self.pronunciation_map)} entrées"
                    )

                except Exception as e:
                    self.get_logger().error(
                        f"Erreur chargement pronunciation map: {e}"
                    )
            else:
                self.get_logger().warn(
                    f"Fichier pronunciation_map introuvable: {file_path}"
                )

        # --------------------
        # Publisher /tts_active (latched)
        # --------------------
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL  # “latched”
        topic = self.get_parameter("tts_active_topic").value
        self.tts_active_pub = self.create_publisher(Bool, topic, qos)
        self._publish_tts_active(False)

        # --------------------
        # Service
        # --------------------
        srv_name = self.get_parameter("service_name").value
        self.srv = self.create_service(Text2Speach, srv_name, self._on_say_request)
        self.get_logger().info(f"PicoTTS prêt. Service: {srv_name}, /tts_active: {topic}")

        # --------------------
        # Vérifs binaires
        # --------------------
        self._bin_pico2wave = shutil.which("pico2wave")
        self._bin_sox = shutil.which("sox")
        self._bin_paplay = shutil.which("paplay")
        self._bin_aplay = shutil.which("aplay")
        self._bin_pactl = shutil.which("pactl")

        if not self._bin_pico2wave:
            raise RuntimeError("pico2wave introuvable. Installez: sudo apt install libttspico-utils")

        # --------------------
        # Audio output setup
        # --------------------

        self.pulseaudio_sink = None
        configured_pulse_device = str(self.get_parameter("pulse_device").value).strip()
        force_default = bool(self.get_parameter("force_default_pulse_sink").value)

        if self._bin_pactl:
            try:
                if configured_pulse_device and force_default:
                    set_default = subprocess.run(
                        ["pactl", "set-default-sink", configured_pulse_device],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                        text=True
                    )
                    if set_default.returncode == 0:
                        self.get_logger().info(
                            f"Default sink forcé sur : {configured_pulse_device}"
                        )
                    else:
                        err = (set_default.stderr or set_default.stdout).strip()
                        self.get_logger().warn(
                            f"Impossible de forcer le default sink: {err}"
                        )

                result = subprocess.run(
                    ["pactl", "get-default-sink"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                if result.returncode == 0:
                    default_sink = result.stdout.strip()
                    # Si un sink est configure dans le YAML, on l'utilise pour le volume et la lecture.
                    self.pulseaudio_sink = configured_pulse_device or default_sink

                    self.get_logger().info(f"Default sink Pulse détecté : {default_sink}")
                    if configured_pulse_device:
                        self.get_logger().info(
                            f"Sink Pulse configuré pour TTS : {configured_pulse_device}"
                        )
                else:
                    self.get_logger().warn("Impossible de détecter le sink Pulse.")
                    self.pulseaudio_sink = configured_pulse_device or None
            except Exception as e:
                self.get_logger().warn(f"Erreur détection sink : {e}")
                self.pulseaudio_sink = configured_pulse_device or None
        else:
            self.pulseaudio_sink = configured_pulse_device or None

        self._apply_volume()

        # --------------------
        # Worker queue
        # --------------------
        qsize = int(self.get_parameter("queue_size").value)
        self._q: "queue.Queue[TTSRequestItem]" = queue.Queue(maxsize=qsize)
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        # ---------------------
        # Subscription pour volume dynamique
        # ---------------------
        self.create_subscription(
            String,
            "/system_out_volume",
            self.out_volume_callback,
            10
        )

    # --------------------
    # Utils
    # --------------------
    def _publish_tts_active(self, active: bool):
        msg = Bool()
        msg.data = active
        self.tts_active_pub.publish(msg)

    def _sanitize_text(self, text: str) -> str:
        # Nettoyage minimal et déterministe
        t = text.strip()
        t = re.sub(r"\s+", " ", t)
        return t

    def apply_pronunciation_map(self, text: str) -> str:
        if not self.pronunciation_map:
            return text

        items = sorted(
            self.pronunciation_map.items(),
            key=lambda kv: len(kv[0]),
            reverse=True
        )

        for k, v in items:
            pattern = r"\b" + re.escape(k) + r"\b"
            text = re.sub(pattern, v, text)

        return text

    def normalize_symbols(self, text: str) -> str:
        """
        Normalise symboles techniques avant TTS.
        Stable et déterministe.
        """

        replacements = {
            "°C": " degrés",
            "°c": " degrés",
            "%": " pourcent",
            "kg.cm": " kilogramme centimètre",
            "rpm": " tours par minute",
            "V": " volts",
            "W": " watts",
            "A": " ampères",
        }

        for k, v in replacements.items():
            text = text.replace(k, v)

        return text

    def normalize_numbers(self, text: str) -> str:
        """
        Gestion simple et robuste des nombres techniques.
        """

        # 30:1 → 30 pour 1
        text = re.sub(r"(\d+):(\d+)", r"\1 pour \2", text)

        # Décimales 12.5 → 12 virgule 5
        text = re.sub(r"(\d+)\.(\d+)", r"\1 virgule \2", text)

        # IP → lecture chiffre par chiffre
        def ip_replacer(match):
            ip = match.group(0)
            parts = ip.split(".")
            return " point ".join(" ".join(list(p)) for p in parts)

        text = re.sub(r"\b\d+\.\d+\.\d+\.\d+\b", ip_replacer, text)

        return text

    def preprocess_text(self, text: str) -> str:
        """
        Pipeline de normalisation complète.
        Ordre IMPORTANT.
        """

        text = text.strip()

        text = self.normalize_symbols(text)
        text = self.normalize_numbers(text)
        text = self.apply_pronunciation_map(text)

        # Nettoyage espaces multiples
        text = re.sub(r"\s+", " ", text)

        return text

    def _pick_lang(self, request) -> str:
        default_lang = self.get_parameter("default_lang").value
        allow_dynamic = bool(self.get_parameter("allow_dynamic_lang").value)

        # Si votre service possède un champ langue (ex: request.lang), on l’utilise.
        # Sinon, on reste sur default_lang.
        if allow_dynamic and hasattr(request, "lang") and isinstance(getattr(request, "lang"), str):
            lang = getattr(request, "lang").strip()
            if lang:
                return lang
        return default_lang

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

    def _apply_volume(self):
        if not self.pulseaudio_sink:
            return

        try:
            subprocess.run(
                ["pactl", "set-sink-volume", self.pulseaudio_sink, f"{self.volume}%"],
                check=True
            )
        except Exception as e:
            self.get_logger().warn(f"Volume non appliqué : {e}")

    # --------------------
    # Service callback
    # --------------------
    def _on_say_request(self, request, response):
        # Champ texte: adaptez si nécessaire (text / sentence / data / msg ...)
        if hasattr(request, "text"):
            text = getattr(request, "text")
        elif hasattr(request, "sentence"):
            text = getattr(request, "sentence")
        else:
            # fallback
            text = str(request)

        text = self._sanitize_text(str(text))
        if not text:
            # Réponse "ok" mais rien à dire
            if hasattr(response, "success"):
                response.success = True
            if hasattr(response, "message"):
                response.message = "Empty text"
            return response

        lang = self._pick_lang(request)
        block = bool(self.get_parameter("block_on_call").value)

        item_done = threading.Event() if block else None
        holder = {"ok": False, "msg": ""} if block else None

        item = TTSRequestItem(
            text=text,
            lang=lang,
            done_event=item_done,
            result_holder=holder
        )

        try:
            self._q.put(item, timeout=0.2)
        except queue.Full:
            self.get_logger().warn("Queue TTS pleine, requête rejetée.")
            if hasattr(response, "success"):
                response.success = False
            if hasattr(response, "message"):
                response.message = "TTS queue full"
            return response

        if block:
            item_done.wait()  # attend fin synthèse+lecture
            ok = holder.get("ok", False)
            msg = holder.get("msg", "")
            if hasattr(response, "success"):
                response.success = bool(ok)
            if hasattr(response, "message"):
                response.message = msg
        else:
            # asynchrone: on valide juste l'acceptation
            if hasattr(response, "success"):
                response.success = True
            if hasattr(response, "message"):
                response.message = "Accepted"
        return response

    # --------------------
    # Worker
    # --------------------
    def _worker_loop(self):
        while not self._stop.is_set():
            try:
                item = self._q.get(timeout=0.2)
            except queue.Empty:
                continue

            ok, msg = self._process_item(item)
            if item.result_holder is not None:
                item.result_holder["ok"] = ok
                item.result_holder["msg"] = msg
            if item.done_event is not None:
                item.done_event.set()

            self._q.task_done()

    def _process_item(self, item: TTSRequestItem) -> Tuple[bool, str]:

        text = self.preprocess_text(item.text)

        tmpdir = tempfile.mkdtemp(prefix="pico_tts_")
        wav_path = os.path.join(tmpdir, "tts.wav")
        wav_post_path = os.path.join(tmpdir, "tts_post.wav")
        wav_final_path = os.path.join(tmpdir, "tts_final.wav")

        try:
            # 1️⃣ Génération Pico (toujours 16 kHz)
            cmd_pico = [
                self._bin_pico2wave,
                "-l", item.lang,
                "-w", wav_path,
                text
            ]

            r = subprocess.run(cmd_pico, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            if r.returncode != 0:
                err = (r.stderr or r.stdout).strip()
                self.get_logger().error(f"pico2wave failed: {err}")
                return False, f"pico2wave failed: {err}"

            out_path = wav_path

            # 2 Conversion finale vers 48 kHz (CRUCIAL pour Jetson)
            if self._bin_sox:
                convert_cmd = [
                    self._bin_sox,
                    out_path,
                    "-r", "48000",
                    wav_final_path
                ]

                r = subprocess.run(convert_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

                if r.returncode == 0:
                    out_path = wav_final_path
                else:
                    err = (r.stderr or r.stdout).strip()
                    self.get_logger().warn(f"SoX 48k conversion failed: {err}")

            # 3 Lecture
            self._publish_tts_active(True)
            try:
                self._play_wav(out_path)
            finally:
                self._publish_tts_active(False)

            return True, "OK"

        except Exception as e:
            self.get_logger().error(f"TTS error: {e}")
            return False, f"Exception: {e}"

        finally:
            try:
                shutil.rmtree(tmpdir, ignore_errors=True)
            except Exception:
                pass

    def _play_wav(self, wav_path: str):
        backend = str(self.get_parameter("audio_backend").value).lower().strip()

        if backend == "pulse":
            if not self._bin_paplay:
                self.get_logger().warn("paplay introuvable, fallback ALSA (aplay).")
                backend = "alsa"
            else:
                dev = str(self.get_parameter("pulse_device").value).strip()
                cmd = [self._bin_paplay]
                if dev:
                    cmd += ["--device", dev]
                cmd += [wav_path]
                subprocess.run(cmd, check=False)
                return

        # ALSA
        if not self._bin_aplay:
            raise RuntimeError("Ni paplay ni aplay disponibles pour lire l'audio.")
        alsa_dev = str(self.get_parameter("alsa_device").value).strip() or "default"
        cmd = [self._bin_aplay, "-D", alsa_dev, wav_path]
        subprocess.run(cmd, check=False)

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main():
    rclpy.init()
    node = PicoTTSNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()