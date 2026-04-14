import queue
import re
import subprocess
import threading
import time
from collections import deque

import numpy as np
import math
import sounddevice as sd
import webrtcvad
from faster_whisper.transcribe import WhisperModel

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from qbo_msgs.msg import ListenResult
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

FIXED_SR = 16000

def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x

def logprob_to_conf(avg_logprob: float) -> float:
    """
    avg_logprob ~ [-2.0 .. -0.1]
    Sigmoid centrée autour de -1.0.
    -1.0 => ~0.5
    -0.5 => ~0.82
    -0.2 => ~0.93
    -1.5 => ~0.18
    """
    x = float(avg_logprob)
    k = 3.0   # pente (plus grand = plus tranché)
    x0 = -1.0 # centre
    return clamp01(1.0 / (1.0 + math.exp(-k * (x - x0))))

def segment_confidence(seg, debug: bool = False, logger=None) -> float:
    """
    Combine plusieurs signaux si présents.
    """
    conf = 0.6
    # base : avg_logprob
    if hasattr(seg, "avg_logprob") and seg.avg_logprob is not None:
        conf = logprob_to_conf(float(seg.avg_logprob))

    # pénalise si "no_speech_prob" est élevé (si dispo)
    if hasattr(seg, "no_speech_prob") and seg.no_speech_prob is not None:
        ns = float(seg.no_speech_prob)  # 0..1
        # pénalité douce (pas trop violente)
        conf *= (1.0 - 0.7 * clamp01(ns))

    # pénalise si compression_ratio trop haut (hallucination possible)
    if hasattr(seg, "compression_ratio") and seg.compression_ratio is not None:
        cr = float(seg.compression_ratio)
        # Whisper recommande souvent de se méfier quand cr > ~2.4
        if cr > 2.4:
            conf *= 0.5
        elif cr > 2.0:
            conf *= 0.8
    if debug and logger is not None:
        logger.debug(
            "seg avg_logprob=%s no_speech_prob=%s compression_ratio=%s => conf=%.3f"
            % (
                getattr(seg, "avg_logprob", None),
                getattr(seg, "no_speech_prob", None),
                getattr(seg, "compression_ratio", None),
                conf,
            )
        )

    return clamp01(conf)

def aggregate_confidence(confs: list[float]) -> float:
    """
    Agrégation simple : moyenne pondérée (ici moyenne).
    Tu peux aussi prendre min(confs) si tu veux être conservateur.
    """
    if not confs:
        return 0.0
    return float(sum(confs) / len(confs))

def get_default_input_info():
    """Récupère l'index et le nom du device par défaut du système."""
    d = sd.query_devices(kind="input")
    devs = sd.query_devices()
    for i, di in enumerate(devs):
        if di["name"] == d["name"] and di.get("max_input_channels", 0) > 0:
            return i, di["name"]
    return None, None

def get_alsa_default_device():
    """
    Trouve le device ALSA "default".
    Ce device est automatiquement routé par PulseAudio vers la source configurée.
    Retourne (index, nom) ou (None, None) si introuvable.
    """
    devs = sd.query_devices()
    hostapis = sd.query_hostapis()

    # Trouver l'index de l'API ALSA
    alsa_api_idx = None
    for idx, api in enumerate(hostapis):
        if api["name"].upper() == "ALSA":
            alsa_api_idx = idx
            break

    if alsa_api_idx is None:
        return None, None

    # Chercher le device "default" de l'API ALSA
    for i, d in enumerate(devs):
        if d["hostapi"] == alsa_api_idx:
            if d.get("max_input_channels", 0) > 0:
                if d["name"].lower() == "default":
                    return i, d["name"]

    return None, None

def configure_pulseaudio_source(pattern: str, logger=None, required=False):
    """
    Configure une source audio PulseAudio comme source par défaut.

    Args:
        pattern: Pattern à chercher dans le nom de la source (ex: "reSpeaker_XVF3800", "Seeed")
        logger: Logger pour les messages
        required: Si True, lève une exception si la source n'est pas trouvée

    Returns:
        Le nom complet de la source PulseAudio si succès, None sinon.

    Raises:
        RuntimeError: Si required=True et la source n'est pas trouvée
    """
    try:
        # Lister les sources
        result = subprocess.run(
            ["pactl", "list", "short", "sources"],
            capture_output=True,
            text=True,
            check=True,
            timeout=5
        )

        if logger:
            logger.info(f"🔍 Recherche d'une source contenant: '{pattern}'")

        # Chercher la source avec le pattern
        matching_source = None
        pattern_lower = pattern.lower()

        for line in result.stdout.splitlines():
            # Format: INDEX NAME MODULE FORMAT CHANNELS SAMPLE_RATE STATE
            parts = line.split()
            if len(parts) >= 2:
                source_name = parts[1]
                source_name_lower = source_name.lower()

                # Filtrer les sources qui ne sont PAS des inputs
                # Exclure les .monitor (sorties) et privilégier alsa_input
                if ".monitor" in source_name_lower:
                    continue  # Skip les monitors (outputs)

                # Chercher le pattern dans le nom (case-insensitive)
                if pattern_lower in source_name_lower:
                    matching_source = source_name
                    if logger:
                        logger.info(f"  ✓ Source trouvée: {source_name}")
                    break

        if not matching_source:
            error_msg = f"Source audio contenant '{pattern}' non trouvée dans PulseAudio"
            if logger:
                logger.error(f"❌ {error_msg}")
                logger.info("Sources disponibles:")
                for line in result.stdout.splitlines():
                    parts = line.split()
                    if len(parts) >= 2:
                        logger.info(f"  - {parts[1]}")

            if required:
                raise RuntimeError(error_msg)
            return None

        # Configurer comme source par défaut
        if logger:
            logger.info(f"⚙️  Configuration de la source par défaut...")

        subprocess.run(
            ["pactl", "set-default-source", matching_source],
            check=True,
            timeout=5,
            capture_output=True
        )

        # Vérifier
        result_info = subprocess.run(
            ["pactl", "info"],
            capture_output=True,
            text=True,
            check=True,
            timeout=5
        )

        for line in result_info.stdout.splitlines():
            if "Default Source" in line:
                current_source = line.split(":")[-1].strip()
                if current_source == matching_source:
                    if logger:
                        logger.info(f"✅ Source configurée: {matching_source}")
                    return matching_source
                else:
                    error_msg = f"Échec de configuration. Source actuelle: {current_source}"
                    if logger:
                        logger.error(f"❌ {error_msg}")
                    if required:
                        raise RuntimeError(error_msg)
                    return None

        return matching_source

    except subprocess.TimeoutExpired:
        error_msg = "Timeout lors de l'exécution de pactl"
        if logger:
            logger.error(f"❌ {error_msg}")
        if required:
            raise RuntimeError(error_msg)
        return None

    except subprocess.CalledProcessError as e:
        error_msg = f"Erreur pactl: {e}"
        if logger:
            logger.error(f"❌ {error_msg}")
        if required:
            raise RuntimeError(error_msg)
        return None

    except FileNotFoundError:
        error_msg = "pactl non disponible (PulseAudio non installé?)"
        if logger:
            logger.error(f"❌ {error_msg}")
        if required:
            raise RuntimeError(error_msg)
        return None

    except Exception as e:
        error_msg = f"Erreur inattendue: {e}"
        if logger:
            logger.error(f"❌ {error_msg}")
        if required:
            raise RuntimeError(error_msg)
        return None

class ListenNode(Node):
    def __init__(self):
        super().__init__("qbo_listen")

        # ---- Params (simplifiés) ----
        self.declare_parameter("pulseaudio_source_pattern", "reSpeaker_XVF3800")  # pattern pour trouver la source dans pactl (ex: "reSpeaker", "XVF3800", "Seeed") - laissez vide pour utiliser le device par défaut du système
        self.declare_parameter("asr_channel_index", 1)
        self.declare_parameter("audio_channels_opened", 2)  # nombre de canaux effectivement ouverts sur le device (ex: Jabra envoie du stéréo même si on veut du mono, donc on ouvre 2 canaux et on sélectionne ensuite)
        self.declare_parameter("sample_rate", FIXED_SR)  # immuable (on vérifie)
        self.declare_parameter("frame_ms", 20)  # 10 ou 20ms conseillé
        self.declare_parameter("vad_enabled", True)
        self.declare_parameter("vad_mode", 2)  # 0..3 (3 = plus agressif)
        self.declare_parameter("silence_frames_end", 20)  # silence pour clore une utterance (20*20ms=400ms)
        self.declare_parameter("hard_stop_frames", 40)  # silence OBLIGATOIRE après timeout (40*20ms=800ms)
        self.declare_parameter("pre_roll_frames", 15)  # contexte avant détection (frames)
        self.declare_parameter("min_utt_ms", 300)  # durée minimale (réduit pour phrases courtes)
        self.declare_parameter("tts_ignore_ms", 250)  # ignore window après fin TTS
        self.declare_parameter("max_utt_ms", 10000)  # timeout souple (attendra hard_stop si parole continue)
        self.declare_parameter("min_speech_ratio", 0.25)

        self.declare_parameter("system_lang", "fr")
        self.declare_parameter("whisper_model", "small")
        self.declare_parameter("whisper_device", "cuda")
        self.declare_parameter("debug_segment_metrics", False)
        self.declare_parameter("min_confidence", 0.5)  # 0.0..1.0

        sr = int(self.get_parameter("sample_rate").value)
        self.asr_channel_index = int(self.get_parameter("asr_channel_index").value)
        self.input_channels_opened = int(self.get_parameter("audio_channels_opened").value)
        self.frame_ms = int(self.get_parameter("frame_ms").value)

        if self.input_channels_opened < 1:
            self.get_logger().warning("audio_channels_opened < 1. Forçage à 1.")
            self.input_channels_opened = 1

        if self.asr_channel_index < 0 or self.asr_channel_index >= self.input_channels_opened:
            self.get_logger().warning(
                f"asr_channel_index={self.asr_channel_index} invalide pour {self.input_channels_opened} canaux. Forçage à 0."
            )
            self.asr_channel_index = 0

        if self.frame_ms not in (10, 20, 30):
            self.get_logger().warning(
                f"frame_ms={self.frame_ms} non supporté par webrtcvad (10/20/30). Forçage à 20."
            )
            self.frame_ms = 20

        if sr != FIXED_SR:
            self.get_logger().warning(
                f"sample_rate demandé={sr}, mais Jabra impose {FIXED_SR}. Forçage à {FIXED_SR}."
            )

        self.vad_enabled = bool(self.get_parameter("vad_enabled").value)
        self.vad_mode = int(self.get_parameter("vad_mode").value)
        self.silence_frames_end = int(self.get_parameter("silence_frames_end").value)
        self.hard_stop_frames = int(self.get_parameter("hard_stop_frames").value)
        self.pre_roll_frames = int(self.get_parameter("pre_roll_frames").value)
        self.min_utt_ms = int(self.get_parameter("min_utt_ms").value)
        self.tts_ignore_ms = int(self.get_parameter("tts_ignore_ms").value)
        self.max_utt_ms = int(self.get_parameter("max_utt_ms").value)
        self.min_speech_ratio = float(self.get_parameter("min_speech_ratio").value)

        self.lang = self.get_parameter("system_lang").value
        model_size = self.get_parameter("whisper_model").value
        whisper_device = self.get_parameter("whisper_device").value
        self.debug_segment_metrics = bool(self.get_parameter("debug_segment_metrics").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value) # 0.0..1.0

        # ---- Blacklist des hallucinations Whisper courantes ----
        self.whisper_hallucinations = {
            "sous-titres réalisés par la communauté d'amara.org",
            "sous-titres réalisés par la communauté d'amara",
            "merci d'avoir regardé cette vidéo",
            "merci de vous être abonné",
            "n'oubliez pas de vous abonner",
            "likez et abonnez-vous",
        }

        # ---- ROS I/O ----
        self.result_pub = self.create_publisher(ListenResult, "/listen", 10)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, "/tts_active", self._on_tts_active, qos)

        self.tts_active = False
        self.ignore_until = 0.0  # timestamp jusqu'auquel on ignore (fin TTS + marge)

        # ---- Recherche du device audio ----
        pulseaudio_source_pattern = self.get_parameter("pulseaudio_source_pattern").value
        self.device_index = None
        self.device_name = None
        self.pulse_source_name = None

        # Si pattern défini → configuration PulseAudio + ALSA "default"
        if pulseaudio_source_pattern:
            self.get_logger().info("🔧 Configuration PulseAudio...")

            # Configure la source PulseAudio (arrêt si échec car required=True)
            self.pulse_source_name = configure_pulseaudio_source(
                pulseaudio_source_pattern,
                logger=self.get_logger(),
                required=True  # Arrête le node si la source n'est pas trouvée
            )

            # Utiliser le device ALSA "default" qui sera automatiquement routé par PulseAudio
            self.device_index, self.device_name = get_alsa_default_device()

            if self.device_index is None:
                raise RuntimeError(
                    f"PulseAudio configuré mais device ALSA 'default' introuvable. "
                    f"Vérifiez votre configuration ALSA/PulseAudio."
                )

            self.get_logger().info(
                f"✅ Configuration réussie: '{self.device_name}' → {self.pulse_source_name}"
            )
        else:
            # Pas de pattern → utiliser le device par défaut du système
            self.device_index, self.device_name = get_default_input_info()
            if self.device_index is None:
                raise RuntimeError("Aucune source d'entree detectee par le systeme.")
            self.get_logger().info(
                f"🎤 Utilisation du device par défaut: '{self.device_name}' (index={self.device_index})"
            )

        # ---- Résumé de la configuration audio ----
        self.get_logger().info("")
        self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        self.get_logger().info(f"🎤 Device sélectionné: [{self.device_index}] {self.device_name}")
        if self.pulse_source_name:
            self.get_logger().info(f"🔊 Source PulseAudio: {self.pulse_source_name}")
        self.get_logger().info(f"📊 Canaux: {self.input_channels_opened} (ASR sur canal {self.asr_channel_index})")
        self.get_logger().info(f"🎼 Sample rate: {FIXED_SR} Hz | Frame: {self.frame_ms} ms")
        self.get_logger().info("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        self.get_logger().info("")

        # ---- Whisper ----
        self.get_logger().info("🔄 Chargement du modèle Whisper...")
        self.voice_model = WhisperModel(
            model_size,
            device=str(whisper_device),
            compute_type="int8_float16",
        )
        self.get_logger().info("✅ Modèle Whisper prêt.")

        # ---- Audio stream config ----
        self.blocksize = int(FIXED_SR * self.frame_ms / 1000)  # ex: 320 pour 20ms
        self.frame_bytes_expected = self.blocksize * 2  # int16 mono

        self.vad = webrtcvad.Vad(self.vad_mode)
        self.buffer_queue: "queue.Queue[bytes]" = queue.Queue(maxsize=300)

        self.stop_evt = threading.Event()
        self.audio_thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.audio_thread.start()

        # ---- Diagnostic minimal : hardware_id + OK ----
        self._publish_diagnostic_minimal()

        # ---- Timer pour heartbeat diagnostic (toutes les 10 secondes) ----
        self.diagnostic_timer = self.create_timer(10.0, self._publish_diagnostic_minimal)

    def _publish_diagnostic_minimal(self):
        """Publie un diagnostic minimal : hardware_id depuis YAML, OK si actif, ERROR sinon."""
        try:
            # Hardware_id = pattern attendu depuis le YAML
            expected_pattern = self.get_parameter("pulseaudio_source_pattern").value
            hardware_id = expected_pattern if expected_pattern else "unknown"

            # Vérifier si le hardware attendu est bien actif
            if self.pulse_source_name and expected_pattern:
                # Vérifier si le pattern est présent dans la source active (case-insensitive)
                if expected_pattern.lower() in self.pulse_source_name.lower():
                    level = DiagnosticStatus.OK
                    message = "System input listening (source active)"
                else:
                    level = DiagnosticStatus.ERROR
                    message = f"Incorrect hardware: {self.pulse_source_name}"
            else:
                # Pas de pattern défini ou pas de source → WARNING
                level = DiagnosticStatus.WARN
                message = "Audio configuration not defined"

            # Créer et publier le diagnostic
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()

            status = DiagnosticStatus()
            status.level = level
            status.name = "qbo_listen"
            status.message = message
            status.hardware_id = hardware_id

            diag_array.status = [status]
            self.diagnostic_pub.publish(diag_array)

            # Log adapté au niveau
            if not hasattr(self, '_diagnostic_published'):
                if level == DiagnosticStatus.OK:
                    self.get_logger().info(f"📊 Diagnostic: {hardware_id} [OK]")
                elif level == DiagnosticStatus.ERROR:
                    self.get_logger().error(f"📊 Diagnostic: {hardware_id} [ERROR] - {message}")
                else:
                    self.get_logger().warn(f"📊 Diagnostic: {hardware_id} [WARN] - {message}")
                self._diagnostic_published = True
            else:
                self.get_logger().debug(f"Heartbeat: {hardware_id} [niveau={level}]")
        except Exception as e:
            self.get_logger().warning(f"Erreur publication diagnostic: {e}")

    def _on_tts_active(self, msg: Bool):
        now = time.monotonic()
        if msg.data:
            self.tts_active = True
        else:
            self.tts_active = False
            self.ignore_until = now + (self.tts_ignore_ms / 1000.0)

    def _audio_cb(self, indata, frames, t, status):
        if status:
            self.get_logger().warning(f"Audio status: {status}")

        pcm = np.frombuffer(indata, dtype=np.int16)

        # frames * channels doit matcher
        exp = frames * self.input_channels_opened
        if pcm.size != exp:
            return

        pcm = pcm.reshape(frames, self.input_channels_opened)
        mono = pcm[:, self.asr_channel_index]

        try:
            self.buffer_queue.put_nowait(mono.tobytes())
        except queue.Full:
            pass

    def _listen_loop(self):
        self.get_logger().info(
            f"Ecoute active sur: {self.device_name}"
        )

        pre_roll = deque(maxlen=self.pre_roll_frames)  # contexte avant détection

        self.get_logger().info(
            f"⚙️  Pre-roll: {self.pre_roll_frames * self.frame_ms}ms | "
            f"Silence end: {self.silence_frames_end * self.frame_ms}ms | "
            f"Hard stop: {self.hard_stop_frames * self.frame_ms}ms | "
            f"Max utt: {self.max_utt_ms}ms"
        )

        def drain_queue(max_items=5000):
            """Vide la queue rapidement pour éviter de traiter du backlog."""
            drained = 0
            while drained < max_items:
                try:
                    self.buffer_queue.get_nowait()
                    drained += 1
                except queue.Empty:
                    break
            return drained

        with sd.RawInputStream(
            samplerate=FIXED_SR,
            blocksize=self.blocksize,
            device=self.device_index,
            dtype="int16",
            channels=self.input_channels_opened,
            latency="high",
            callback=self._audio_cb,
        ):
            while not self.stop_evt.is_set():

                # ---------- Ignore TTS / ignore window (drain + reset) ----------
                now = time.monotonic()
                if self.tts_active or now < self.ignore_until:
                    drained = drain_queue()
                    pre_roll.clear()
                    # petite pause pour ne pas boucler à vide
                    if drained:
                        self.get_logger().debug(
                            f"⏭️ Ignoring audio (TTS/ignore). Drained {drained} frames"
                        )
                    time.sleep(0.02)
                    continue

                audio_frames = deque()
                pre_roll.clear()
                silent = 0
                speaking = False
                start_t = None
                speech_frames = 0
                total_frames = 0
                seg_start = time.monotonic()
                timeout_reached = False

                # ----------------- CAPTURE SEGMENT -----------------
                while not self.stop_evt.is_set():

                    # Re-check TTS inside capture loop too (important!)
                    now = time.monotonic()
                    if self.tts_active or now < self.ignore_until:
                        drained = drain_queue()
                        audio_frames.clear()
                        pre_roll.clear()
                        silent = 0
                        speaking = False
                        if drained:
                            self.get_logger().debug(
                                f"⏭️ TTS started mid-segment. Drained {drained} frames, abort segment"
                            )
                        break  # abandonner ce segment et revenir au while externe

                    try:
                        frame = self.buffer_queue.get(timeout=1.0)
                    except queue.Empty:
                        continue

                    if len(frame) < self.frame_bytes_expected:
                        continue

                    total_frames += 1
                    pre_roll.append(frame)

                    if self.vad_enabled:
                        try:
                            is_speech = self.vad.is_speech(frame, FIXED_SR)  # OK car stream FIXED_SR
                        except Exception:
                            continue
                    else:
                        is_speech = True

                    if is_speech:
                        speech_frames += 1
                        if not speaking:
                            speaking = True
                            start_t = time.monotonic()
                            audio_frames.extend(pre_roll)  # pré-roll
                        audio_frames.append(frame)
                        silent = 0
                    elif speaking:
                        silent += 1

                        # Logique de fin : dépend si timeout atteint ou non
                        if timeout_reached:
                            # Après timeout : besoin de silence LONG (hard_stop) pour forcer la fin
                            if silent >= self.hard_stop_frames:
                                self.get_logger().debug(
                                    f"🛑 Hard stop après timeout (silence={silent*self.frame_ms}ms)"
                                )
                                break
                        else:
                            # Avant timeout : silence court suffit (comportement normal)
                            if silent >= self.silence_frames_end:
                                break

                    # ---- Timeout souple : marquer mais continuer si parole active ----
                    elapsed_ms = int((time.monotonic() - seg_start) * 1000)
                    if elapsed_ms >= self.max_utt_ms and not timeout_reached:
                        timeout_reached = True
                        self.get_logger().debug(
                            f"⏱️  Timeout atteint ({elapsed_ms}ms), attente hard_stop..."
                        )
                        # Ne pas break ! Continuer jusqu'à hard_stop si parole continue

                    # ---- Hard cap absolu : 15 secondes max (sécurité) ----
                    if elapsed_ms >= 15000:
                        self.get_logger().warning(
                            f"⚠️  Hard cap absolu atteint (15s), arrêt forcé"
                        )
                        break

                # si on a abort parce que TTS, recommencer direct
                if self.tts_active or time.monotonic() < self.ignore_until:
                    continue

                if not audio_frames:
                    continue

                utt_ms = int((time.monotonic() - (start_t or time.monotonic())) * 1000)
                if utt_ms < self.min_utt_ms:
                    continue

                # ----------------- FILTRAGE BRUIT (uniquement segments très courts + ratio très bas) -----------------
                speech_ratio = speech_frames / max(1, total_frames)

                # Filtrage léger : seulement pour les segments courts avec quasi aucune voix
                # (évite de traiter du bruit pur mais laisse passer la parole naturelle avec pauses)
                if utt_ms < 800 and speech_ratio < 0.15:
                    self.get_logger().debug(
                        f"🔇 Bruit court ignoré (speech_ratio={speech_ratio:.2f}, utt_ms={utt_ms})"
                    )
                    continue

                # ----------------- TRANSCRIPTION -----------------
                raw = b"".join(audio_frames)
                pcm = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0

                try:
                    segments_iter, _ = self.voice_model.transcribe(
                        pcm,
                        language=self.lang,
                        beam_size=1,
                        vad_filter=False,  # IMPORTANT: on a déjà VAD webrtcvad
                        condition_on_previous_text=False,
                    )
                    segments = list(segments_iter)  # éviter que le générateur se consume
                except Exception as e:
                    self.get_logger().error(f"Whisper error: {e}")
                    continue

                texts, confs = [], []
                for s in segments:
                    t = (getattr(s, "text", "") or "").strip()
                    if t:
                        texts.append(t)
                        confs.append(
                            segment_confidence(
                                s,
                                debug=self.debug_segment_metrics,
                                logger=self.get_logger(),
                            )
                        )

                text = " ".join(texts).strip()
                conf = aggregate_confidence(confs)

                if not text or len(text) < 2 or text in {".", "..", "...", "…"}:
                    continue

                # Filtrer les hallucinations Whisper courantes
                text_lower = text.lower()
                if text_lower in self.whisper_hallucinations:
                    self.get_logger().info(f"🚫 Hallucination filtrée: {text}")
                    continue

                # Filtrer les phrases trop courtes avec confiance faible
                word_count = len(text.split())
                if word_count <= 2 and conf < 0.65:
                    self.get_logger().info(f"❌ Phrase courte + low conf ({conf:.2f}): {text}")
                    continue

                if conf < self.min_confidence:
                    self.get_logger().info(f"❌ Low confidence ({conf:.2f}): {text}")
                    continue

                self.result_pub.publish(
                    ListenResult(sentence=text, confidence=float(conf))
                )
                self.get_logger().info(
                    f"📝 ({conf:.2f}) {text}  [speech_ratio={speech_ratio:.2f}, utt_ms={utt_ms}]"
                )

    def destroy_node(self):
        self.stop_evt.set()
        if hasattr(self, 'diagnostic_timer') and self.diagnostic_timer:
            self.diagnostic_timer.cancel()
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
        node.audio_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
