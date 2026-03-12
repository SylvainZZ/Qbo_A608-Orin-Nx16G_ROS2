import queue
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

def list_input_sources(logger):
    devs = sd.query_devices()
    hostapis = sd.query_hostapis()
    logger.info("=== Audio input sources (sounddevice) ===")
    for i, d in enumerate(devs):
        if d.get("max_input_channels", 0) < 1:
            continue
        api = hostapis[d["hostapi"]]["name"]
        logger.info(
            f"{i:2d}: {d['name']} | api={api} | in={d.get('max_input_channels',0)} | default_sr={int(d.get('default_samplerate',0))}"
        )

def get_default_input_info():
    d = sd.query_devices(kind="input")
    devs = sd.query_devices()
    for i, di in enumerate(devs):
        if di["name"] == d["name"] and di.get("max_input_channels", 0) > 0:
            return i, di["name"]
    return None, None

class ListenNode(Node):
    def __init__(self):
        super().__init__("qbo_listen")

        # ---- Params (simplifiés) ----
        self.declare_parameter("asr_channel_index", 1)
        self.declare_parameter("audio_channels_opened", 2)  # nombre de canaux effectivement ouverts sur le device (ex: Jabra envoie du stéréo même si on veut du mono, donc on ouvre 2 canaux et on sélectionne ensuite)
        self.declare_parameter("sample_rate", FIXED_SR)  # immuable (on vérifie)
        self.declare_parameter("frame_ms", 20)  # 10 ou 20ms conseillé
        self.declare_parameter("vad_enabled", True)
        self.declare_parameter("vad_mode", 2)  # 0..3 (3 = plus agressif)
        self.declare_parameter("silence_frames_end", 20)  # 12*20ms=240ms
        self.declare_parameter("min_utt_ms", 400)
        self.declare_parameter("tts_ignore_ms", 250)  # ignore window après fin TTS
        self.declare_parameter("max_utt_ms", 4000)
        self.declare_parameter("min_speech_ratio", 0.25)

        self.declare_parameter("system_lang", "fr")
        self.declare_parameter("whisper_model", "small")
        self.declare_parameter("whisper_device", "cuda")
        self.declare_parameter("list_input_sources_on_start", True)
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
        self.min_utt_ms = int(self.get_parameter("min_utt_ms").value)
        self.tts_ignore_ms = int(self.get_parameter("tts_ignore_ms").value)
        self.max_utt_ms = int(self.get_parameter("max_utt_ms").value)
        self.min_speech_ratio = float(self.get_parameter("min_speech_ratio").value)

        self.lang = self.get_parameter("system_lang").value
        model_size = self.get_parameter("whisper_model").value
        whisper_device = self.get_parameter("whisper_device").value
        self.list_input_sources_on_start = bool(self.get_parameter("list_input_sources_on_start").value)
        self.debug_segment_metrics = bool(self.get_parameter("debug_segment_metrics").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value) # 0.0..1.0

        # ---- ROS I/O ----
        self.result_pub = self.create_publisher(ListenResult, "/listen", 10)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, "/tts_active", self._on_tts_active, qos)

        self.tts_active = False
        self.ignore_until = 0.0  # timestamp jusqu'auquel on ignore (fin TTS + marge)

        # ---- Input sources ----
        if self.list_input_sources_on_start:
            list_input_sources(self.get_logger())

        self.device_index, self.device_name = get_default_input_info()
        if self.device_index is None:
            raise RuntimeError("Aucune source d'entree par defaut detectee par le systeme.")

        self.get_logger().info(
            f"🎤 Using default input source: '{self.device_name}' index={self.device_index}"
        )

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
            f"🎹 Listening @ {FIXED_SR} Hz mono {self.device_name} | frame={self.frame_ms}ms ..."
        )

        pre_roll = deque(maxlen=5)        # 5 × frame_ms (ex 20ms) = 100 ms de contexte
        grace_after_noise = 0

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
                        if silent >= self.silence_frames_end:
                            break

                    # ---- hard cap anti musique continue ----
                    elapsed_ms = int((time.monotonic() - seg_start) * 1000)
                    if elapsed_ms >= self.max_utt_ms:
                        break

                # si on a abort parce que TTS, recommencer direct
                if self.tts_active or time.monotonic() < self.ignore_until:
                    continue

                if not audio_frames:
                    continue

                utt_ms = int((time.monotonic() - (start_t or time.monotonic())) * 1000)
                if utt_ms < self.min_utt_ms:
                    continue

                # ----------------- FILTRAGE BRUIT -----------------
                speech_ratio = speech_frames / max(1, total_frames)

                # segments courts => toujours tenter
                if utt_ms < 1200:
                    speech_ratio = 1.0

                if speech_ratio < self.min_speech_ratio and grace_after_noise <= 0:
                    self.get_logger().info(
                        f"🔇 Skipped (speech_ratio={speech_ratio:.2f}, utt_ms={utt_ms})"
                    )
                    grace_after_noise = 2
                    continue

                grace_after_noise = max(0, grace_after_noise - 1)

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
