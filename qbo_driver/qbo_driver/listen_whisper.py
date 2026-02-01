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

FIXED_SR = 16000  # Jabra: 16k uniquement



def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else 1.0 if x > 1.0 else x

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

def segment_confidence(seg) -> float:
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
    print(f"Debug: seg avg_logprob={getattr(seg, 'avg_logprob', None)} no_speech_prob={getattr(seg, 'no_speech_prob', None)} compression_ratio={getattr(seg, 'compression_ratio', None)} => conf={conf}")

    return clamp01(conf)

def aggregate_confidence(confs: list[float]) -> float:
    """
    Agrégation simple : moyenne pondérée (ici moyenne).
    Tu peux aussi prendre min(confs) si tu veux être conservateur.
    """
    if not confs:
        return 0.0
    return float(sum(confs) / len(confs))


def list_devices(logger):
    devs = sd.query_devices()
    hostapis = sd.query_hostapis()
    logger.info("=== Audio devices (sounddevice) ===")
    for i, d in enumerate(devs):
        api = hostapis[d["hostapi"]]["name"]
        logger.info(
            f"{i:2d}: {d['name']} | api={api} | in={d.get('max_input_channels',0)} out={d.get('max_output_channels',0)} | default_sr={int(d.get('default_samplerate',0))}"
        )


def find_device_index(name_hint: str, want_input=True, want_output=False) -> int:
    """
    Retourne l'index du device dont le nom contient name_hint (case-insensitive).
    Filtre aussi sur capacité input/output si demandé.
    """
    hint = (name_hint or "").lower()
    devs = sd.query_devices()
    hostapis = sd.query_hostapis()

    candidates = []
    for i, d in enumerate(devs):
        if want_input and d.get("max_input_channels", 0) < 1:
            continue
        if want_output and d.get("max_output_channels", 0) < 1:
            continue
        if hint in d["name"].lower():
            api = hostapis[d["hostapi"]]["name"]
            candidates.append((api, i))

    # préférence ALSA si plusieurs matches
    for pref in ("ALSA", "JACK", "PulseAudio"):
        for api, idx in candidates:
            if api == pref:
                return idx

    if candidates:
        return candidates[0][1]

    # dernier fallback: device input par défaut
    d = sd.query_devices(kind="input")
    # sounddevice accepte aussi device=None (défaut), mais on garde une valeur explicite si possible
    for i, di in enumerate(devs):
        if di["name"] == d["name"] and di.get("max_input_channels", 0) > 0:
            return i
    return None


class ListenNode(Node):
    def __init__(self):
        super().__init__("qbo_listen")

        # ---- Params (simplifiés) ----
        self.declare_parameter("audio_device_name", "Jabra SPEAK 410 USB")
        self.declare_parameter("sample_rate", FIXED_SR)  # immuable (on vérifie)
        self.declare_parameter("input_channels", 1)
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
        self.declare_parameter("min_confidence", 0.5)  # 0.0..1.0

        self.device_name = self.get_parameter("audio_device_name").value
        sr = int(self.get_parameter("sample_rate").value)
        self.channels = int(self.get_parameter("input_channels").value)
        self.frame_ms = int(self.get_parameter("frame_ms").value)

        if sr != FIXED_SR:
            self.get_logger().warning(
                f"sample_rate demandé={sr}, mais Jabra impose {FIXED_SR}. Forçage à {FIXED_SR}."
            )
        if self.channels != 1:
            self.get_logger().warning("Jabra input mono attendu. Forçage channels=1.")
            self.channels = 1

        self.vad_enabled = bool(self.get_parameter("vad_enabled").value)
        self.vad_mode = int(self.get_parameter("vad_mode").value)
        self.silence_frames_end = int(self.get_parameter("silence_frames_end").value)
        self.min_utt_ms = int(self.get_parameter("min_utt_ms").value)
        self.tts_ignore_ms = int(self.get_parameter("tts_ignore_ms").value)
        self.max_utt_ms = int(self.get_parameter("max_utt_ms").value)
        self.min_speech_ratio = float(self.get_parameter("min_speech_ratio").value)

        self.lang = self.get_parameter("system_lang").value
        model_size = self.get_parameter("whisper_model").value
        self.min_confidence = float(self.get_parameter("min_confidence").value) # 0.0..1.0

        # ---- ROS I/O ----
        self.result_pub = self.create_publisher(ListenResult, "/listen", 10)
        # self.create_subscription(Bool, "/tts_active", self._on_tts_active, 10)

        self.tts_active = False
        self.ignore_until = 0.0

        # ---- Debug devices ----
        list_devices(self.get_logger())

        self.device_index = find_device_index(self.device_name, want_input=True)
        self.get_logger().info(f"🎤 Using device: '{self.device_name}' index={self.device_index}")

        # ---- Whisper ----
        self.get_logger().info("🔄 Chargement du modèle Whisper...")
        self.voice_model = WhisperModel(
            model_size,
            device="cuda",
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

    # def _on_tts_active(self, msg: Bool):
    #     now = time.monotonic()
    #     if msg.data:
    #         self.tts_active = True
    #     else:
    #         self.tts_active = False
    #         self.ignore_until = now + (self.tts_ignore_ms / 1000.0)

    def _audio_cb(self, indata, frames, t, status):
        if status:
            self.get_logger().warning(f"Audio status: {status}")
        # indata est bytes (RawInputStream) -> on pousse tel quel
        try:
            self.buffer_queue.put_nowait(bytes(indata))
        except queue.Full:
            # drop si congestion, préfère la stabilité
            pass

    def _listen_loop(self):
        self.get_logger().info(
            f"🎹 Listening @ {FIXED_SR} Hz mono (Jabra) | frame={self.frame_ms}ms ..."
        )

        pre_roll = deque(maxlen=5)        # 5 × 20 ms = 100 ms de contexte
        grace_after_noise = 0             # nombre de segments tolérés après bruit

        with sd.RawInputStream(
            samplerate=FIXED_SR,
            blocksize=self.blocksize,
            device=self.device_index,
            dtype="int16",
            channels=1,
            latency="high",
            callback=self._audio_cb,
        ):
            while not self.stop_evt.is_set():

                # Pendant ignore-window : on consomme sans traiter
                now = time.monotonic()
                if self.tts_active or now < self.ignore_until:
                    try:
                        _ = self.buffer_queue.get(timeout=0.5)
                    except queue.Empty:
                        pass
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
                            is_speech = self.vad.is_speech(frame, FIXED_SR)
                        except Exception:
                            continue
                    else:
                        is_speech = True

                    if is_speech:
                        speech_frames += 1

                        if not speaking:
                            speaking = True
                            start_t = time.monotonic()
                            audio_frames.extend(pre_roll)   # 🔑 pré-roll

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
                    grace_after_noise = 2      # fenêtre de grâce
                    audio_frames.clear()
                    pre_roll.clear()
                    continue

                grace_after_noise = max(0, grace_after_noise - 1)

                # ----------------- TRANSCRIPTION -----------------
                raw = b"".join(audio_frames)
                pcm = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0

                segments, _ = self.voice_model.transcribe(
                    pcm,
                    language=self.lang,
                    beam_size=1,
                    vad_filter=False,
                    condition_on_previous_text=False,
                )

                texts, confs = [], []
                for s in segments:
                    t = (s.text or "").strip()
                    if t:
                        texts.append(t)
                        confs.append(segment_confidence(s))

                text = " ".join(texts).strip()
                conf = aggregate_confidence(confs)

                if not text or len(text) < 2 or text in {".", "..", "...", "…"}:
                    continue

                if conf < self.min_confidence:
                    self.get_logger().info(
                        f"❌ Low confidence ({conf:.2f}): {text}"
                    )
                    continue

                self.result_pub.publish(
                    ListenResult(sentence=text, confidence=float(conf))
                )
                self.get_logger().info(
                    f"📝 ({conf:.2f}) {text}  "
                    f"[speech_ratio={speech_ratio:.2f}, utt_ms={utt_ms}]"
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
