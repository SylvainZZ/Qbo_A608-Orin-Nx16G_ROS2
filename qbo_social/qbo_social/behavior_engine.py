#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json

from qbo_msgs.msg import SocialEvent, WorldState, BehaviorIntent, DecisionTrace


class SocialBehaviorEngine(Node):

    def __init__(self):
        super().__init__('qbo_social_behavior_engine')

        self.sub_event = self.create_subscription(
            SocialEvent,
            '/qbo_social/events',
            self._on_event,
            10
        )

        self.sub_world = self.create_subscription(
            WorldState,
            '/qbo_social/world_state',
            self._on_world,
            10
        )

        self.pub_intent = self.create_publisher(
            BehaviorIntent,
            '/qbo_social/intent',
            10
        )

        self.pub_trace = self.create_publisher(
            DecisionTrace,
            '/qbo_social/decision_trace',
            10
        )

        # ===== Params =====
        self.declare_parameter("greet_cooldown", 10.0)
        self.declare_parameter("stable_head_stop_delay", 5.0)  # Temps avant d'arrêter la tête si visage stable
        self.declare_parameter("search_timeout", 30.0)  # Temps max de recherche après perte
        self.declare_parameter("diag_cooldown", 5.0)

        self.greet_cooldown = self.get_parameter("greet_cooldown").value
        self.stable_head_stop_delay = self.get_parameter("stable_head_stop_delay").value
        self.search_timeout = self.get_parameter("search_timeout").value
        self.diag_cooldown = self.get_parameter("diag_cooldown").value

        # ===== state =====
        self.world = None
        self.last_greet_time = 0
        self.last_diag_time = {}

        # ===== Face tracking state machine =====
        self.tracking_mode = "IDLE"  # IDLE, TRACKING_FULL, TRACKING_HEAD_ONLY, SEARCHING
        self.face_stable_since = None  # Timestamp when face became stable
        self.face_lost_since = None    # Timestamp when face was lost
        self.head_stopped = False      # Flag to avoid re-sending head stop command

        # ===== Timer pour vérifier périodiquement les conditions de tracking =====
        # Vérifie toutes les 500ms si les délais sont dépassés (stable_head_stop_delay, search_timeout)
        self.tracking_check_timer = self.create_timer(0.5, self._update_tracking_behavior)

        self.get_logger().info("SocialBehaviorEngine started")
        self.get_logger().info(f"  - stable_head_stop_delay: {self.stable_head_stop_delay}s")
        self.get_logger().info(f"  - search_timeout: {self.search_timeout}s")

    # =========================
    def _on_world(self, msg):
        self.world = msg
        # Note: _update_tracking_behavior() est appelé par le timer, pas besoin ici

    # =========================
    def _on_event(self, event: SocialEvent):

        payload = {}
        if event.payload_json:
            try:
                payload = json.loads(event.payload_json)
            except:
                pass

        now = self.get_clock().now().seconds_nanoseconds()[0]

        # =========================
        # FACE TRACKING EVENTS (ne nécessitent pas world)
        # =========================
        if event.event_type == "FACE_APPEARED":
            self.get_logger().info(f"Face appeared (mode was: {self.tracking_mode}) → Start full tracking")
            self.tracking_mode = "TRACKING_FULL"
            self.face_stable_since = None
            self.face_lost_since = None
            self.head_stopped = False
            self._publish_intent(
                intent_type="TRACK_FACE_FULL",
                reason="face_appeared",
                triggering_event="FACE_APPEARED",
                relevance=1.0
            )

        elif event.event_type == "FACE_STABLE":
            if self.face_stable_since is None:
                self.face_stable_since = now
                self.get_logger().info(f"Face stable detected, will stop head in {self.stable_head_stop_delay}s")

            # Si on reçoit FACE_STABLE alors qu'on est en IDLE, c'est que FACE_APPEARED
            # n'a pas été traité (timing de démarrage). On passe en TRACKING_FULL.
            if self.tracking_mode == "IDLE":
                self.get_logger().info("FACE_STABLE received while IDLE → Start full tracking")
                self.tracking_mode = "TRACKING_FULL"
                self.head_stopped = False
                self._publish_intent(
                    intent_type="TRACK_FACE_FULL",
                    reason="face_stable_during_idle",
                    triggering_event="FACE_STABLE",
                    relevance=1.0
                )

        elif event.event_type == "FACE_LOST":
            self.get_logger().info("Face lost → Start search mode")
            self.tracking_mode = "SEARCHING"
            self.face_stable_since = None
            self.face_lost_since = now
            self.head_stopped = False
            self._publish_intent(
                intent_type="START_PERSON_SEARCH",
                reason="face_lost",
                triggering_event="FACE_LOST",
                relevance=1.0
            )

        # =========================
        # PERSON RECOGNITION (nécessite world)
        # =========================
        if event.event_type == "PERSON_RECOGNIZED":

            if self.world is None:
                return

            if event.person_name == "":
                return

            # cooldown anti-spam
            if (now - self.last_greet_time) < self.greet_cooldown:
                return

            # condition sociale
            if not self.world.face_stable:
                return

            self.last_greet_time = now

            self._publish_intent(
                intent_type="GREET_PERSON",
                name=event.person_name,
                reason="recognized_person",
                triggering_event="PERSON_RECOGNIZED",
                relevance=0.8
            )

        # =========================
        # DIAGNOSTICS (nécessite world)
        # =========================

        if event.event_type.startswith("DIAGNOSTIC"):

            if self.world is None:
                return

            severity = payload.get("severity", "info")
            key = payload.get("key", "")

            last = self.last_diag_time.get(key, 0)

            if (now - last) < self.diag_cooldown:
                return

            self.last_diag_time[key] = now

            # 🔴 ERROR → priorité haute
            if severity == "error" and payload.get("active", True):

                self._publish_intent(
                    intent_type="SPEAK",
                    reason=f"diagnostic_error:{key}",
                    name=""
                )

            # 🟡 WARNING → seulement si idle
            elif severity == "warning" and payload.get("active", True):

                if self.world.mode == "IDLE":

                    self._publish_intent(
                        intent_type="SPEAK",
                        reason=f"diagnostic_warning:{key}",
                        name=""
                    )

            # 🟢 RESOLVED → optionnel
            elif payload.get("active") is False:

                self._publish_intent(
                    intent_type="SPEAK",
                    reason=f"diagnostic_resolved:{key}",
                    name=""
                )
        # ========================
        # TEXT GENERATED (AIML) (nécessite world)
        # ========================

        if event.event_type == "TEXT_GENERATED":

            if self.world is None:
                return

            payload = json.loads(event.payload_json)
            text = payload.get("text", "")

            # 🔥 décision sociale ici
            if self.world.mode == "IDLE":

                self._publish_intent(
                    intent_type="SPEAK",
                    payload={"text": text},
                    reason="text_generated"
                )

    # =========================
    def _publish_intent(self, intent_type, name="", reason="", payload=None,
                       triggering_event="", relevance=1.0, suppressed=False):

        msg = BehaviorIntent()
        now = self.get_clock().now().to_msg()

        msg.header.stamp = now
        msg.stamp = now
        msg.intent_type = intent_type
        msg.target_person_name = name
        msg.priority = 1.0
        msg.reason = reason

        msg.payload_json = json.dumps(payload or {})

        self.pub_intent.publish(msg)

        self.get_logger().info(
            f"INTENT → {intent_type} ({name})"
        )

        # Publier trace de décision pour debugging
        self._publish_trace(
            triggering_event=triggering_event,
            chosen_intent=intent_type,
            reason=reason,
            relevance=relevance,
            suppressed=suppressed
        )

    # =========================
    # TRACKING BEHAVIOR STATE MACHINE
    # =========================
    def _update_tracking_behavior(self):
        """Update tracking behavior based on current world state and timers."""
        if self.world is None:
            return

        now = self.get_clock().now().seconds_nanoseconds()[0]

        # =========================
        # ÉTAT 1: Face stable depuis X secondes → arrêter mouvement tête
        # =========================

        # Log de debug détaillé pour comprendre pourquoi l'arrêt ne se déclenche pas
        if self.tracking_mode == "TRACKING_FULL" and self.face_stable_since is not None:
            elapsed = now - self.face_stable_since

            # Log toutes les secondes avec détails des conditions
            if int(elapsed) % 1 == 0 and elapsed > 0:
                self.get_logger().info(
                    f"[TIMER CHECK] mode={self.tracking_mode}, "
                    f"face_stable_since={self.face_stable_since is not None}, "
                    f"world.face_stable={self.world.face_stable}, "
                    f"head_stopped={self.head_stopped}, "
                    f"elapsed={elapsed:.1f}s/{self.stable_head_stop_delay}s"
                )

        if (self.tracking_mode == "TRACKING_FULL" and
            self.face_stable_since is not None and
            self.world.face_stable and
            not self.head_stopped):

            elapsed = now - self.face_stable_since

            if elapsed >= self.stable_head_stop_delay:
                self.get_logger().info(
                    f"✓ All conditions met! Face stable for {elapsed:.1f}s → Stop head movement")
        # =========================
        # ÉTAT 2: Recherche depuis 30s sans succès → arrêter
        # =========================
        if (self.tracking_mode == "SEARCHING" and
            self.face_lost_since is not None):

            elapsed = now - self.face_lost_since

            # Log de debug toutes les 5 secondes
            if int(elapsed) % 5 == 0 and elapsed > 0:
                self.get_logger().debug(
                    f"Searching for {elapsed:.1f}s (will timeout at {self.search_timeout}s)"
                )

            if elapsed >= self.search_timeout:
                self.get_logger().info(
                    f"Search timeout ({self.search_timeout}s) → Stop searching"
                )
                self.tracking_mode = "IDLE"
                self.face_lost_since = None
                self._publish_intent(
                    intent_type="STOP_PERSON_SEARCH",
                    reason="search_timeout",
                    triggering_event="timer_check",
                    relevance=1.0
                )

    # =========================
    # DECISION TRACE (DEBUG)
    # =========================
    def _publish_trace(self, triggering_event, chosen_intent, reason, relevance, suppressed=False):
        """Publie une trace de décision pour debugging."""
        msg = DecisionTrace()
        msg.stamp = self.get_clock().now().to_msg()
        msg.triggering_event = triggering_event
        msg.chosen_intent = chosen_intent
        msg.reason = reason
        msg.relevance_score = relevance
        msg.speech_allowed = (self.world.mode != "ENGAGED") if self.world else True
        msg.suppressed_by_cooldown = suppressed

        self.pub_trace.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SocialBehaviorEngine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()