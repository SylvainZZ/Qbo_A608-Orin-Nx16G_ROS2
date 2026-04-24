#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

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
            '/qbo_social/intent_raw',
            10
        )

        self.pub_trace = self.create_publisher(
            DecisionTrace,
            '/qbo_social/decision_trace',
            10
        )

        self.declare_parameter("greet_memory_duration", 60.0)
        self.greet_memory_duration = self.get_parameter("greet_memory_duration").value

        self.declare_parameter("recognition_memory_duration", 3.0)
        self.recognition_memory_duration = self.get_parameter("recognition_memory_duration").value

        self.declare_parameter("session_timeout", 10.0)
        self.session_timeout = self.get_parameter("session_timeout").value

        self.declare_parameter("lost_context_memory_duration", 5.0)
        self.lost_context_memory_duration = self.get_parameter("lost_context_memory_duration").value

        self.declare_parameter("spatial_reassociation_threshold", 0.8)
        self.spatial_reassociation_threshold = self.get_parameter("spatial_reassociation_threshold").value

        # Optionnel seulement si tu veux un garde-fou global
        self.declare_parameter("greet_cooldown", 10.0)
        self.greet_cooldown = self.get_parameter("greet_cooldown").value
        self.last_global_greet_time = 0.0

        self.world = None

        # Mémoire sociale longue
        self.last_greet_by_person = {}   # {"person_name": timestamp}

        # Mémoire technique courte
        self.last_recognized_person = ""
        self.last_recognized_time = 0.0

        # Mémoire de session
        self.session_greeted = False
        self.session_person_name = ""
        self.session_started_at = None
        self.face_lost_at = None  # Timestamp du dernier FACE_LOST

        # Mémoire spatiale pour continuité
        self.last_face_position = None  # {"x": ..., "y": ..., "z": ..., "distance": ...}
        self.pending_lost_context = None  # {"person_name": ..., "lost_at": ..., "position": ...}

        # Timer pour vérifier le timeout de session
        self.create_timer(1.0, self._check_session_timeout)

        self.get_logger().info("SocialBehaviorEngine started")
        self.get_logger().info(f"  - greet_memory_duration: {self.greet_memory_duration}s")
        self.get_logger().info(f"  - recognition_memory_duration: {self.recognition_memory_duration}s")
        self.get_logger().info(f"  - session_timeout: {self.session_timeout}s")
        self.get_logger().info(f"  - lost_context_memory_duration: {self.lost_context_memory_duration}s")
        self.get_logger().info(f"  - spatial_reassociation_threshold: {self.spatial_reassociation_threshold}m")
        self.get_logger().info(f"  - greet_cooldown: {self.greet_cooldown}s")

    def _on_world(self, msg: WorldState):
        self.world = msg

    def _reset_session(self, now: float, reason: str):
        """Reset complet de la session avec trace."""
        session_duration = (now - self.session_started_at) if self.session_started_at else 0.0

        self._publish_trace(
            triggering_event="SESSION_RESET",
            chosen_intent="",
            reason=f"{reason}_duration_{session_duration:.1f}s",
            relevance=0.0,
            suppressed=False
        )

        # Reset mémoire technique / session uniquement
        self.session_person_name = ""
        self.session_greeted = False
        self.session_started_at = None
        self.last_recognized_person = ""
        self.last_recognized_time = 0.0
        self.face_lost_at = None
        self.pending_lost_context = None  # Reset aussi le contexte de perte
        # Note : last_greet_by_person est conservé pour éviter la salutation trop régulière

    def _extract_position_from_event(self, event: SocialEvent) -> dict:
        """Extrait la position 3D depuis le payload d'un événement."""
        try:
            import json
            payload = json.loads(event.payload_json)
            return {
                "x": payload.get("face_x", 0.0),
                "y": payload.get("face_y", 0.0),
                "z": payload.get("face_z", 0.0),
                "distance": payload.get("distance", 0.0)
            }
        except Exception:
            return {"x": 0.0, "y": 0.0, "z": 0.0, "distance": 0.0}

    def _compute_spatial_distance(self, pos1: dict, pos2: dict) -> float:
        """Calcule la distance euclidienne entre deux positions 3D."""
        if not pos1 or not pos2:
            return float('inf')
        
        import math
        dx = pos1["x"] - pos2["x"]
        dy = pos1["y"] - pos2["y"]
        dz = pos1["z"] - pos2["z"]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def _check_spatial_continuity(self, event: SocialEvent, now: float) -> bool:
        """Vérifie la continuité spatiale et temporelle avec le contexte de perte.
        
        Retourne True si la continuité est plausible, False sinon.
        Publie des traces de décision.
        """
        if self.pending_lost_context is None:
            return True  # Pas de contexte de perte, on accepte

        # Vérification temporelle
        time_since_loss = now - self.pending_lost_context["lost_at"]
        if time_since_loss > self.lost_context_memory_duration:
            # Contexte de perte expiré, on nettoie et on accepte
            self.pending_lost_context = None
            return True

        # Vérification de l'identité
        if event.person_name != self.pending_lost_context["person_name"]:
            # Personne différente, on accepte
            return True

        # Vérification spatiale
        new_position = self._extract_position_from_event(event)
        old_position = self.pending_lost_context["position"]
        spatial_distance = self._compute_spatial_distance(new_position, old_position)

        if spatial_distance > self.spatial_reassociation_threshold:
            # Position incohérente : probablement une fausse détection
            self._publish_trace(
                triggering_event=event.event_type,
                chosen_intent="",
                reason=f"spatial_discontinuity_detected_dist_{spatial_distance:.2f}m_time_{time_since_loss:.1f}s",
                relevance=0.6,
                suppressed=True
            )
            self.get_logger().warn(
                f"⚠️ Spatial discontinuity: {event.person_name} detected {spatial_distance:.2f}m away "
                f"from last position after {time_since_loss:.1f}s (threshold={self.spatial_reassociation_threshold}m)"
            )
            return False
        else:
            # Continuité plausible
            self._publish_trace(
                triggering_event=event.event_type,
                chosen_intent="",
                reason=f"spatial_continuity_confirmed_dist_{spatial_distance:.2f}m_time_{time_since_loss:.1f}s",
                relevance=0.8,
                suppressed=False
            )
            # Nettoyer le contexte de perte car continuité confirmée
            self.pending_lost_context = None
            return True

    def _check_session_timeout(self):
        """Vérification périodique du timeout de session."""
        if self.world is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # Si pas de visage ET qu'un face_lost_at existe ET que le timeout est dépassé
        if not self.world.face_present and self.face_lost_at is not None:
            if (now - self.face_lost_at) > self.session_timeout:
                self._reset_session(now, "session_timeout_expired")

    def _try_greet_with_recent_recognition(self, triggering_event: str, now: float, person_id: str = ""):
        # 1) Il faut une personne reconnue en mémoire courte
        if not self.last_recognized_person:
            self._publish_trace(
                triggering_event=triggering_event,
                chosen_intent="",
                reason="ignored_no_recognized_person",
                relevance=0.1,
                suppressed=False
            )
            return

        # 2) La reconnaissance doit être récente
        if (now - self.last_recognized_time) > self.recognition_memory_duration:
            self._publish_trace(
                triggering_event=triggering_event,
                chosen_intent="",
                reason="ignored_recognition_too_old",
                relevance=0.2,
                suppressed=False
            )
            return

        # 3) Le visage doit être stable dans le world model
        if self.world is None or not self.world.face_stable:
            self._publish_trace(
                triggering_event=triggering_event,
                chosen_intent="",
                reason="ignored_face_not_stable",
                relevance=0.2,
                suppressed=False
            )
            return

        # 4) Déjà salué dans la session courante
        if self.session_greeted and self.session_person_name == self.last_recognized_person:
            self._publish_trace(
                triggering_event=triggering_event,
                chosen_intent="",
                reason="suppressed_already_greeted_in_session",
                relevance=0.3,
                suppressed=True
            )
            return

        # 5) Déjà salué récemment dans la mémoire longue
        #    -> ne pas resaluer trop vite après une petite perte / reconnexion
        last_greet_time = self.last_greet_by_person.get(self.last_recognized_person, 0.0)
        if (now - last_greet_time) < self.greet_memory_duration:
            self._publish_trace(
                triggering_event=triggering_event,
                chosen_intent="",
                reason="suppressed_already_greeted_recently",
                relevance=0.4,
                suppressed=True
            )
            return

        # 6) Salutation autorisée
        self._publish_intent(
            intent_type="GREET_PERSON",
            target_person_id=person_id,
            target_person_name=self.last_recognized_person,
            priority=1.0,
            reason="face_stable_with_recent_recognition"
        )

        self._publish_trace(
            triggering_event=triggering_event,
            chosen_intent="GREET_PERSON",
            reason="face_stable_with_recent_recognition",
            relevance=0.9,
            suppressed=False
        )

        # 7) Mettre à jour les mémoires APRÈS le salut
        self.session_person_name = self.last_recognized_person
        self.session_greeted = True
        self.last_greet_by_person[self.last_recognized_person] = now

        # Démarrer la session au moment de la salutation
        if self.session_started_at is None:
            self.session_started_at = now

        # 8) Nettoyage de la mémoire longue
        self._cleanup_greet_memory(now)

        # 9) Reset de la mémoire technique courte uniquement
        self.last_recognized_person = ""
        self.last_recognized_time = 0.0

    def _cleanup_greet_memory(self, now: float):
        to_remove = [
            person for person, timestamp in self.last_greet_by_person.items()
            if (now - timestamp) > self.greet_memory_duration
        ]
        for person in to_remove:
            del self.last_greet_by_person[person]

    def _on_event(self, event: SocialEvent):
        if self.world is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        if event.event_type == "FACE_APPEARED":
            # Vérifier si on revient après une perte courte
            if self.face_lost_at is not None:
                loss_duration = now - self.face_lost_at

                if loss_duration >= self.session_timeout:
                    # Perte longue : reset de session
                    self._reset_session(now, f"long_loss_{loss_duration:.1f}s")
                else:
                    # Perte courte : conserver la session
                    self._publish_trace(
                        triggering_event=event.event_type,
                        chosen_intent="",
                        reason=f"session_preserved_short_loss_{loss_duration:.1f}s",
                        relevance=0.5,
                        suppressed=False
                    )
                    self.face_lost_at = None  # Réinitialiser le timestamp de perte
            return

        elif event.event_type == "PERSON_RECOGNIZED":
            if not event.person_name:
                self._publish_trace(
                    triggering_event=event.event_type,
                    chosen_intent="",
                    reason="ignored_empty_person_name",
                    relevance=0.0,
                    suppressed=False
                )
                return

            # IMPORTANT : Vérifier la continuité spatiale avant d'accepter la reconnaissance
            if not self._check_spatial_continuity(event, now):
                # Discontinuité spatiale détectée : probablement une fausse reconnaissance
                # On ignore cette reconnaissance
                return

            # Reconnaissance acceptée : sauvegarder la position
            self.last_face_position = self._extract_position_from_event(event)

            self.last_recognized_person = event.person_name
            self.last_recognized_time = now

            self._publish_trace(
                triggering_event=event.event_type,
                chosen_intent="",
                reason="person_recognized_memorized",
                relevance=0.5,
                suppressed=False
            )

            # IMPORTANT : si le visage est déjà stable, essayer tout de suite
            self._try_greet_with_recent_recognition(
                triggering_event=event.event_type,
                now=now,
                person_id=event.person_id
            )
            return

        elif event.event_type == "FACE_STABLE":
            # Sauvegarder la position du visage stable
            self.last_face_position = self._extract_position_from_event(event)

            # Vérifier si on revient après une perte courte
            if self.face_lost_at is not None:
                loss_duration = now - self.face_lost_at

                if loss_duration >= self.session_timeout:
                    # Perte longue : reset de session avant de continuer
                    self._reset_session(now, f"long_loss_{loss_duration:.1f}s")
                else:
                    # Perte courte : conserver la session
                    self._publish_trace(
                        triggering_event=event.event_type,
                        chosen_intent="",
                        reason=f"session_preserved_short_loss_{loss_duration:.1f}s",
                        relevance=0.5,
                        suppressed=False
                    )
                    self.face_lost_at = None  # Réinitialiser le timestamp de perte

            self._try_greet_with_recent_recognition(
                triggering_event=event.event_type,
                now=now,
                person_id=event.person_id
            )
            return

        elif event.event_type == "FACE_LOST":
            # Enregistrer le moment de la perte sans resetter immédiatement
            self.face_lost_at = now

            # Sauvegarder le contexte de perte pour continuité spatiale
            # Seulement si on avait une session active avec une personne identifiée
            if self.session_person_name and self.last_face_position:
                self.pending_lost_context = {
                    "person_name": self.session_person_name,
                    "lost_at": now,
                    "position": self.last_face_position.copy()
                }
                self.get_logger().debug(
                    f"Lost context saved: {self.session_person_name} at "
                    f"pos=[{self.last_face_position['x']:.2f}, {self.last_face_position['y']:.2f}, "
                    f"{self.last_face_position['z']:.2f}]m"
                )

            self._publish_trace(
                triggering_event=event.event_type,
                chosen_intent="",
                reason="face_lost_session_pending",
                relevance=0.1,
                suppressed=False
            )
            # Reset de la mémoire technique courte uniquement
            self.last_recognized_person = ""
            self.last_recognized_time = 0.0
            return

    def _cleanup_greet_memory(self, now: float):
        """Supprime les entrées obsolètes du dictionnaire last_greet_by_person."""
        to_remove = [
            person for person, timestamp in self.last_greet_by_person.items()
            if (now - timestamp) > self.greet_memory_duration
        ]
        for person in to_remove:
            del self.last_greet_by_person[person]

    def _publish_intent(self, intent_type: str, target_person_id: str = "",
                        target_person_name: str = "", priority: float = 1.0,
                        reason: str = "", payload: str = "{}"):
        msg = BehaviorIntent()
        now = self.get_clock().now().to_msg()

        msg.header.stamp = now
        msg.stamp = now
        msg.intent_type = intent_type
        msg.target_person_id = target_person_id
        msg.target_person_name = target_person_name
        msg.priority = priority
        msg.reason = reason
        msg.payload_json = payload

        self.pub_intent.publish(msg)

        self.get_logger().info(
            f"INTENT -> {intent_type} ({target_person_name}) reason={reason}"
        )

    def _publish_trace(self, triggering_event: str, chosen_intent: str,
                       reason: str, relevance: float, suppressed: bool):
        msg = DecisionTrace()
        now = self.get_clock().now().to_msg()

        msg.header.stamp = now
        msg.stamp = now
        msg.triggering_event = triggering_event
        msg.chosen_intent = chosen_intent
        msg.reason = reason
        msg.relevance_score = relevance
        msg.speech_allowed = True
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