#!/usr/bin/env python3

from qbo_msgs.msg import FollowerStatus, FaceRecognitionResult
import time


class FaceAdapter:
    """
    Adapter pour convertir les messages FollowerStatus en événements sociaux.
    Utilise le nouveau topic /qbo_face_following/status au lieu de FaceObservation.
    """

    def __init__(self, node):
        self.node = node

        # ===== Params =====
        self.face_stable_time = node.get_parameter("face_stable_time").value
        self.face_lost_timeout = node.get_parameter("face_lost_timeout").value

        # ===== State =====
        self.face_present = False
        self.face_stable = False
        self.last_seen_time = None
        self.first_seen_time = None
        self.last_recognition = None

        # Dernière position 3D du visage
        self.last_face_position = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'distance': 0.0
        }

        # ===== Subscriptions =====
        self.node.create_subscription(
            FollowerStatus,
            '/qbo_face_following/status',
            self._on_follower_status,
            10
        )

        self.node.create_subscription(
            FaceRecognitionResult,
            '/qbo_face_recognition/result',
            self._on_recognition,
            10
        )

        # ===== Timer pour vérifier les timeouts =====
        self.node.create_timer(0.2, self._check_timeouts)

        self.node.get_logger().info("FaceAdapter initialized (using FollowerStatus)")

    # =========================
    # FOLLOWER STATUS CALLBACK
    # =========================
    def _on_follower_status(self, msg: FollowerStatus):
        """
        Traite les messages FollowerStatus du face_follower.
        tracking_state: 0=IDLE, 1=SEARCHING, 2=TRACKING, 3=BLOCKED
        """
        now = time.time()

        # Log de debug pour comprendre pourquoi les événements ne sont pas publiés
        self.node.get_logger().debug(
            f"FollowerStatus: state={msg.tracking_state}, faces={msg.faces_detected}, "
            f"dist={msg.face_distance:.2f}, pos=({msg.face_x:.2f}, {msg.face_y:.2f}, {msg.face_z:.2f})"
        )

        # Le visage est détecté si on est en mode TRACKING
        valid = (
            msg.tracking_state == 2  # TRACKING
            and msg.faces_detected > 0
            and msg.face_distance > 0.0
        )

        if not valid:
            # if msg.tracking_state != 2 and msg.faces_detected > 0:
            #     self.node.get_logger().warn(
            #         f"⚠️  Face detected but tracking_state={msg.tracking_state} (expected 2=TRACKING). "
            #         f"faces_detected={msg.faces_detected}, distance={msg.face_distance:.2f}m"
            #     )
            return

        # Sauvegarder la position 3D du visage
        self.last_face_position = {
            'x': msg.face_x,
            'y': msg.face_y,
            'z': msg.face_z,
            'distance': msg.face_distance,
            'head_pan': msg.head_pan,
            'head_tilt': msg.head_tilt
        }

        # Première détection
        if not self.face_present:
            self.face_present = True
            self.first_seen_time = now
            self._publish_event("FACE_APPEARED", msg)

        self.last_seen_time = now

        # Vérification de stabilité
        if (
            not self.face_stable
            and self.first_seen_time is not None
            and (now - self.first_seen_time) > self.face_stable_time
        ):
            self.face_stable = True
            self._publish_event("FACE_STABLE", msg)

    # =========================
    # RECOGNITION CALLBACK
    # =========================
    def _on_recognition(self, msg: FaceRecognitionResult):
        """Traite les résultats de reconnaissance faciale."""
        now = time.time()

        if msg.known:
            self.last_recognition = {
                "name": msg.name,
                "timestamp": now,
                "confidence": msg.similarity,
                "face_id": msg.face_id
            }

            self.node.get_logger().info(
                f"Recognition cached: {msg.name} (confidence={msg.similarity:.2f})"
            )

        self._publish_recognition_event(msg)

    def _publish_recognition_event(self, rec_msg):
        """Publie un événement de reconnaissance de personne."""

        # Filtrer les reconnaissances invalides:
        # 1. Ignorer les inconnus (unknown)
        # 2. Ignorer les confiances < 0.6 (trop faible)
        if not rec_msg.known or rec_msg.name == "unknown" or rec_msg.similarity < 0.6:
            self.node.get_logger().debug(
                f"Recognition ignored: name={rec_msg.name}, known={rec_msg.known}, "
                f"conf={rec_msg.similarity:.2f} (threshold=0.6)"
            )
            return

        msg = self.node._create_event_msg(
            event_type="PERSON_RECOGNIZED",
            source="face_recognition",
            payload={
                "known": rec_msg.known,
                "similarity": rec_msg.similarity
            }
        )

        msg.person_id = str(rec_msg.face_id)
        msg.person_name = rec_msg.name
        msg.confidence = rec_msg.similarity

        self.node.pub_event.publish(msg)

        self.node.get_logger().info(
            f"EVENT → PERSON_RECOGNIZED ({rec_msg.name}, conf={rec_msg.similarity:.2f})"
        )

    # =========================
    # TIMEOUT CHECK
    # =========================
    def _check_timeouts(self):
        """Vérifie si le visage a été perdu (timeout)."""
        now = time.time()

        if self.face_present and self.last_seen_time is not None:
            if (now - self.last_seen_time) > self.face_lost_timeout:
                self.face_present = False
                self.face_stable = False
                self.first_seen_time = None
                self.last_seen_time = None
                self.last_recognition = None

                self._publish_event("FACE_LOST", None)

    # =========================
    # EVENT PUBLISHER
    # =========================
    def _publish_event(self, event_type, follower_msg):
        """Publie un événement social lié au visage."""

        payload = {}
        person_id = ""
        person_name = ""
        confidence = 0.0

        # Ajouter les informations de position 3D si disponible
        if follower_msg:
            payload = {
                "face_x": self.last_face_position['x'],
                "face_y": self.last_face_position['y'],
                "face_z": self.last_face_position['z'],
                "distance": self.last_face_position['distance'],
                "head_pan": self.last_face_position['head_pan'],
                "head_tilt": self.last_face_position['head_tilt'],
                "tracking_state": follower_msg.tracking_state,
                "head_enabled": follower_msg.head_movement_enabled,
                "rotation_enabled": follower_msg.base_rotation_enabled
            }

            # Ajouter la reconnaissance si récente (< 2 secondes)
            if self.last_recognition:
                now = time.time()
                age = now - self.last_recognition["timestamp"]

                if age < 2.0:
                    person_name = self.last_recognition["name"]
                    person_id = str(self.last_recognition["face_id"])
                    confidence = self.last_recognition["confidence"]
                    payload["recognized_name"] = person_name
                    payload["recognition_age"] = age

        msg = self.node._create_event_msg(
            event_type=event_type,
            source="face_tracking",
            payload=payload
        )

        msg.person_id = person_id
        msg.person_name = person_name
        msg.confidence = confidence

        self.node.pub_event.publish(msg)

        # Log enrichi avec position 3D
        if follower_msg:
            self.node.get_logger().info(
                f"EVENT → {event_type} (pos=[{self.last_face_position['x']:.2f}, "
                f"{self.last_face_position['y']:.2f}, {self.last_face_position['z']:.2f}]m, "
                f"dist={self.last_face_position['distance']:.2f}m, person={person_name or 'unknown'})"
            )
        else:
            self.node.get_logger().info(f"EVENT → {event_type}")
