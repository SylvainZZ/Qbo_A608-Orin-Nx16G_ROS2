#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json

from qbo_msgs.msg import BehaviorIntent, Nose
from qbo_msgs.msg import SocialEvent
from qbo_msgs.srv import SetFollowerStatus


class SocialActionExecutor(Node):

    def __init__(self):
        super().__init__('qbo_social_action_executor')

        self.sub_intent = self.create_subscription(
            BehaviorIntent,
            '/qbo_social/intent',
            self._on_intent,
            10
        )

        self.pub_nose = self.create_publisher(
            Nose,
            '/qbo_arduqbo/nose_ctrl/cmd_nose',
            10
        )

        # Service client pour contrôler le face follower
        self.follower_service = self.create_client(
            SetFollowerStatus,
            '/qbo_face_following/set_follower_status'
        )

        self.get_logger().info("Waiting for face follower service...")
        self.follower_service.wait_for_service(timeout_sec=5.0)
        self.get_logger().info("SocialActionExecutor started")


    # =========================
    def _on_intent(self, intent: BehaviorIntent):

        self.get_logger().info(
            f"EXEC → {intent.intent_type}"
        )

        # ===== FACE TRACKING ACTIONS =====
        if intent.intent_type == "TRACK_FACE_HEAD_ONLY":
            self._set_follower_control(enable_head=True, enable_base=False)

        elif intent.intent_type == "TRACK_FACE_FULL":
            self._set_follower_control(enable_head=True, enable_base=True)

        elif intent.intent_type == "STOP_FACE_TRACKING":
            self._set_follower_control(enable_head=False, enable_base=False)

        elif intent.intent_type == "START_PERSON_SEARCH":
            # Active la rotation de base pour faciliter la détection
            # (Future: pourrait inclure un scan actif rotatif)
            self._set_follower_control(enable_head=True, enable_base=True)

        elif intent.intent_type == "STOP_PERSON_SEARCH":
            # Désactive la recherche
            self._set_follower_control(enable_head=False, enable_base=False)

        # ===== SOCIAL ACTIONS =====
        elif intent.intent_type == "GREET_PERSON":

            # 👉 LED verte
            msg = Nose()
            msg.color = 4  # vert
            self.pub_nose.publish(msg)

            # 👉 log (placeholder AIML)
            self.get_logger().info(
                f"Hello {intent.target_person_name}!"
            )

    def _publish_generated_text(self, text, context):

        msg = SocialEvent()

        now = self.get_clock().now().to_msg()

        msg.header.stamp = now
        msg.stamp = now

        msg.event_type = "TEXT_GENERATED"
        msg.source = "action_executor"

        msg.person_id = ""
        msg.person_name = ""
        msg.confidence = 1.0

        msg.payload_json = json.dumps({
            "text": text,
            "context": context
        })

        self.pub_event.publish(msg)

        self.get_logger().info(
            f"EVENT → TEXT_GENERATED: {text}"
        )

    def _handle_speak(self, intent):

        payload = json.loads(intent.payload_json or "{}")

        text = payload.get("text", "")

        # =========================
        # 🔹 CAS 1 : TEXTE DIRECT
        # =========================
        if text:

            self._publish_generated_text(
                text=text,
                context={
                    "source": "direct_intent",
                    "reason": intent.reason
                }
            )
            return

        # =========================
        # 🔹 CAS 2 : DIAGNOSTIC → AIML
        # =========================
        if payload.get("event_type") == "diagnostic":

            text = self._query_aiml_diagnostic(payload)

            if text:

                self._publish_generated_text(
                    text=text,
                    context={
                        "source": "aiml_diagnostic",
                        "key": payload.get("key", ""),
                        "severity": payload.get("severity", ""),
                        "reason": intent.reason
                    }
                )

    # =========================
    # FOLLOWER CONTROL
    # =========================
    def _set_follower_control(self, enable_head: bool, enable_base: bool):
        """
        Contrôle le face follower via le service SetFollowerStatus.

        Args:
            enable_head: Active/désactive les mouvements de tête (pan/tilt)
            enable_base: Active/désactive la rotation de la base mobile
        """
        if not self.follower_service.service_is_ready():
            self.get_logger().warn("Face follower service not available")
            return

        request = SetFollowerStatus.Request()
        request.enable_head_movement = enable_head
        request.enable_base_rotation = enable_base

        future = self.follower_service.call_async(request)
        future.add_done_callback(
            lambda f: self._follower_response_callback(f, enable_head, enable_base)
        )

    def _follower_response_callback(self, future, enable_head: bool, enable_base: bool):
        """Callback pour la réponse du service SetFollowerStatus."""
        try:
            response = future.result()
            if response.success:
                mode = "FULL" if (enable_head and enable_base) else \
                       "HEAD_ONLY" if enable_head else \
                       "DISABLED"
                self.get_logger().info(
                    f"Follower mode: {mode} | {response.message}"
                )
            else:
                self.get_logger().warn(
                    f"Follower service failed: {response.message}"
                )
        except Exception as e:
            self.get_logger().error(
                f"Service call failed: {str(e)}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SocialActionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()