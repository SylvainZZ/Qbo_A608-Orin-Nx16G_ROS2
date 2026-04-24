#!/usr/bin/env python3
"""
Action Executor - Orchestrateur principal des actions comportementales.
Reçoit les intents, route vers les handlers appropriés, et publie les résultats.
"""

import rclpy
from rclpy.node import Node
import json
import sys
import os

from qbo_msgs.msg import BehaviorIntent, SocialEvent
from std_msgs.msg import Header

# Import des clients
sys.path.append(os.path.join(os.path.dirname(__file__), 'clients'))
from follower_client import FollowerClient
from bringup_client import BringupClient
from display_client import DisplayClient
from tts_client import TTSClient

# Import des actions
sys.path.append(os.path.join(os.path.dirname(__file__), 'actions'))
from tracking_actions import (
    TrackFaceHeadOnlyAction, TrackFaceFullAction,
    StopFaceTrackingAction, StartPersonSearchAction, StopPersonSearchAction
)
from display_actions import (
    SetNoseColorAction, ShowSmileAction,
    DisplayTextAction, ClearDisplayAction
)
from speech_actions import SayShortPhraseAction, SpeakTextAction, StopSpeakingAction
from social_actions import GreetPersonAction, ExpressRecognitionAction, ExpressEmotionAction
from profile_actions import StartProfileAction, StopProfileAction


class SocialActionExecutor(Node):
    """
    Orchestrateur principal des actions comportementales.

    Responsabilités :
    - Recevoir les intents via /qbo_social/intent
    - Router vers le handler approprié
    - Exécuter l'action
    - Publier ACTION_DONE ou ACTION_FAILED
    """

    def __init__(self):
        super().__init__('qbo_social_action_executor')

        # =============================
        # 1. INITIALISER LES CLIENTS
        # =============================
        self.get_logger().info("Initializing clients...")

        self.follower_client = FollowerClient(self)
        self.bringup_client = BringupClient(self)
        self.display_client = DisplayClient(self)
        self.tts_client = TTSClient(self)

        # =============================
        # 2. ENREGISTRER LES HANDLERS
        # =============================
        self.handlers = {}
        self._register_handlers()

        # =============================
        # 3. CRÉER SUBSCRIBER/PUBLISHER
        # =============================
        self.sub_intent = self.create_subscription(
            BehaviorIntent,
            '/qbo_social/intent',
            self._on_intent,
            10
        )

        self.pub_event = self.create_publisher(
            SocialEvent,
            '/qbo_social/event',
            10
        )

        self.get_logger().info(
            f"SocialActionExecutor started with {len(self.handlers)} handlers"
        )

    # =============================
    # ENREGISTREMENT DES HANDLERS
    # =============================
    def _register_handlers(self):
        """
        Enregistre tous les handlers d'actions.
        Format : { "INTENT_TYPE": HandlerInstance }
        """

        # Tracking actions
        tracking_handlers = [
            TrackFaceHeadOnlyAction(self, self.follower_client),
            TrackFaceFullAction(self, self.follower_client),
            StopFaceTrackingAction(self, self.follower_client),
            StartPersonSearchAction(self, self.follower_client),
            StopPersonSearchAction(self, self.follower_client),
        ]

        # Display actions
        display_handlers = [
            SetNoseColorAction(self, self.display_client),
            ShowSmileAction(self, self.display_client),
            DisplayTextAction(self, self.display_client),
            ClearDisplayAction(self, self.display_client),
        ]

        # Speech actions
        speech_handlers = [
            SayShortPhraseAction(self, self.tts_client),
            SpeakTextAction(self, self.tts_client),
            StopSpeakingAction(self, self.tts_client),
        ]

        # Social actions
        social_handlers = [
            GreetPersonAction(self, self.display_client, self.tts_client),
            ExpressRecognitionAction(self, self.display_client, self.tts_client),
            ExpressEmotionAction(self, self.display_client, self.tts_client),
        ]

        # Profile actions
        profile_handlers = [
            StartProfileAction(self, self.bringup_client),
            StopProfileAction(self, self.bringup_client),
        ]

        # Enregistrer tous les handlers
        all_handlers = (
            tracking_handlers +
            display_handlers +
            speech_handlers +
            social_handlers +
            profile_handlers
        )

        for handler in all_handlers:
            for intent_type in handler.intent_types:
                self.handlers[intent_type] = handler
                self.get_logger().info(
                    f"  Registered: {intent_type} → {handler.__class__.__name__}"
                )

    # =============================
    # DISPATCH DES INTENTS
    # =============================
    def _on_intent(self, intent: BehaviorIntent):
        """
        Callback principal : reçoit un intent et le route vers le handler approprié.
        """
        intent_type = intent.intent_type

        self.get_logger().info(f"RECEIVED → {intent_type}")

        # Chercher le handler correspondant
        handler = self.handlers.get(intent_type)

        if not handler:
            self.get_logger().warn(
                f"⚠ No handler registered for: {intent_type}"
            )
            self._publish_action_failed(intent, "No handler found")
            return

        # Exécuter l'action
        try:
            success = handler.execute(intent)

            if success:
                self.get_logger().info(f"✓ DONE → {intent_type}")
                self._publish_action_done(intent)
            else:
                self.get_logger().warn(f"✗ FAILED → {intent_type}")
                self._publish_action_failed(intent, "Handler returned False")

        except Exception as e:
            self.get_logger().error(
                f"✗ EXCEPTION in {intent_type}: {e}"
            )
            self._publish_action_failed(intent, str(e))

    # =============================
    # PUBLICATION D'ÉVÉNEMENTS
    # =============================
    def _publish_action_done(self, intent: BehaviorIntent):
        """Publie un événement ACTION_DONE."""
        msg = SocialEvent()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.stamp = msg.header.stamp
        msg.event_type = "ACTION_DONE"
        msg.source = "action_executor"
        msg.person_id = intent.target_person_id
        msg.person_name = intent.target_person_name
        msg.confidence = 1.0
        msg.payload_json = json.dumps({
            "intent_type": intent.intent_type,
            "reason": intent.reason
        })
        self.pub_event.publish(msg)

    def _publish_action_failed(self, intent: BehaviorIntent, error: str):
        """Publie un événement ACTION_FAILED."""
        msg = SocialEvent()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.stamp = msg.header.stamp
        msg.event_type = "ACTION_FAILED"
        msg.source = "action_executor"
        msg.person_id = intent.target_person_id
        msg.person_name = intent.target_person_name
        msg.confidence = 0.0
        msg.payload_json = json.dumps({
            "intent_type": intent.intent_type,
            "reason": intent.reason,
            "error": error
        })
        self.pub_event.publish(msg)


# =============================
# MAIN
# =============================
def main(args=None):
    rclpy.init(args=args)
    node = SocialActionExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
