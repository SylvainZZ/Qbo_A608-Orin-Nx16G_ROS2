#!/usr/bin/env python3
"""
Script de debug unifié pour le Social Behavior Engine de QBO.
Affiche tous les flux: Events, World State, Intents et Decision Traces.

Usage:
    ros2 run qbo_social debug_behavior_state

Options:
    --verbose : Affiche aussi les WorldState (peut être verbeux)
"""

import rclpy
from rclpy.node import Node
from qbo_msgs.msg import WorldState, SocialEvent, BehaviorIntent, DecisionTrace
import sys


class BehaviorDebugMonitor(Node):
    def __init__(self, show_world_updates=False):
        super().__init__('behavior_debug_monitor')

        self.show_world_updates = show_world_updates
        self.event_count = 0
        self.intent_count = 0
        self.trace_count = 0

        self.create_subscription(WorldState, '/qbo_social/world_state', self._on_world, 10)
        self.create_subscription(SocialEvent, '/qbo_social/events', self._on_event, 10)
        self.create_subscription(BehaviorIntent, '/qbo_social/intent', self._on_intent, 10)
        self.create_subscription(DecisionTrace, '/qbo_social/decision_trace', self._on_trace, 10)

        self._print_header()

    def _print_header(self):
        print("\n" + "=" * 100)
        print(" " * 30 + "🤖 QBO SOCIAL BEHAVIOR ENGINE - DEBUG MONITOR 🤖")
        print("=" * 100)
        print(f"{'Légende:':<20} 📥 Event  |  🌍 World  |  💭 Decision  |  🎯 Intent")
        print(f"{'Option:':<20} --verbose pour afficher les mises à jour WorldState")
        print("=" * 100 + "\n")

    def _on_world(self, msg):
        """Affiche les mises à jour du WorldState (optionnel, car peut être verbeux)."""
        if not self.show_world_updates:
            return

        print(f"🌍 WORLD │ mode={msg.mode:<15} │ face={msg.face_present}/{msg.face_stable:<5} │ "
              f"focus={msg.focus_person_name or 'none':<10} │ engagement={msg.engagement_level:.2f}")

    def _on_event(self, msg):
        """Affiche les événements sociaux entrants."""
        self.event_count += 1

        # Extraction des infos du payload pour les événements face
        extra_info = ""
        if "FACE" in msg.event_type and msg.payload_json:
            import json
            try:
                payload = json.loads(msg.payload_json)
                if "distance" in payload:
                    extra_info = f" │ dist={payload['distance']:.2f}m"
                if msg.person_name:
                    extra_info += f" │ person={msg.person_name}"
            except:
                pass
        elif msg.event_type == "PERSON_RECOGNIZED":
            extra_info = f" │ {msg.person_name} (conf={msg.confidence:.2f})"

        print(f"📥 EVENT #{self.event_count:<3} │ {msg.event_type:<25} │ from {msg.source:<20}{extra_info}")

    def _on_intent(self, msg):
        """Affiche les intentions émises vers l'action executor."""
        self.intent_count += 1

        person_info = f"→ {msg.target_person_name}" if msg.target_person_name else ""

        print(f"🎯 INTENT #{self.intent_count:<3} │ {msg.intent_type:<25} │ reason={msg.reason:<30} {person_info}")

    def _on_trace(self, msg):
        """Affiche les traces de décision (pourquoi une décision a été prise)."""
        self.trace_count += 1

        # Barre de pertinence visuelle
        relevance_bar = "█" * int(msg.relevance_score * 10)
        relevance_display = f"[{relevance_bar:<10}] {msg.relevance_score:.2f}"

        # Icônes de statut
        status = "✓" if not msg.suppressed_by_cooldown else "🔒 COOLDOWN"
        speech = "🔊" if msg.speech_allowed else "🔇"

        print(f"\n{'─' * 100}")
        print(f"💭 DECISION TRACE #{self.trace_count}")
        print(f"   Trigger   : {msg.triggering_event}")
        print(f"   Intent    : {msg.chosen_intent or '(none)'}")
        print(f"   Reason    : {msg.reason}")
        print(f"   Relevance : {relevance_display}")
        print(f"   Status    : {status}  {speech}")
        print(f"{'─' * 100}\n")


def main(args=None):
    rclpy.init(args=args)

    # Vérifier si --verbose est passé en argument
    show_world = '--verbose' in sys.argv

    node = BehaviorDebugMonitor(show_world_updates=show_world)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n" + "=" * 100)
        print(f"📊 STATISTIQUES FINALES:")
        print(f"   - Events reçus       : {node.event_count}")
        print(f"   - Intents émis       : {node.intent_count}")
        print(f"   - Décisions tracées  : {node.trace_count}")
        print("=" * 100)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
