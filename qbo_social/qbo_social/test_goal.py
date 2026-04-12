#!/usr/bin/env python3
"""
Script de test pour définir des goals dans le SocialBehaviorEngine.

Usage:
    # Définir l'objectif "chercher et saluer"
    python3 test_goal.py set FIND_AND_GREET

    # Définir avec timeout personnalisé (90 secondes)
    python3 test_goal.py set FIND_AND_GREET 90

    # Annuler l'objectif actuel
    python3 test_goal.py cancel
"""

import rclpy
from rclpy.node import Node
from qbo_msgs.msg import SocialEvent
import json
import sys


class GoalTester(Node):
    def __init__(self):
        super().__init__('goal_tester')

        self.pub_event = self.create_publisher(
            SocialEvent,
            '/qbo_social/events',
            10
        )

        # Attendre que le publisher soit prêt
        self.create_timer(0.5, self.check_ready)
        self.ready = False

    def check_ready(self):
        if self.pub_event.get_subscription_count() > 0:
            self.ready = True
            self.get_logger().info("Publisher ready!")

    def set_goal(self, goal_name: str, timeout: float = 60.0):
        """Publie un event SET_GOAL."""
        msg = SocialEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'goal_tester'
        msg.stamp = msg.header.stamp
        msg.event_type = 'SET_GOAL'
        msg.source = 'test_script'
        msg.payload_json = json.dumps({
            "goal": goal_name,
            "timeout": timeout
        })

        self.pub_event.publish(msg)
        self.get_logger().info(f"✓ Goal défini: {goal_name} (timeout: {timeout}s)")

    def cancel_goal(self):
        """Publie un event CANCEL_GOAL."""
        msg = SocialEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'goal_tester'
        msg.stamp = msg.header.stamp
        msg.event_type = 'CANCEL_GOAL'
        msg.source = 'test_script'
        msg.payload_json = '{}'

        self.pub_event.publish(msg)
        self.get_logger().info("✓ Goal annulé")


def main(args=None):
    if len(sys.argv) < 2:
        print(__doc__)
        return

    rclpy.init(args=args)
    node = GoalTester()

    # Attendre que le publisher soit prêt
    print("Attente de la connexion au topic...")
    while not node.ready:
        rclpy.spin_once(node, timeout_sec=0.1)

    command = sys.argv[1].lower()

    if command == "set":
        if len(sys.argv) < 3:
            print("Usage: python3 test_goal.py set <GOAL_NAME> [timeout]")
            return

        goal_name = sys.argv[2]
        timeout = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0

        node.set_goal(goal_name, timeout)

    elif command == "cancel":
        node.cancel_goal()

    else:
        print(f"Commande inconnue: {command}")
        print(__doc__)

    # Attendre un peu que le message soit publié
    rclpy.spin_once(node, timeout_sec=0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
