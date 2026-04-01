#!/usr/bin/env python3
"""
Exemple de client pour le Social Behavior Engine (SBE)
Démontre comment contrôler le face follower et monitorer son statut
"""

import rclpy
import time
from rclpy.node import Node
from qbo_msgs.srv import SetFollowerStatus
from qbo_msgs.msg import FollowerStatus


class SBEFollowerClient(Node):
    """Client exemple pour contrôler le face follower depuis le SBE"""

    def __init__(self):
        super().__init__('sbe_follower_client')

        # Client du service de contrôle
        self.control_client = self.create_client(
            SetFollowerStatus,
            '/qbo_face_following/set_status'
        )

        # Souscription au statut
        self.status_sub = self.create_subscription(
            FollowerStatus,
            '/qbo_face_following/status',
            self.status_callback,
            10
        )

        self.get_logger().info('SBE Follower Client initialized')
        self.last_status = None

    def status_callback(self, msg):
        """Callback pour monitorer le statut du follower"""
        self.last_status = msg

        # Exemple : logger les événements importants
        state_names = {
            0: 'IDLE',
            1: 'SEARCHING',
            2: 'TRACKING',
            3: 'BLOCKED'
        }

        state = state_names.get(msg.tracking_state, 'UNKNOWN')

        if msg.tracking_state == 2:  # TRACKING
            self.get_logger().info(
                f'👁️  Tracking face at ({msg.face_x:.2f}, {msg.face_y:.2f}, {msg.face_z:.2f}) m, '
                f'distance={msg.face_distance:.2f} m'
            )
        elif msg.tracking_state == 3:  # BLOCKED
            self.get_logger().warn(f'⛔ Follower blocked: {msg.blocking_reason}')

    def set_follower_control(self, enable_head=True, enable_rotation=True):
        """
        Contrôler les capacités du follower

        Args:
            enable_head: Activer les mouvements de tête
            enable_rotation: Activer la rotation de la base
        """
        if not self.control_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Service not available')
            return False

        request = SetFollowerStatus.Request()
        request.enable_head_movement = enable_head
        request.enable_base_rotation = enable_rotation

        self.get_logger().info(
            f'📡 Setting follower: head={enable_head}, rotation={enable_rotation}'
        )

        future = self.control_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ {response.message}')
                return True
            else:
                self.get_logger().warn(f'❌ {response.message}')
                return False
        else:
            self.get_logger().error('Service call failed or timed out')
            return False

    def get_face_position_3d(self):
        """
        Récupérer la position 3D du visage détecté

        Returns:
            tuple: (x, y, z, distance) ou None si pas de visage
        """
        if self.last_status is None:
            return None

        if self.last_status.tracking_state == 2:  # TRACKING
            return (
                self.last_status.face_x,
                self.last_status.face_y,
                self.last_status.face_z,
                self.last_status.face_distance
            )
        return None


def main():
    """Exemple d'utilisation du client SBE"""
    rclpy.init()
    client = SBEFollowerClient()

    # Attendre un peu pour recevoir le premier statut
    rclpy.spin_once(client, timeout_sec=1.0)

    # Exemple 1 : Activer tout
    client.get_logger().info('\n=== Exemple 1: Tout activer ===')
    client.set_follower_control(enable_head=True, enable_rotation=True)

    # Attendre et monitorer
    for i in range(10):
        rclpy.spin_once(client, timeout_sec=0.5)
        pos = client.get_face_position_3d()
        if pos:
            client.get_logger().info(
                f'Face à ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) m, '
                f'distance={pos[3]:.2f} m'
            )

    # ⏱️ Attendre 10 secondes avant l'exemple suivant
    client.get_logger().info('⏱️  Attente de 10 secondes...')
    time.sleep(10)

    # Exemple 2 : Désactiver la base (tête seule)
    client.get_logger().info('\n=== Exemple 2: Tête seule ===')
    client.set_follower_control(enable_head=True, enable_rotation=False)

    for i in range(10):
        rclpy.spin_once(client, timeout_sec=0.5)

    # ⏱️ Attendre 10 secondes avant l'exemple suivant
    client.get_logger().info('⏱️  Attente de 10 secondes...')
    time.sleep(10)

    # Exemple 3 : Tout désactiver
    client.get_logger().info('\n=== Exemple 3: Tout désactiver ===')
    client.set_follower_control(enable_head=False, enable_rotation=False)

    for i in range(5):
        rclpy.spin_once(client, timeout_sec=0.5)

    client.get_logger().info('✅ Exemples terminés')
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
