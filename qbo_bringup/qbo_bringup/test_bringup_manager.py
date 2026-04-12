#!/usr/bin/env python3
"""
Script de test automatisé pour le QBO Bringup Manager.

Teste les fonctionnalités principales :
  - Service availability
  - START/STOP/RESTART/STATUS pour chaque profil
  - Gestion d'erreurs
  - Intégration avec action_executor

Usage:
    python3 test_bringup_manager.py
"""

import time
import rclpy
from rclpy.node import Node
from qbo_msgs.srv import ManageProfile


class BringupManagerTester(Node):
    """Node de test pour le Bringup Manager."""

    def __init__(self):
        super().__init__('bringup_manager_tester')

        self.client = self.create_client(
            ManageProfile,
            '/qbo_bringup/manage_profile'
        )

        self.get_logger().info("Attente du service /qbo_bringup/manage_profile...")
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service non disponible après 10s !")
            raise RuntimeError("Service /qbo_bringup/manage_profile non démarré")

        self.get_logger().info("✓ Service disponible")

    def call_service(self, profile: str, action: str, expected_success: bool = True):
        """Appelle le service et vérifie le résultat."""
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"TEST: {action} profile {profile}")
        self.get_logger().info(f"{'='*60}")

        request = ManageProfile.Request()
        request.profile_name = profile
        request.action = action
        request.target_nodes = []

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done():
            response = future.result()

            # Afficher la réponse
            self.get_logger().info(f"  Success: {response.success}")
            self.get_logger().info(f"  Message: {response.message}")
            self.get_logger().info(f"  Active nodes: {response.active_nodes}")
            self.get_logger().info(f"  Profile state: {self._state_name(response.profile_state)}")

            # Vérifier le résultat attendu
            if response.success == expected_success:
                self.get_logger().info(f"✓ Test PASSED")
                return True
            else:
                self.get_logger().error(f"✗ Test FAILED (expected success={expected_success})")
                return False
        else:
            self.get_logger().error("✗ Service call timeout")
            return False

    def _state_name(self, state: int) -> str:
        """Convertit un code d'état en nom."""
        states = {0: "STOPPED", 1: "STARTING", 2: "RUNNING", 3: "STOPPING", 4: "ERROR"}
        return states.get(state, "UNKNOWN")

    def run_tests(self):
        """Exécute la suite de tests complète."""
        results = []

        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("DÉBUT DES TESTS")
        self.get_logger().info("="*70)

        # Test 1: STATUS d'un profil non démarré
        results.append(self.call_service("MINIMAL", "STATUS", expected_success=True))
        time.sleep(1)

        # Test 2: START profil MINIMAL
        results.append(self.call_service("MINIMAL", "START", expected_success=True))
        time.sleep(3)

        # Test 3: STATUS du profil actif
        results.append(self.call_service("MINIMAL", "STATUS", expected_success=True))
        time.sleep(1)

        # Test 4: START d'un profil déjà actif (devrait échouer)
        results.append(self.call_service("MINIMAL", "START", expected_success=False))
        time.sleep(1)

        # Test 5: STOP profil MINIMAL
        results.append(self.call_service("MINIMAL", "STOP", expected_success=True))
        time.sleep(2)

        # Test 6: STOP d'un profil déjà arrêté (devrait échouer)
        results.append(self.call_service("MINIMAL", "STOP", expected_success=False))
        time.sleep(1)

        # Test 7: RESTART d'un profil non démarré
        results.append(self.call_service("VISION", "RESTART", expected_success=True))
        time.sleep(3)

        # Test 8: STATUS du profil VISION
        results.append(self.call_service("VISION", "STATUS", expected_success=True))
        time.sleep(1)

        # Test 9: STOP VISION
        results.append(self.call_service("VISION", "STOP", expected_success=True))
        time.sleep(2)

        # Test 10: Profil invalide (devrait échouer)
        results.append(self.call_service("INVALID_PROFILE", "START", expected_success=False))
        time.sleep(1)

        # Test 11: Action invalide (devrait échouer)
        results.append(self.call_service("MINIMAL", "INVALID_ACTION", expected_success=False))
        time.sleep(1)

        # Résumé
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("RÉSUMÉ DES TESTS")
        self.get_logger().info("="*70)
        passed = sum(results)
        total = len(results)
        self.get_logger().info(f"Tests réussis : {passed}/{total}")

        if passed == total:
            self.get_logger().info("✓ TOUS LES TESTS ONT RÉUSSI !")
            return True
        else:
            self.get_logger().error(f"✗ {total - passed} test(s) ont échoué")
            return False


def main(args=None):
    rclpy.init(args=args)

    try:
        tester = BringupManagerTester()
        success = tester.run_tests()

        tester.destroy_node()
        rclpy.shutdown()

        exit(0 if success else 1)

    except Exception as e:
        print(f"Erreur pendant les tests : {e}")
        exit(1)


if __name__ == '__main__':
    main()
