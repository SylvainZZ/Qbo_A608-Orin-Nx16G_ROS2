import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from qbo_msgs.msg import ListenResult
import csv
import time
from datetime import datetime

TEST_INPUT_FILE = "tests/questions.txt"
LOG_OUTPUT_FILE = f"tests/test_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"


class TestDriver(Node):
    def __init__(self):
        super().__init__("qa_batch_tester")
        self.pub = self.create_publisher(ListenResult, "/listen_result", 10)
        self.get_logger().info("📤 Testeur prêt. Envoi des questions...")

        time.sleep(1)  # Laisse le noeud se stabiliser

        with open(TEST_INPUT_FILE, "r", encoding="utf-8") as f:
            questions = [q.strip() for q in f if q.strip()]

        with open(LOG_OUTPUT_FILE, "w", newline="", encoding="utf-8") as log_file:
            writer = csv.writer(log_file)
            writer.writerow(["Question", "Réponse (log)", "ScoreConfiance", "Intent", "Params détectés"])

            for q in questions:
                msg = ListenResult()
                msg.sentence = q
                msg.confidence = 0.95  # Forçage haute confiance pour éviter les effets VAD
                self.pub.publish(msg)
                self.get_logger().info(f"➡️ Question envoyée : {q}")
                rclpy.spin_once(self, timeout_sec=1.5)  # attend traitement
                # les logs seront lus depuis terminal ou via CSV à la main
                writer.writerow([q, "-", "-", "-", "-"])  # ligne vide pour compléter plus tard

        self.get_logger().info(f"📄 Fichier CSV créé : {LOG_OUTPUT_FILE}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = TestDriver()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
