#!/usr/bin/env python3

import rclpy
import os, json, re

from rclpy.node import Node
from qbo_msgs.msg import ListenResult
from qbo_msgs.srv import Text2Speach
from rivescript import RiveScript

class BrainRS(Node):
    def __init__(self):
        super().__init__('brain_rs')
        self.get_logger().info("Démarrage de brainRS...")

        # Chargement du bot RiveScript
        self.bot = RiveScript(utf8=True)
        self.bot.unicode_punctuation = re.compile(r'[.,!?;:]')
        self.bot.load_directory("/home/qbo-v2/qbo_ws/src/qbo_driver/config/RS")
        self.bot.sort_replies()

        # Initialisation du client ROS2 pour le service de parole
        self.client = self.create_client(Text2Speach, '/qbo_driver/piper2wave_say')

        # S'abonner au topic /listen
        self.subscription = self.create_subscription(
            ListenResult,
            '/listen',
            self.listen_callback,
            10
        )

    def normalize_input(self, sentence: str) -> str:
        sentence = sentence.lower()
        sentence = re.sub(r"[’']", " ", sentence)             # remplace apostrophes par espace
        sentence = re.sub(r"[^\w\s]", "", sentence)           # supprime ponctuation (.,!? etc.)
        # sentence = re.sub(r"\s+", " ", sentence).strip()      # supprime les espaces multiples
        return sentence

    def listen_callback(self, msg: ListenResult):
        sentence = msg.sentence.lower().strip()
        confidence = msg.confidence

        self.get_logger().info(f"Reçu: '{sentence}' avec confidence {confidence:.2f}")
        sentence_cleaned = self.normalize_input(sentence)
        self.get_logger().info(f"Phrase nettoyée: '{sentence_cleaned}'")

        if confidence < 0.5:
            self.say("Je ne suis pas sûr d'avoir compris. Peux-tu répéter ?")
            return

        reply = self.bot.reply("localuser", sentence_cleaned)

        self.get_logger().info(f"Retour du reply: '{reply}'")

        if reply == "[ERR: No Reply Matched]":
            self.say("Je ne suis pas sûr d'avoir compris. Peux-tu répéter ?")
        elif confidence >= 0.6:
            self.say(reply)
        else:
            self.say("Tu voulais dire : " + reply + " ?")

    def say(self, text):
        if not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service /qbo_driver/piper2wave_say non disponible")
            return

        request = Text2Speach.Request()
        request.sentence = text

        future = self.client.call_async(request)
        future.add_done_callback(self.say_done)

    def say_done(self, future):
        try:
            result = future.result()
            self.get_logger().info("Parole envoyée avec succès.")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'envoi de la parole: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BrainRS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("⏹️ Arrêt demandé → sortie propre.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
