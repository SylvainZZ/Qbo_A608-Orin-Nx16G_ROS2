from qbo_msgs.msg import ListenResult

"""
ListenAdapter : Écoute les résultats de reconnaissance vocale depuis /listen
et publie des événements SPEECH_RECOGNIZED via /qbo_social/events.
"""


class ListenAdapter:

    def __init__(self, node, min_confidence: float = 0.5):
        """
        Initialise le ListenAdapter.

        Args:
            node: Le nœud ROS parent (SocialEventAdapter)
            min_confidence: Confiance minimale pour émettre un événement (0.0 à 1.0)
        """
        self.node = node
        self.min_confidence = max(0.0, min(1.0, min_confidence))

        self.node.get_logger().info(
            f"ListenAdapter : min_confidence={self.min_confidence:.2f}"
        )

        # Subscription au topic /listen (publié par qbo_listen / listen_whisper.py)
        self.node.create_subscription(
            ListenResult,
            '/listen',
            self._on_listen_result,
            10
        )

    def _on_listen_result(self, msg: ListenResult):
        """
        Callback appelé quand un résultat de reconnaissance vocale arrive.
        Filtre selon la confidence et publie un événement SPEECH_RECOGNIZED.
        """
        text = msg.sentence.strip()
        confidence = msg.confidence

        # Log de réception (debug)
        self.node.get_logger().debug(
            f"ListenResult reçu : '{text}' (confidence={confidence:.2f})"
        )

        # Filtrage par confidence
        if confidence < self.min_confidence:
            self.node.get_logger().debug(
                f"Confidence trop faible ({confidence:.2f} < {self.min_confidence:.2f}), ignoré"
            )
            return

        # Ignorer les résultats vides
        if not text:
            self.node.get_logger().debug("Texte vide, ignoré")
            return

        # Publier l'événement SPEECH_RECOGNIZED
        self._publish_speech_event(text, confidence)

    def _publish_speech_event(self, text: str, confidence: float):
        """
        Publie un événement SPEECH_RECOGNIZED.

        Args:
            text: Le texte reconnu
            confidence: La confiance de reconnaissance (0.0 à 1.0)
        """
        msg = self.node._create_event_msg(
            event_type="SPEECH_RECOGNIZED",
            source="listen",
            payload={
                "sentence": text,  # "sentence" pour correspondre au format attendu par behavior_engine
                "confidence": confidence,
                "language": self.node.get_parameter("system_lang").value if self.node.has_parameter("system_lang") else "fr"
            }
        )

        # Ajouter la confidence au message
        msg.confidence = confidence

        # Publier l'événement
        self.node.pub_event.publish(msg)

        self.node.get_logger().info(
            f"EVENT → SPEECH_RECOGNIZED : '{text}' (confidence={confidence:.2f})"
        )
