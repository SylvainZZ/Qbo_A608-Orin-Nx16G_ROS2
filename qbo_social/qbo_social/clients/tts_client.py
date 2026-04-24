#!/usr/bin/env python3
"""
Client encapsulant le service de synthèse vocale (TTS).
Utilise le service Pico TTS: /qbo_driver/say_to_TTS
"""

from qbo_msgs.srv import Text2Speach


class TTSClient:
    """Encapsule les appels au service TTS Pico."""

    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()

        # Client du service Pico TTS
        self.service = node.create_client(
            Text2Speach,
            '/qbo_driver/say_to_TTS'
        )

        # Attendre que le service soit disponible (non-bloquant)
        if not self.service.wait_for_service(timeout_sec=3.0):
            self.logger.warn("Service TTS '/qbo_driver/say_to_TTS' non disponible")
        else:
            self.logger.info("TTS client connecté au service Pico")

    def speak(self, text: str, language: str = "fr", callback=None):
        """
        Fait parler le robot via Pico TTS.

        Args:
            text: Texte à prononcer
            language: Langue (ex: "fr-FR", "en-US", "es-ES") - Géré dynamiquement
            callback: Fonction appelée après la synthèse (optionnel)

        Returns:
            bool: True si la requête a été envoyée, False sinon
        """
        if not self.service.service_is_ready():
            self.logger.warn(f"Service TTS non prêt, impossible de dire: '{text}'")
            if callback:
                callback(False)
            return False

        # Créer la requête
        request = Text2Speach.Request()
        request.sentence = text

        self.logger.info(f"🗣️ TTS: '{text}' (langue: {language})")

        # Appel asynchrone
        future = self.service.call_async(request)

        # Si un callback est fourni, l'attacher
        if callback:
            future.add_done_callback(lambda f: self._handle_response(f, callback))

        return True

    def _handle_response(self, future, callback):
        """Gère la réponse du service TTS."""
        try:
            response = future.result()
            success = response.success
            if success:
                self.logger.info("✅ TTS: Synthèse réussie")
            else:
                self.logger.warn("⚠️ TTS: Échec de la synthèse")
            callback(success)
        except Exception as e:
            self.logger.error(f"❌ TTS: Erreur lors de l'appel du service: {e}")
            callback(False)

    def stop_speaking(self):
        """
        Interrompt la synthèse vocale en cours.
        Note: Pico TTS utilise une queue bloquante, l'interruption n'est pas supportée.
        Cette méthode est un placeholder pour compatibilité.
        """
        self.logger.warn("⚠️ Stop speaking: non supporté par Pico TTS (queue bloquante)")
        return False
