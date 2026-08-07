#!/usr/bin/env python3
"""
AimlClient — Client du service /aiml/generate_text.

Suivant la même architecture que BringupClient / TTSClient / DisplayClient.
Fournit une API bloquante (à appeler depuis un thread, pas depuis un callback ROS2).
"""

import json
import time

from qbo_msgs.srv import GenerateText
from rclpy.callback_groups import ReentrantCallbackGroup


class AimlClient:

    SERVICE_TIMEOUT = 8.0   # secondes

    def __init__(self, node):
        self._node = node

        # ReentrantCallbackGroup pour autoriser l'appel depuis un thread
        self._cb_group = ReentrantCallbackGroup()
        self._client = node.create_client(
            GenerateText,
            '/aiml/generate_text',
            callback_group=self._cb_group
        )

        node.get_logger().info("AimlClient initialized (/aiml/generate_text)")

    def is_available(self) -> bool:
        """Vérifie si le service est disponible (non bloquant)."""
        return self._client.service_is_ready()

    def ask_conversation(self, text: str, params: dict = None) -> tuple[bool, str]:
        """
        Envoie une phrase au moteur de conversation AIML.

        Args:
            text      : phrase de l'utilisateur
            params    : paramètres contextuels optionnels (couleur, nombre, …)

        Returns:
            (success, response_text)
            response_text : texte à prononcer
        """
        context_json = json.dumps({"params": params or {}})
        return self._call(request_type="conversation", text=text, context_json=context_json)

    def ask_diagnostic(self, key: str, severity: str, message: str) -> tuple[bool, str]:
        """
        Envoie un événement diagnostic au moteur AIML.

        Returns:
            (success, response_text)
        """
        context_json = json.dumps({
            "key": key,
            "severity": severity,
            "message": message,
        })
        return self._call(request_type="diagnostic", text="", context_json=context_json)

    # ─────────────────────────────────────────────────────────────────────────
    # Méthode interne
    # ─────────────────────────────────────────────────────────────────────────

    def _call(self, request_type: str, text: str, context_json: str) -> tuple[bool, str]:
        """
        Appel bloquant du service GenerateText.
        ⚠  À appeler depuis un thread dédié, jamais depuis un callback ROS2.

        Returns:
            (success, response_text)
        """
        # Attendre que le service soit disponible
        if not self._client.wait_for_service(timeout_sec=self.SERVICE_TIMEOUT):
            self._node.get_logger().error(
                "[AimlClient] Service /aiml/generate_text indisponible"
            )
            return False, ""

        req = GenerateText.Request()
        req.type = request_type
        req.text = text
        req.context_json = context_json

        future = self._client.call_async(req)

        # Polling (compatible single-threaded executor)
        deadline = time.monotonic() + self.SERVICE_TIMEOUT
        while not future.done():
            if time.monotonic() > deadline:
                self._node.get_logger().error("[AimlClient] Timeout réponse AIML")
                return False, ""
            time.sleep(0.05)

        result = future.result()
        if result is None:
            return False, ""

        return (
            bool(result.success),
            result.response_text or "",
        )


# =============================================================================
# QueryAIMLAction — Handler de l'intent QUERY_AIML
# =============================================================================

import threading


class QueryAIMLAction:

    intent_types = ["QUERY_AIML"]

    def __init__(self, node, aiml_client: AimlClient, tts_client):
        self._node = node
        self._aiml = aiml_client
        self._tts = tts_client
        self._busy = threading.Lock()

    def execute(self, intent) -> bool:
        """
        Lance le traitement AIML dans un thread dédié.
        Retourne True immédiatement (non bloquant).
        """
        import json as _json
        try:
            payload = _json.loads(intent.payload_json or "{}")
        except Exception:
            payload = {}

        sentence = payload.get("sentence", "")
        person_name = intent.target_person_name or payload.get("person_name", "")

        if not sentence:
            self._node.get_logger().warn("[QueryAIMLAction] Phrase vide, ignorée")
            return False

        thread = threading.Thread(
            target=self._process,
            args=(sentence, person_name),
            daemon=True
        )
        thread.start()
        return True

    def _process(self, sentence: str, person_name: str):
        """Appel bloquant AIML + TTS (dans un thread dédié)."""
        if not self._busy.acquire(blocking=False):
            self._node.get_logger().warn(
                "[QueryAIMLAction] Déjà en cours, intent ignoré"
            )
            return

        try:
            self._node.get_logger().info(
                f"[QueryAIMLAction] → AIML: '{sentence}'"
            )
            success, response_text = self._aiml.ask_conversation(sentence)

            if not success or not response_text:
                self._node.get_logger().warn(
                    "[QueryAIMLAction] Pas de réponse AIML"
                )
                return

            self._node.get_logger().info(
                f"[QueryAIMLAction] ← AIML: '{response_text}'"
            )
            self._tts.speak(response_text)

        finally:
            self._busy.release()
