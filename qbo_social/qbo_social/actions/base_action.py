#!/usr/bin/env python3
"""
Classe abstraite de base pour toutes les actions comportementales.
"""

from abc import ABC, abstractmethod
from qbo_msgs.msg import BehaviorIntent


class BaseAction(ABC):
    """
    Interface commune pour tous les handlers d'actions.

    Chaque action concrète doit :
    - Définir intent_types : liste des types d'intents gérés
    - Implémenter execute(intent) : logique d'exécution
    """

    intent_types = []  # À surcharger dans les classes filles

    def __init__(self, node):
        """
        Args:
            node: Instance du nœud ROS2 parent (pour logger, créer publishers, etc.)
        """
        self.node = node
        self.logger = node.get_logger()

    @abstractmethod
    def execute(self, intent: BehaviorIntent) -> bool:
        """
        Exécute l'action correspondant à l'intent reçu.

        Args:
            intent: Message BehaviorIntent contenant le type et les données

        Returns:
            bool: True si succès, False si échec
        """
        raise NotImplementedError

    def log_info(self, message: str):
        """Helper pour logger avec le nom de la classe."""
        self.logger.info(f"[{self.__class__.__name__}] {message}")

    def log_warn(self, message: str):
        """Helper pour logger un warning."""
        self.logger.warn(f"[{self.__class__.__name__}] {message}")

    def log_error(self, message: str):
        """Helper pour logger une erreur."""
        self.logger.error(f"[{self.__class__.__name__}] {message}")
