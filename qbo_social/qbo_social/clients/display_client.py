#!/usr/bin/env python3
"""
Client encapsulant les publishers pour l'affichage (LED, LCD, etc.).
"""

from qbo_msgs.msg import Nose, LCD, Mouth


class DisplayClient:
    """Encapsule les publishers pour l'affichage visuel."""

    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()

        # Publisher pour le nez LED
        self.pub_nose = node.create_publisher(
            Nose,
            '/qbo_arduqbo/nose_ctrl/cmd_nose',
            10
        )

        # Publisher pour l'écran LCD
        self.pub_lcd = node.create_publisher(
            LCD,
            '/qbo_arduqbo/lcd_ctrl/cmd_lcd',
            10
        )

        # Publisher pour la matrice LED de la bouche (20 LEDs)
        self.pub_mouth = node.create_publisher(
            Mouth,
            '/qbo_arduqbo/mouth_ctrl/cmd_mouth',
            10
        )

    def set_nose_color(self, color: int):
        """
        Change la couleur du nez LED.

        Args:
            color: Code couleur (0=off, 1=rouge, 2=bleu, 3=violet, 4=vert,
                   5=jaune, 6=cyan, 7=blanc)
        """
        msg = Nose()
        msg.color = color
        self.pub_nose.publish(msg)
        self.logger.info(f"Nose color set to {color}")

    def show_smile(self):
        """
        Affiche un sourire sur la matrice LED de la bouche (20 LEDs).
        
        Pattern sourire classique :
        Layout approximatif (4 lignes x 5 colonnes):
          0  1  2  3  4
        0 .  .  .  .  .
        1 .  .  .  .  .
        2 X  .  .  .  X    (coins relevés)
        3 .  X  X  X  .    (sourire)
        """
        # Pattern sourire : LEDs allumées aux positions [0, 4, 16, 17, 18]
        # Ajustez selon la disposition réelle de votre matrice
        smile_pattern = [
            True,  False, False, False, True,   # Ligne 0: coins
            False, False, False, False, False,  # Ligne 1: vide
            False, False, False, False, False,  # Ligne 2: vide
            False, True,  True,  True,  False   # Ligne 3: sourire
        ]
        
        msg = Mouth()
        msg.mouth_image = smile_pattern
        self.pub_mouth.publish(msg)
        self.logger.info("😊 Mouth LED: smile pattern")

    def set_mouth_pattern(self, pattern: list):
        """
        Définit un pattern personnalisé sur la matrice LED de la bouche.
        
        Args:
            pattern: Liste de 20 booléens (True = LED allumée, False = éteinte)
        """
        if len(pattern) != 20:
            self.logger.error(f"Mouth pattern must have 20 elements, got {len(pattern)}")
            return
        
        msg = Mouth()
        msg.mouth_image = pattern
        self.pub_mouth.publish(msg)
        self.logger.info(f"😮 Mouth LED: custom pattern")

    def clear_mouth(self):
        """Éteint toutes les LEDs de la bouche."""
        msg = Mouth()
        msg.mouth_image = [False] * 20
        self.pub_mouth.publish(msg)
        self.logger.info("Mouth LED: cleared")

    def display_text(self, line1: str, line2: str = ""):
        """
        Affiche du texte sur l'écran LCD.

        Note: Le LCD est limité à 20 caractères. Si line2 est fourni,
        les 2 lignes sont concaténées avec line1 tronquée à 10 chars.

        Args:
            line1: Première ligne (ou texte unique si line2 vide)
            line2: Deuxième ligne (optionnel)
        """
        if line2:
            # Mode 2 lignes : 10 caractères par ligne
            text = f"{line1[:10]:<10}{line2[:10]:<10}"
        else:
            # Mode 1 ligne : 20 caractères
            text = f"{line1[:20]:<20}"

        msg = LCD()
        msg.msg = text
        self.pub_lcd.publish(msg)
        self.logger.info(f"📟 LCD: '{text}'")

    def clear_display(self):
        """Efface l'affichage LCD (20 espaces)."""
        msg = LCD()
        msg.msg = " " * 20
        self.pub_lcd.publish(msg)
        self.logger.info("📟 LCD: cleared")
