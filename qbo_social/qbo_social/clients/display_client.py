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

        # Timers pour les animations temporaires
        self.mouth_timer = None
        self.nose_timer = None

    def set_nose_color(self, color: int, duration=2.0):
        """
        Change la couleur du nez LED.
        S'éteint automatiquement après la durée spécifiée.

        Args:
            color: Code couleur (0=off, 1=rouge, 2=bleu, 3=violet, 4=vert,
                   5=jaune, 6=cyan, 7=blanc)
            duration: Durée d'affichage en secondes (0 = permanent)
        """
        msg = Nose()
        msg.color = color
        self.pub_nose.publish(msg)
        self.logger.info(f"👃 Nose LED: color {color} (duration={duration}s)")

        # Programmer l'extinction si durée > 0 et couleur non nulle
        if duration > 0 and color != 0:
            self._schedule_nose_clear(duration)

    def show_smile(self, variant='normal', duration=2.0):
        """
        Affiche un sourire sur la matrice LED de la bouche (20 LEDs).
        S'éteint automatiquement après la durée spécifiée.

        Args:
            variant: Type de sourire ('normal', 'big', 'wink')
            duration: Durée d'affichage en secondes (0 = permanent)

        Patterns disponibles (4 lignes x 5 colonnes):
          0  1  2  3  4
        0 .  .  .  .  .
        1 .  .  .  .  .
        2 X  .  .  .  X    (coins relevés)
        3 .  X  X  X  .    (sourire)
        """
        patterns = {
            # Sourires
            'normal': [
                False, False, False, False, False,  # Ligne 0: vide
                False, False, False, False, False,  # Ligne 1: vide
                True,  False, False, False, True,   # Ligne 2: coins relevés
                False, True,  True,  True,  False   # Ligne 3: sourire
            ],
            'big': [
                False, False, False, False, False,  # Ligne 0: vide
                True,  False, False, False, True,   # Ligne 1: coins très relevés
                True,  False, False, False, True,   # Ligne 2: coins relevés
                False, True,  True,  True,  False   # Ligne 3: sourire
            ],
            'happy': [
                True,  False, False, False, True,   # Ligne 0: coins très hauts
                True,  True,  False, True,  True,   # Ligne 1: sourire large
                True,  False, False, False, True,   # Ligne 2: coins relevés
                False, True,  True,  True,  False   # Ligne 3: sourire bas
            ],
            'wink': [
                False, False, False, False, False,  # Ligne 0: vide
                False, False, False, False, False,  # Ligne 1: vide
                True,  False, False, False, False,  # Ligne 2: coin gauche seulement
                False, True,  True,  True,  False   # Ligne 3: sourire
            ],

            # Émotions négatives
            'sad': [
                False, False, False, False, False,  # Ligne 0: vide
                False, False, False, False, False,  # Ligne 1: vide
                False, True,  True,  True,  False,  # Ligne 2: bouche retournée haut
                True,  False, False, False, True    # Ligne 3: coins bas
            ],
            'angry': [
                False, False, False, False, False,  # Ligne 0: vide
                True,  True,  False, True,  True,   # Ligne 1: ligne agressive
                False, False, False, False, False,  # Ligne 2: vide
                True,  True,  True,  True,  True    # Ligne 3: ligne serrée
            ],
            'worried': [
                False, False, False, False, False,  # Ligne 0: vide
                False, False, False, False, False,  # Ligne 1: vide
                False, True,  False, True,  False,  # Ligne 2: ondulation inquiète
                True,  False, True,  False, True    # Ligne 3: ondulation
            ],

            # États particuliers
            'surprised': [
                False, True,  True,  True,  False,  # Ligne 0: bouche ouverte haut
                True,  False, False, False, True,   # Ligne 1: côtés
                True,  False, False, False, True,   # Ligne 2: côtés
                False, True,  True,  True,  False   # Ligne 3: bouche ouverte bas
            ],
            'neutral': [
                False, False, False, False, False,  # Ligne 0: vide
                False, False, False, False, False,  # Ligne 1: vide
                False, False, False, False, False,  # Ligne 2: vide
                False, True,  True,  True,  False   # Ligne 3: ligne neutre
            ],
            'thinking': [
                False, False, False, False, False,  # Ligne 0: vide
                False, False, False, False, False,  # Ligne 1: vide
                False, False, True,  False, False,  # Ligne 2: point central
                False, True,  False, True,  False   # Ligne 3: réflexion
            ],
            'sleepy': [
                False, False, False, False, False,  # Ligne 0: vide
                False, False, False, False, False,  # Ligne 1: vide
                False, False, False, False, False,  # Ligne 2: vide
                True,  False, False, False, True    # Ligne 3: petite bouche fatiguée
            ],
            'excited': [
                True,  False, True,  False, True,   # Ligne 0: excitation
                False, True,  True,  True,  False,  # Ligne 1: bouche ouverte
                True,  False, False, False, True,   # Ligne 2: coins relevés
                False, True,  True,  True,  False   # Ligne 3: sourire
            ]
        }

        smile_pattern = patterns.get(variant, patterns['normal'])

        msg = Mouth()
        msg.mouth_image = smile_pattern
        self.pub_mouth.publish(msg)
        self.logger.info(f"😊 Mouth LED: {variant} smile (duration={duration}s)")

        # Programmer l'extinction si durée > 0
        if duration > 0:
            self._schedule_mouth_clear(duration)

    def set_mouth_pattern(self, pattern: list, duration=2.0):
        """
        Définit un pattern personnalisé sur la matrice LED de la bouche.
        S'éteint automatiquement après la durée spécifiée.

        Args:
            pattern: Liste de 20 booléens (True = LED allumée, False = éteinte)
            duration: Durée d'affichage en secondes (0 = permanent)
        """
        if len(pattern) != 20:
            self.logger.error(f"Mouth pattern must have 20 elements, got {len(pattern)}")
            return

        msg = Mouth()
        msg.mouth_image = pattern
        self.pub_mouth.publish(msg)
        self.logger.info(f"😮 Mouth LED: custom pattern (duration={duration}s)")

        # Programmer l'extinction si durée > 0
        if duration > 0:
            self._schedule_mouth_clear(duration)

    def clear_nose(self):
        """Éteint le nez LED."""
        # Annuler le timer si actif
        if self.nose_timer is not None:
            self.nose_timer.cancel()
            self.nose_timer = None

        msg = Nose()
        msg.color = 0  # OFF
        self.pub_nose.publish(msg)
        self.logger.info("👃 Nose LED: cleared")

    def clear_mouth(self):
        """Éteint toutes les LEDs de la bouche."""
        # Annuler le timer si actif
        if self.mouth_timer is not None:
            self.mouth_timer.cancel()
            self.mouth_timer = None

        msg = Mouth()
        msg.mouth_image = [False] * 20
        self.pub_mouth.publish(msg)
        self.logger.info("😶 Mouth LED: cleared")

    def _schedule_nose_clear(self, duration: float):
        """
        Programme l'extinction du nez après une durée.

        Args:
            duration: Délai en secondes avant extinction
        """
        # Annuler le timer précédent s'il existe
        if self.nose_timer is not None:
            self.nose_timer.cancel()

        # Créer un nouveau timer
        self.nose_timer = self.node.create_timer(
            duration,
            self._on_nose_timer_expired
        )

    def _on_nose_timer_expired(self):
        """Callback appelé quand le timer du nez expire."""
        self.clear_nose()
        self.logger.info("⏱ Nose LED: auto-cleared after timeout")

    def _schedule_mouth_clear(self, duration: float):
        """
        Programme l'extinction de la bouche après une durée.

        Args:
            duration: Délai en secondes avant extinction
        """
        # Annuler le timer précédent s'il existe
        if self.mouth_timer is not None:
            self.mouth_timer.cancel()

        # Créer un nouveau timer
        self.mouth_timer = self.node.create_timer(
            duration,
            self._on_mouth_timer_expired
        )

    def _on_mouth_timer_expired(self):
        """Callback appelé quand le timer de la bouche expire."""
        self.clear_mouth()
        self.logger.info("⏱ Mouth LED: auto-cleared after timeout")

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
