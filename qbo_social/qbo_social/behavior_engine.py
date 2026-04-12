#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json

from qbo_msgs.msg import SocialEvent, WorldState, BehaviorIntent, DecisionTrace

# ===== GOAL REQUIREMENTS (sémantique) =====
# Déclaration des prérequis de chaque goal
GOAL_REQUIREMENTS = {
    "FIND_AND_GREET": {
        "services": ["face_follower"],  # Services ROS2 requis
        "description": "Chercher et saluer une personne (nécessite vision)"
    },
    # Futurs goals:
    # "PATROL": {
    #     "services": ["navigate_to_pose"],
    #     "description": "Patrouiller entre des points"
    # },
}


class SocialBehaviorEngine(Node):

    def __init__(self):
        super().__init__('qbo_social_behavior_engine')

        # ===== ARCHITECTURE ÉVÉNEMENTIELLE (Option A) =====
        # 1. /qbo_social/events → Traitement DIRECT pour réactivité maximale
        #    (FACE_APPEARED, FACE_STABLE, FACE_LOST → tracking immédiat)
        # 2. /qbo_social/world_state → Conditions sociales UNIQUEMENT
        #    (mode IDLE/ENGAGED, éviter spam, dashboard monitoring)

        self.sub_event = self.create_subscription(
            SocialEvent,
            '/qbo_social/events',
            self._on_event,
            10
        )

        self.sub_world = self.create_subscription(
            WorldState,
            '/qbo_social/world_state',
            self._on_world,
            10
        )

        self.pub_intent = self.create_publisher(
            BehaviorIntent,
            '/qbo_social/intent_raw',
            10
        )

        self.pub_trace = self.create_publisher(
            DecisionTrace,
            '/qbo_social/decision_trace',
            10
        )

        # ===== Params =====
        self.declare_parameter("greet_cooldown", 10.0)
        self.declare_parameter("stable_head_stop_delay", 5.0)  # Temps avant d'arrêter la tête si visage stable
        self.declare_parameter("search_timeout", 30.0)  # Temps max de recherche après perte
        self.declare_parameter("diag_cooldown", 5.0)

        self.greet_cooldown = self.get_parameter("greet_cooldown").value
        self.stable_head_stop_delay = self.get_parameter("stable_head_stop_delay").value
        self.search_timeout = self.get_parameter("search_timeout").value
        self.diag_cooldown = self.get_parameter("diag_cooldown").value

        # ===== state =====
        self.world = None
        self.last_greet_time = 0
        self.last_diag_time = {}

        # ===== Face tracking state machine =====
        self.tracking_mode = "IDLE"  # IDLE, TRACKING_FULL, TRACKING_HEAD_ONLY, SEARCHING
        self.face_stable_since = None  # Timestamp when face became stable
        self.face_lost_since = None    # Timestamp when face was lost
        self.head_stopped = False      # Flag to avoid re-sending head stop command

        # ===== Goals proactifs (objectifs comportementaux) =====
        self.current_goal = None       # "FIND_AND_GREET", "PATROL", None
        self.goal_state = "IDLE"       # IDLE, WAITING_REQUIREMENTS, SEARCHING, FOUND, ENGAGING, COMPLETED, FAILED
        self.goal_start_time = None    # Timestamp de début de goal
        self.goal_timeout = 60.0       # Timeout par défaut (60s)
        self.greeted_persons = set()   # Personnes déjà saluées pendant ce goal
        self.goal_requested_modules = set()  # Modules demandés par le goal actuel

        # ===== Timers =====
        # Timer pour vérifier périodiquement les conditions de tracking (500ms)
        # DÉSACTIVÉ pour debug - éviter conflits events/world
        # self.tracking_check_timer = self.create_timer(0.5, self._update_tracking_behavior)
        # Timer proactif pour avancer vers les objectifs (2s)
        # DÉSACTIVÉ pour debug - éviter conflits events/world
        # self.goal_check_timer = self.create_timer(2.0, self._update_goal_behavior)

        self.get_logger().info("SocialBehaviorEngine started")
        self.get_logger().info(f"  - stable_head_stop_delay: {self.stable_head_stop_delay}s")
        self.get_logger().info(f"  - search_timeout: {self.search_timeout}s")
        self.get_logger().info("  - Proactive goals system enabled (2s check interval)")

    # =========================
    def _on_world(self, msg):
        self.world = msg
        # Note: _update_tracking_behavior() est appelé par le timer, pas besoin ici

    # =========================
    def _on_event(self, event: SocialEvent):

        payload = {}
        if event.payload_json:
            try:
                payload = json.loads(event.payload_json)
            except:
                pass

        now = self.get_clock().now().seconds_nanoseconds()[0]

        # =========================
        # FACE TRACKING EVENTS (ne nécessitent pas world)
        # =========================
        # COMMENTÉ pour debug - conflits events directs vs world orchestration
        # if event.event_type == "FACE_APPEARED":
        #     self.get_logger().info(f"Face appeared (mode was: {self.tracking_mode}) → Start full tracking")
        #     self.tracking_mode = "TRACKING_FULL"
        #     self.face_stable_since = None
        #     self.face_lost_since = None
        #     self.head_stopped = False
        #     self._publish_intent(
        #         intent_type="TRACK_FACE_FULL",
        #         reason="face_appeared",
        #         triggering_event="FACE_APPEARED",
        #         relevance=1.0
        #     )
        #
        #     # Mise à jour goal si actif
        #     if self.current_goal == "FIND_AND_GREET" and self.goal_state == "SEARCHING":
        #         self.goal_state = "FOUND"
        #         self.get_logger().info("[GOAL] FIND_AND_GREET → Visage détecté")
        #
        # elif event.event_type == "FACE_STABLE":
        #     if self.face_stable_since is None:
        #         self.face_stable_since = now
        #         self.get_logger().info(f"Face stable detected, will stop head in {self.stable_head_stop_delay}s")
        #
        #     # Si on reçoit FACE_STABLE alors qu'on est en IDLE, c'est que FACE_APPEARED
        #     # n'a pas été traité (timing de démarrage). On passe en TRACKING_FULL.
        #     if self.tracking_mode == "IDLE":
        #         self.get_logger().info("FACE_STABLE received while IDLE → Start full tracking")
        #         self.tracking_mode = "TRACKING_FULL"
        #         self.head_stopped = False
        #         self._publish_intent(
        #             intent_type="TRACK_FACE_FULL",
        #             reason="face_stable_during_idle",
        #             triggering_event="FACE_STABLE",
        #             relevance=1.0
        #         )
        #
        # elif event.event_type == "FACE_LOST":
        #     self.get_logger().info("Face lost → Update state")
        #     self.tracking_mode = "SEARCHING"
        #     self.face_stable_since = None
        #     self.face_lost_since = now
        #     self.head_stopped = False
        #
        #     # Déclencher START_PERSON_SEARCH uniquement si aucun goal actif ne gère déjà la recherche
        #     # Ignorer aussi si le goal vient d'être complété (COMPLETED, ENGAGING) pour éviter de redémarrer
        #     if not (self.current_goal == "FIND_AND_GREET" and self.goal_state in ["SEARCHING", "IDLE", "FOUND", "ENGAGING", "COMPLETED"]):
        #         self._publish_intent(
        #             intent_type="START_PERSON_SEARCH",
        #             reason="face_lost",
        #             triggering_event="FACE_LOST",
        #             relevance=1.0
        #         )
        #     else:
        #         self.get_logger().info("[GOAL] Recherche déjà gérée par goal FIND_AND_GREET")

        # =========================
        # PERSON RECOGNITION (conditions sociales via world)
        # =========================
        # COMMENTÉ pour debug - conflits events directs vs world orchestration
        # if event.event_type == "PERSON_RECOGNIZED":
        #
        #     if self.world is None:
        #         return
        #
        #     if event.person_name == "":
        #         return
        #
        #     person_name = event.person_name
        #
        #     # ===== Logique GOAL: FIND_AND_GREET =====
        #     if self.current_goal == "FIND_AND_GREET":
        #         # Objectif actif: chercher et saluer une personne connue
        #
        #         if person_name not in self.greeted_persons:
        #             # Personne reconnue et pas encore saluée
        #             if self.world.face_stable:
        #                 self.get_logger().info(
        #                     f"[GOAL] FIND_AND_GREET → Personne trouvée: {person_name}"
        #                 )
        #                 self.goal_state = "ENGAGING"
        #                 self.greeted_persons.add(person_name)
        #
        #                 self._publish_intent(
        #                     intent_type="GREET_PERSON",
        #                     name=person_name,
        #                     reason="goal_find_and_greet",
        #                     triggering_event="PERSON_RECOGNIZED",
        #                     relevance=1.0
        #                 )
        #
        #                 # Marquer objectif comme complété
        #                 self.goal_state = "COMPLETED"
        #                 return
        #             else:
        #                 # Visage pas stable, attendre
        #                 self.goal_state = "FOUND"
        #                 return
        #
        #     # ===== Comportement réactif normal (sans goal) =====
        #
        #     # cooldown anti-spam
        #     if (now - self.last_greet_time) < self.greet_cooldown:
        #         return
        #
        #     # condition sociale: ne saluer que si visage stable (pas en mouvement)
        #     if not self.world.face_stable:
        #         return
        #
        #     self.last_greet_time = now
        #
        #     self._publish_intent(
        #         intent_type="GREET_PERSON",
        #         name=person_name,
        #         reason="recognized_person",
        #         triggering_event="PERSON_RECOGNIZED",
        #         relevance=0.8
        #     )

        # =========================
        # DIAGNOSTICS (conditions sociales via world)
        # =========================
        # COMMENTÉ pour debug - conflits events directs vs world orchestration
        # if event.event_type.startswith("DIAGNOSTIC"):
        #
        #     if self.world is None:
        #         return
        #
        #     severity = payload.get("severity", "info")
        #     key = payload.get("key", "")
        #
        #     last = self.last_diag_time.get(key, 0)
        #
        #     if (now - last) < self.diag_cooldown:
        #         return
        #
        #     self.last_diag_time[key] = now
        #
        #     # 🔴 ERROR → priorité haute
        #     if severity == "error" and payload.get("active", True):
        #
        #         self._publish_intent(
        #             intent_type="SPEAK",
        #             reason=f"diagnostic_error:{key}",
        #             name=""
        #         )
        #
        #     # 🟡 WARNING → seulement si mode IDLE (condition sociale)
        #     elif severity == "warning" and payload.get("active", True):
        #
        #         if self.world.mode == "IDLE":
        #
        #             self._publish_intent(
        #                 intent_type="SPEAK",
        #                 reason=f"diagnostic_warning:{key}",
        #                 name=""
        #             )
        #
        #     # 🟢 RESOLVED → optionnel
        #     elif payload.get("active") is False:
        #
        #         self._publish_intent(
        #             intent_type="SPEAK",
        #             reason=f"diagnostic_resolved:{key}",
        #             name=""
        #         )
        # ========================
        # TEXT GENERATED (AIML) (conditions sociales via world)
        # ========================
        # COMMENTÉ pour debug - conflits events directs vs world orchestration
        # if event.event_type == "TEXT_GENERATED":
        #
        #     if self.world is None:
        #         return
        #
        #     payload = json.loads(event.payload_json)
        #     text = payload.get("text", "")
        #
        #     # condition sociale: ne parler que si mode IDLE (pas déjà en conversation)
        #     if self.world.mode == "IDLE":
        #
        #         self._publish_intent(
        #             intent_type="SPEAK",
        #             payload={"text": text},
        #             reason="text_generated"
        #         )

        # ========================
        # GOAL MANAGEMENT (définir/annuler objectifs)
        # ========================

        if event.event_type == "SET_GOAL":
            try:
                payload = json.loads(event.payload_json)
                goal_name = payload.get("goal", "")
                timeout = payload.get("timeout", 60.0)

                if goal_name:
                    self._set_goal(goal_name, timeout)
                    self.get_logger().info(f"Goal défini via event: {goal_name}")
            except Exception as e:
                self.get_logger().error(f"Erreur lors du parsing SET_GOAL: {e}")

        if event.event_type == "CANCEL_GOAL":
            self._cancel_goal(reason="event_request")
            self.get_logger().info("Goal annulé via event")

        # ========================
        # MODULE_READY (modules démarrés par system_mode_manager)
        # ========================

        if event.event_type == "MODULE_READY":
            try:
                payload = json.loads(event.payload_json)
                module_name = payload.get("module", "")

                # Si on attendait des modules pour démarrer le goal
                if self.goal_state == "WAITING_REQUIREMENTS" and module_name in self.goal_requested_modules:
                    self.get_logger().info(f"[GOAL] Module prêt: {module_name}")

                    # Vérifier si tous les prérequis sont maintenant satisfaits
                    missing = self._check_goal_requirements(self.current_goal)
                    if not missing:
                        self.get_logger().info(f"[GOAL] Tous les prérequis satisfaits → Démarrage de {self.current_goal}")
                        self.goal_state = "IDLE"  # Passer en mode actif
                    else:
                        self.get_logger().info(f"[GOAL] Attente modules restants: {', '.join(missing)}")

            except Exception as e:
                self.get_logger().error(f"Erreur lors du parsing MODULE_READY: {e}")

        # ========================
        # MODULE_FAILED (échec démarrage module)
        # ========================

        if event.event_type == "MODULE_FAILED":
            try:
                payload = json.loads(event.payload_json)
                module_name = payload.get("module", "")
                reason = payload.get("reason", "unknown")

                # Si le module échoué était nécessaire au goal
                if self.goal_state == "WAITING_REQUIREMENTS" and module_name in self.goal_requested_modules:
                    self.get_logger().error(
                        f"[GOAL] Module {module_name} indisponible ({reason}) → Annulation de {self.current_goal}"
                    )
                    self._cancel_goal(reason="module_unavailable")

            except Exception as e:
                self.get_logger().error(f"Erreur lors du parsing MODULE_FAILED: {e}")

    # =========================
    def _publish_intent(self, intent_type, name="", reason="", payload=None,
                       triggering_event="", relevance=1.0, suppressed=False):

        msg = BehaviorIntent()
        now = self.get_clock().now().to_msg()

        msg.header.stamp = now
        msg.stamp = now
        msg.intent_type = intent_type
        msg.target_person_name = name
        msg.priority = 1.0
        msg.reason = reason

        msg.payload_json = json.dumps(payload or {})

        self.pub_intent.publish(msg)

        self.get_logger().info(
            f"INTENT → {intent_type} ({name})"
        )

        # Publier trace de décision pour debugging
        self._publish_trace(
            triggering_event=triggering_event,
            chosen_intent=intent_type,
            reason=reason,
            relevance=relevance,
            suppressed=suppressed
        )

    # =========================
    # TRACKING BEHAVIOR STATE MACHINE (logique locale + timers)
    # =========================
    def _update_tracking_behavior(self):
        """Update tracking behavior based on local state machine and timers."""
        now = self.get_clock().now().seconds_nanoseconds()[0]

        # =========================
        # ÉTAT 1: Face stable depuis X secondes → arrêter mouvement tête
        # =========================

        # Log de debug périodique
        if self.tracking_mode == "TRACKING_FULL" and self.face_stable_since is not None:
            elapsed = now - self.face_stable_since

            # Log toutes les secondes avec détails
            if int(elapsed) % 1 == 0 and elapsed > 0:
                self.get_logger().info(
                    f"[TIMER CHECK] mode={self.tracking_mode}, "
                    f"stable_since={self.face_stable_since is not None}, "
                    f"head_stopped={self.head_stopped}, "
                    f"elapsed={elapsed:.1f}s/{self.stable_head_stop_delay}s"
                )

        # Condition simplifiée: basée uniquement sur la state machine locale
        if (self.tracking_mode == "TRACKING_FULL" and
            self.face_stable_since is not None and
            not self.head_stopped):

            elapsed = now - self.face_stable_since

            if elapsed >= self.stable_head_stop_delay:
                self.get_logger().info(
                    f"✓ Face stable for {elapsed:.1f}s → Stop head movement")
                self.tracking_mode = "TRACKING_HEAD_ONLY"
                self.head_stopped = True
                self._publish_intent(
                    intent_type="TRACK_FACE_HEAD_ONLY",
                    reason="face_stable_timeout",
                    triggering_event="timer_check",
                    relevance=1.0
                )
        # =========================
        # ÉTAT 2: Recherche depuis 30s sans succès → arrêter
        # =========================
        if (self.tracking_mode == "SEARCHING" and
            self.face_lost_since is not None):

            elapsed = now - self.face_lost_since

            # Log de debug toutes les 5 secondes
            if int(elapsed) % 5 == 0 and elapsed > 0:
                self.get_logger().debug(
                    f"Searching for {elapsed:.1f}s (will timeout at {self.search_timeout}s)"
                )

            if elapsed >= self.search_timeout:
                self.get_logger().info(
                    f"Search timeout ({self.search_timeout}s) → Stop searching"
                )
                self.tracking_mode = "IDLE"
                self.face_lost_since = None

                # Si un goal est actif, l'annuler (il publiera STOP_PERSON_SEARCH)
                # Sinon, publier directement
                if self.current_goal == "FIND_AND_GREET" and self.goal_state == "SEARCHING":
                    self.get_logger().info("[GOAL] Timeout → Annulation du goal")
                    self._cancel_goal(reason="search_timeout")
                else:
                    self._publish_intent(
                        intent_type="STOP_PERSON_SEARCH",
                        reason="search_timeout",
                        triggering_event="timer_check",
                        relevance=1.0
                    )

    # =========================
    # GOALS PROACTIFS (objectifs comportementaux)
    # =========================

    def _update_goal_behavior(self):
        """
        Timer proactif : avance vers l'objectif actuel.
        Appelé toutes les 2 secondes.
        """
        if self.current_goal is None:
            return

        # Si on attend des modules, ne rien faire (attendre MODULE_READY)
        if self.goal_state == "WAITING_REQUIREMENTS":
            return

        now = self.get_clock().now().seconds_nanoseconds()[0]

        # Vérifier timeout du goal
        if self.goal_start_time and (now - self.goal_start_time) > self.goal_timeout:
            self.get_logger().warn(
                f"[GOAL] {self.current_goal} TIMEOUT après {self.goal_timeout}s → Annulation"
            )
            self._cancel_goal(reason="timeout")
            return

        # ===== GOAL: FIND_AND_GREET =====
        if self.current_goal == "FIND_AND_GREET":

            if self.goal_state == "IDLE":
                # Démarrer la recherche si aucun visage présent
                if self.world and not self.world.face_present:
                    self.get_logger().info("[GOAL] FIND_AND_GREET → Démarrage recherche")
                    self.goal_state = "SEARCHING"
                    self._publish_intent(
                        intent_type="START_PERSON_SEARCH",
                        reason="goal_find_and_greet",
                        triggering_event="goal_timer",
                        relevance=0.7
                    )
                elif self.world and self.world.face_present:
                    # Visage déjà présent
                    self.goal_state = "FOUND"
                    self.get_logger().info("[GOAL] FIND_AND_GREET → Visage déjà présent")

            elif self.goal_state == "SEARCHING":
                # Continuer la recherche (géré par le tracking)
                pass

            elif self.goal_state == "FOUND":
                # Attente de reconnaissance (géré dans _on_event)
                pass

            elif self.goal_state == "ENGAGING":
                # Salutation en cours (géré dans _on_event)
                pass

            elif self.goal_state == "COMPLETED":
                # Objectif atteint
                self.get_logger().info(f"[GOAL] {self.current_goal} COMPLETED")
                self._cancel_goal(reason="completed")

    def _set_goal(self, goal_name: str, timeout: float = 60.0):
        """
        Définit un nouvel objectif comportemental.
        Demande automatiquement les modules nécessaires si manquants.

        Args:
            goal_name: "FIND_AND_GREET", "PATROL", etc.
            timeout: Durée max en secondes (défaut: 60s)
        """
        # Annuler l'objectif précédent si existant
        if self.current_goal:
            self._cancel_goal(reason="new_goal")

        self.current_goal = goal_name
        self.goal_start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.goal_timeout = timeout
        self.greeted_persons = set()  # Reset
        self.goal_requested_modules = set()  # Reset

        self.get_logger().info(
            f"[GOAL] Nouvel objectif: {goal_name} (timeout: {timeout}s)"
        )

        # Vérification et demande automatique des prérequis
        if goal_name in GOAL_REQUIREMENTS:
            missing = self._check_goal_requirements(goal_name)
            if missing:
                # AUTONOME : Demander les modules manquants
                self.goal_state = "WAITING_REQUIREMENTS"
                self.get_logger().info(
                    f"[GOAL] Demande des prérequis manquants: {', '.join(missing)}"
                )
                self._request_modules(goal_name)
                return  # Attendre MODULE_READY

        # Prérequis OK → démarrer immédiatement
        self.goal_state = "IDLE"
        self.get_logger().info(f"[GOAL] Prérequis satisfaits → Démarrage de {goal_name}")

    def _cancel_goal(self, reason: str = "manual"):
        """
        Annule l'objectif actuel et libère les modules demandés.

        Args:
            reason: "completed", "timeout", "manual", "new_goal"
        """
        if self.current_goal:
            old_goal = self.current_goal
            self.get_logger().info(f"[GOAL] Annulation de {old_goal} (raison: {reason})")

            # Arrêter la recherche si en cours
            if self.goal_state == "SEARCHING" and self.tracking_mode == "SEARCHING":
                self._publish_intent(
                    intent_type="STOP_PERSON_SEARCH",
                    reason="goal_cancelled",
                    triggering_event="goal_cancel",
                    relevance=1.0
                )

            # AUTONOME : Libérer les modules demandés par le goal
            if self.goal_requested_modules:
                self._release_modules(old_goal)

        self.current_goal = None
        self.goal_state = "IDLE"
        self.goal_start_time = None
        self.greeted_persons = set()
        self.goal_requested_modules = set()

    def _check_goal_requirements(self, goal_name: str) -> list:
        """
        Vérifie si les prérequis d'un goal sont satisfaits.

        Args:
            goal_name: Nom du goal à vérifier

        Returns:
            Liste des prérequis manquants (vide si tout OK)
        """
        if goal_name not in GOAL_REQUIREMENTS:
            return []  # Pas de prérequis déclarés

        requirements = GOAL_REQUIREMENTS[goal_name]
        missing = []

        # Vérifier les services ROS2
        if "services" in requirements:
            for service_name in requirements["services"]:
                # Construire le nom complet du service
                # Ex: "face_follower" → "/qbo_face_follower/start"
                if service_name == "face_follower":
                    full_service_name = "/qbo_face_follower/start"
                else:
                    full_service_name = f"/{service_name}"

                # Vérifier disponibilité (timeout court: 0.5s)
                available = self.service_is_ready(full_service_name, timeout_sec=0.5)
                if not available:
                    missing.append(f"{service_name} (service: {full_service_name})")

        return missing

    def service_is_ready(self, service_name: str, timeout_sec: float = 0.5) -> bool:
        """
        Vérifie si un service ROS2 est disponible.

        Args:
            service_name: Nom complet du service (ex: "/qbo_face_follower/start")
            timeout_sec: Timeout de vérification

        Returns:
            True si disponible, False sinon
        """
        # Liste tous les services disponibles
        service_names = self.get_service_names_and_types()
        return any(name == service_name for name, _ in service_names)

    def _request_modules(self, goal_name: str):
        """
        Demande le démarrage des modules nécessaires pour un goal.
        Le system_mode_manager s'occupe de les démarrer si possible.
        """
        if goal_name not in GOAL_REQUIREMENTS:
            return

        requirements = GOAL_REQUIREMENTS[goal_name]
        if "services" not in requirements:
            return

        for service_name in requirements["services"]:
            # Vision : face_follower → REQUEST_MODULE(vision)
            if service_name == "face_follower":
                module_name = "vision"
            else:
                module_name = service_name

            self.goal_requested_modules.add(module_name)

            # Publier intent pour demander le module
            self._publish_intent(
                intent_type="REQUEST_MODULE",
                reason=f"goal_{goal_name}",
                triggering_event="goal_requirements",
                relevance=1.0,
                payload={"module": module_name}
            )
            self.get_logger().info(f"[GOAL] Demande module: {module_name}")

    def _release_modules(self, goal_name: str):
        """
        Libère les modules demandés par le goal terminé.
        """
        for module_name in self.goal_requested_modules:
            self._publish_intent(
                intent_type="RELEASE_MODULE",
                reason=f"goal_{goal_name}_finished",
                triggering_event="goal_complete",
                relevance=1.0,
                payload={"module": module_name}
            )
            self.get_logger().info(f"[GOAL] Libération module: {module_name}")

    def _publish_event(self, event_type: str, payload: dict):
        """Publie un event social (ex: GOAL_FAILED)."""
        msg = SocialEvent()
        msg.stamp = self.get_clock().now().to_msg()
        msg.event_type = event_type
        msg.source = "behavior_engine"
        msg.payload_json = json.dumps(payload)

        # Publier sur le topic events
        self.sub_event.topic_name  # Récupérer le topic depuis la subscription
        # Alternative: créer un publisher dédié ou utiliser un topic existant
        self.get_logger().info(f"EVENT → {event_type}: {payload}")

    # =========================
    # DECISION TRACE (DEBUG)
    # =========================
    def _publish_trace(self, triggering_event, chosen_intent, reason, relevance, suppressed=False):
        """Publie une trace de décision pour debugging."""
        msg = DecisionTrace()
        msg.stamp = self.get_clock().now().to_msg()
        msg.triggering_event = triggering_event
        msg.chosen_intent = chosen_intent
        msg.reason = reason
        msg.relevance_score = relevance
        msg.speech_allowed = (self.world.mode != "ENGAGED") if self.world else True
        msg.suppressed_by_cooldown = suppressed

        self.pub_trace.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SocialBehaviorEngine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()