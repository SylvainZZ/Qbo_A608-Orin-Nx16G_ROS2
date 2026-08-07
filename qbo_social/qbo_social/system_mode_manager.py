#!/usr/bin/env python3

import json
import rclpy

from rclpy.node import Node
from qbo_msgs.msg import BehaviorIntent, SocialEvent

# =============================================================================
# PROFILS ROS2
# =============================================================================
# Définition simple des profils disponibles et de leurs dépendances.
# system_mode_manager connaît les profils et peut demander leur démarrage/arrêt,
# mais c'est qbo_bringup_manager qui gère les détails (nœuds, hardware, etc.)
#
# Chaque profil définit :
#   - dependencies : liste des profils requis avant de démarrer celui-ci
#   - description  : description textuelle
#
# ⚠️  SYNCHRONISATION AUTOMATIQUE :
#     L'état des profils est synchronisé automatiquement via événements PROFILE_STATUS
#     publiés par diagnostics_adapter (depuis qbo_bringup_manager).
# =============================================================================

PROFILES = {
    "MINIMAL": {
        "dependencies": [],
        "description": "Profil de base : contrôleurs matériels, batterie, IMU, moteurs, écran",
    },
    "VISION": {
        "dependencies": ["MINIMAL"],
        "description": "Vision et suivi de visages",
    },
    "NAVIGATION": {
        "dependencies": ["MINIMAL"],
        "description": "Navigation autonome",
    },
    "VOICE_OUTPUT": {
        "dependencies": ["MINIMAL"],
        "description": "Synthèse vocale (TTS)",
    },
    "CONVERSATION_ENGINE": {
        "dependencies": ["MINIMAL", "VOICE_OUTPUT"],
        "description": "Moteur de conversation (AIML)",
    },
    "VOICE_INPUT": {
        "dependencies": ["MINIMAL", "VOICE_OUTPUT", "CONVERSATION_ENGINE"],
        "description": "Écoute vocale (STT)",
    },
}


# =============================================================================
# RÈGLES DE PROFILS AUTOMATIQUES
# =============================================================================
# Définit quels profils doivent être démarrés automatiquement selon l'état système.
# Chaque règle spécifie :
#   profiles  : liste de profils à activer
#   condition : callable(robot_state: dict) → bool (True = activer, False = désactiver)
#   priority  : ordre d'évaluation (haute priorité = évalué en premier)
#
# ⚠️  MINIMAL est toujours actif par défaut (géré par _check_profile)
# =============================================================================

AUTO_PROFILE_RULES = [
    {
        "id": "VOICE_DEFAULT",
        "profiles": ["VOICE_OUTPUT", "CONVERSATION_ENGINE"],
        # Active VOICE_OUTPUT + CONVERSATION_ENGINE sauf si batterie en état critique (level 2)
        # battery_status_level est alimenté par les événements DIAGNOSTIC de diagnostics_adapter
        "condition": lambda s: s.get("battery_status_level", 0) < 2,
        "priority": 10,
    },
    # {
    #     "id": "VISION_GOOD_BATTERY",
    #     "profiles": ["VISION"],
    #     "condition": lambda s: s.get("battery_status_level", 0) == 0,  # OK uniquement
    #     "priority": 20,
    # },
]


# =============================================================================
# RÈGLES SYSTÈME (filtrage d'intents)
# =============================================================================
# Chaque règle est un dict :
#   id        : identifiant unique (pour les logs)
#   condition : callable(robot_state: dict) → bool
#   mode      : nom du mode système activé
#   block     : liste d'intent_type bloqués complètement
#   degrade   : dict {intent_type_original: intent_type_remplaçant}
#   priority  : entier, valeur haute = plus prioritaire (évaluée en premier)
# =============================================================================

# ✅ SYSTEM_RULES — Version événementielle
# Les conditions réagissent maintenant aux niveaux de diagnostic (0=OK, 1=WARNING, 2=ERROR)
# publiés par les controllers bas niveau (battery_controller, hardwareOrinA608, etc.)
#
# Niveaux de diagnostic :
#   0 = OK        : Fonctionnement normal
#   1 = WARNING   : Problème mineur (ex: batterie faible, CPU chaud)
#   2 = ERROR     : Problème critique (ex: batterie critique)
#
# Les controllers publient automatiquement ces niveaux dans /diagnostics,
# diagnostics_adapter les convertit en événements DIAGNOSTIC que nous consommons.
# =============================================================================

SYSTEM_RULES = [
    # {
    #     "id": "VISION_MISSING",
    #     "condition": lambda s: s.get("nodes_present", {}).get("qbo_vision") is False,
    #     "mode": "DEGRADED_VISION",
    #     "block": ["TRACK_FACE_FULL", "TRACK_FACE_HEAD_ONLY", "START_PERSON_SEARCH"],
    #     "degrade": {},
    #     "priority": 60,
    # },
    # {
    #     "id": "CRITICAL_BATTERY",
    #     "condition": lambda s: s.get("battery_status_level", 0) == 2,  # ERROR
    #     "mode": "CRITICAL",
    #     "block": ["GREET_PERSON", "DANCE", "TRACK_FACE_FULL",
    #               "TRACK_FACE_HEAD_ONLY", "MOVE_BASE", "START_PERSON_SEARCH"],
    #     "degrade": {},
    #     "priority": 100,
    # },
    # {
    #     "id": "LOW_BATTERY",
    #     "condition": lambda s: s.get("battery_status_level", 0) == 1,  # WARNING
    #     "mode": "LOW_BATTERY",
    #     "block": ["DANCE", "START_PERSON_SEARCH"],
    #     "degrade": {
    #         "TRACK_FACE_FULL": "TRACK_FACE_HEAD_ONLY",
    #     },
    #     "priority": 50,
    # },
    {
        "id": "CHARGING",
        "condition": lambda s: s.get("External Power", "No").strip().lower() in ("yes", "oui", "true", "1"),
        "mode": "CHARGING",
        "block": ["MOVE_BASE"],
        "degrade": {},
        "priority": 30,
    },
]


# =============================================================================
# CAPACITÉS → PROFILS
# =============================================================================
# Mapping entre les capacités abstraites et les profils qui les fournissent.
# Une capacité peut être fournie par plusieurs profils (ex: VOICE via VOICE_OUTPUT
# ou CONVERSATION_ENGINE).
# =============================================================================

CAPABILITY_TO_PROFILE = {
    "VISION": ["VISION"],
    "NAVIGATION": ["NAVIGATION"],
    "VOICE_OUTPUT": ["VOICE_OUTPUT", "CONVERSATION_ENGINE"],
    "VOICE_INPUT": ["VOICE_INPUT"],
    "CONVERSATION": ["CONVERSATION_ENGINE"],
}


# =============================================================================
# EXIGENCES DES INTENTS
# =============================================================================
# Définit les capacités requises pour chaque type d'intent.
# Si une capacité est manquante, l'intent peut être :
#   - bloqué (si "block_if_missing": True)
#   - dégradé vers un fallback (si "fallback" est défini)
#   - laissé passer avec un warning
# =============================================================================

INTENT_REQUIREMENTS = {
    "GREET_PERSON": {
        "required_capabilities": ["VOICE_OUTPUT"],
        "block_if_missing": True,
        "description": "Saluer une personne nécessite la sortie vocale",
    },
    "QUERY_AIML": {
        "required_capabilities": ["CONVERSATION"],
        "block_if_missing": True,
        "description": "Requête AIML nécessite le moteur de conversation (CONVERSATION_ENGINE)",
    },
    "START_PERSON_SEARCH": {
        "required_capabilities": ["VISION"],
        "block_if_missing": True,
        "description": "Recherche de personne nécessite la vision",
    },
    "TRACK_FACE_FULL": {
        "required_capabilities": ["VISION"],
        "fallback": "TRACK_FACE_HEAD_ONLY",
        "description": "Suivi visage complet nécessite vision, sinon head only",
    },
    "TRACK_FACE_HEAD_ONLY": {
        "required_capabilities": [],
        "description": "Suivi tête uniquement (dynamixel), pas de vision requise",
    },
    "MOVE_BASE": {
        "required_capabilities": [],
        "description": "Mouvement de base, pas de capacité spécifique requise",
    },
    "DANCE": {
        "required_capabilities": [],
        "description": "Danse, pas de capacité spécifique requise",
    },
}


# =============================================================================
# EXTRACTEURS DE DIAGNOSTICS — SUPPRIMÉ
# =============================================================================
# ✅ Migration terminée : Toutes les métriques et états sont maintenant gérés par
#    diagnostics_adapter via événements structurés.
#
# Les controllers bas niveau (battery_controller, hardwareOrinA608, etc.) publient
# automatiquement dans /diagnostics. diagnostics_adapter extrait selon configuration
# JSON et convertit en événements que system_mode_manager consomme.
#
# Ancien système (SUPPRIMÉ) :
#   - DIAGNOSTIC_EXTRACTORS extrayait battery_voltage, cpu_temp_c, etc.
#   - _on_diagnostics() traitait directement /diagnostics
#   - SYSTEM_RULES comparait ces valeurs à des seuils
#
# Nouveau système (ACTIF) :
#   - diagnostics_adapter émet SYSTEM_STATE_UPDATED (clés critiques brutes : External Power, Power PC, ...)
#   - diagnostics_adapter émet PROFILE_STATUS (profils actifs)
#   - diagnostics_adapter émet DIAGNOSTIC (niveaux WARNING/ERROR)
#   - system_mode_manager consomme UNIQUEMENT les événements (pas de /diagnostics)
#
# Note : METRICS_UPDATED désactivé pour l'instant (trop changeant)
# =============================================================================


# =============================================================================
# NŒUD
# =============================================================================

class SystemModeManager(Node):

    def __init__(self):
        super().__init__('qbo_system_mode_manager')

        # ===== Paramètre : profil actif =====
        self.declare_parameter('active_profile', 'MINIMAL')

        # ===== État robot (alimenté par events uniquement) =====
        self.robot_state = {
            # Clés critiques (depuis SYSTEM_STATE_UPDATED)
            # Valeurs brutes stockées telles quelles ("Yes"/"No", "Ok"/"Error", etc.)
            # Exemples : "External Power", "Power PC", ...
            # Les règles SYSTEM_RULES font les vérifications Yes/No/Ok

            # Diagnostics niveaux (depuis DIAGNOSTIC)
            # Exemples : battery_status_level, cpu_status_level
            # 0 = OK, 1 = WARNING, 2 = ERROR

            # Présence des nœuds (depuis NODE_MISSING/NODE_RECOVERED)
            "nodes_present": {},

            # Profils actifs (depuis PROFILE_STATUS)
            "active_profiles": {},
            "available_capabilities": set(),
        }
        self._current_mode = "NORMAL"
        self._current_profile = self.get_parameter('active_profile').value

        # ===== Flag de synchronisation initiale =====
        self._profile_state_received = False  # True après réception du premier PROFILE_STATUS

        # ===== Gestion autonome des modules (pour goals proactifs) =====
        self.requested_modules = {}  # {module_name: {"requestor": str, "timestamp": float}}

        # ===== Profils inhibés manuellement (arrêt externe : API, autre nœud) =====
        # Un profil dans ce set ne sera PAS redémarré automatiquement par AUTO_PROFILE_RULES.
        # Il est retiré du set dès qu'il réapparaît dans PROFILE_STATUS (quelle que soit la source).
        self._manually_stopped_profiles: set = set()

        # ===== Profils arrêtés par nos propres auto-rules =====
        # Utilisé pour distinguer nos arrêts internes des arrêts externes dans PROFILE_STATUS.
        self._auto_stopped_profiles: set = set()

        # ===== Gestion VOICE_INPUT pilotée par le visage =====
        # VOICE_INPUT démarre sur FACE_STABLE et s'arrête sur FACE_LOST + debounce.
        # Ce mécanisme est indépendant des AUTO_PROFILE_RULES et des inhibitions manuelles.
        self._voice_input_stop_timer = None  # Timer debounce arrêt VOICE_INPUT
        self._VOICE_INPUT_STOP_DELAY = 5.0   # secondes avant arrêt après FACE_LOST
        self.module_to_profile = {   # Mapping module → profil ROS2
            "vision": "VISION",
            "navigation": "NAVIGATION",
            "voice_output": "VOICE_OUTPUT",
            "voice_input": "VOICE_INPUT",
            "conversation": "CONVERSATION_ENGINE",
        }

        # ===== Subscriptions =====
        self.create_subscription(
            BehaviorIntent,
            '/qbo_social/intent_raw',
            self._on_intent_raw,
            10
        )

        self.create_subscription(
            SocialEvent,
            '/qbo_social/events',
            self._on_social_event,
            10
        )

        # ===== Publishers =====
        self.pub_intent = self.create_publisher(
            BehaviorIntent,
            '/qbo_social/intent',
            10
        )

        self.pub_event = self.create_publisher(
            SocialEvent,
            '/qbo_social/events',
            10
        )

        # ===== Watchdog profil (toutes les 5 s) =====
        self.create_timer(5.0, self._check_profile)

        # ===== Gestion automatique des profils (toutes les 10 s) =====
        self.create_timer(10.0, self._manage_auto_profiles)

        # ===== Publication périodique de l'état (toutes les 15 s pour monitoring) =====
        # self.create_timer(15.0, self._publish_status)

        self.get_logger().info(
            f"SystemModeManager started — profil initial : {self._current_profile} "
            f"(synchronisation via événements)"
        )
        self.get_logger().info("  Mode: 100% événementiel (pas de souscription /diagnostics)")
        self.get_logger().info("  Watchdog: checking profile every 5s")
        self.get_logger().info("  Auto profiles: managing every 10s")
        self.get_logger().info("  Status publisher: on startup only (periodic disabled)")
        self.get_logger().info("  Intent publisher: /qbo_social/intent")

        # Publier l'état initial une fois au démarrage
        # (Le timer périodique est commenté pour l'instant)
        # self._publish_status()

    # =========================================================================
    # SYNCHRONISATION DU PROFIL ACTIF (SIMPLIFIÉ)
    # =========================================================================
    # ✅ Simplifié : Appelé depuis _on_social_event lors de la réception de PROFILE_STATUS.
    #    Les profils sont déjà structurés et validés par diagnostics_adapter.
    # =========================================================================

    def _sync_active_profile(self, active_profiles: list):
        """
        Synchronise le profil actif avec la liste des profils RUNNING.
        Sélectionne le profil le plus "complet" parmi ceux actifs.
        Priorité : FULL > VISION/NAVIGATION > MINIMAL > VOICE_* > CONVERSATION_ENGINE

        Args:
            active_profiles: Liste des profils actuellement RUNNING
        """
        # Filtrer les valeurs invalides (protection supplémentaire)
        active_profiles = [
            p for p in active_profiles
            if p and p.lower() != "none"
        ]

        if not active_profiles:
            # Aucun profil actif → retour à MINIMAL par défaut
            new_profile = "MINIMAL"
        else:
            # Ordre de priorité des profils (du plus complet au plus simple)
            priority_order = [
                "FULL",
                "VISION",
                "NAVIGATION",
                "MINIMAL",
                "VOICE_INPUT",
                "VOICE_OUTPUT",
                "CONVERSATION_ENGINE"
            ]

            # Trouver le profil le plus prioritaire parmi ceux actifs
            new_profile = None
            for profile in priority_order:
                if profile in active_profiles:
                    new_profile = profile
                    break

            # Si aucun profil connu n'est trouvé, prendre le premier
            if new_profile is None:
                new_profile = active_profiles[0]

        # Vérifier si le profil a changé
        if new_profile != self._current_profile:
            old_profile = self._current_profile
            self._current_profile = new_profile

            self.get_logger().info(
                f"[PROFILE] Profil synchronisé : {old_profile} → {new_profile} "
                f"(profils actifs: {active_profiles})"
            )

        # Vérifier si des modules demandés sont maintenant prêts
        self._check_requested_modules()

    def _check_requested_modules(self):
        """Vérifie si les modules demandés sont maintenant disponibles."""
        active_profiles = self.robot_state.get("active_profiles", {})

        for module_name, request_info in list(self.requested_modules.items()):
            profile_name = request_info["profile"]

            # Vérifier si le profil est maintenant actif
            if profile_name in active_profiles:
                self.get_logger().info(
                    f"[MODULE_READY] Profil {profile_name} actif → Module {module_name} prêt"
                )
                self._publish_module_ready(module_name)
                # Ne pas retirer de requested_modules, géré par RELEASE_MODULE

    def _update_available_capabilities(self):
        """
        Met à jour les capacités disponibles basées sur les profils actifs.
        Consulte CAPABILITY_TO_PROFILE pour déterminer quelles capacités
        sont fournies par les profils actuellement RUNNING.
        """
        active_profiles = set(self.robot_state["active_profiles"].keys())
        capabilities = set()

        for capability, profiles in CAPABILITY_TO_PROFILE.items():
            # Si au moins un des profils fournissant cette capacité est actif
            if any(p in active_profiles for p in profiles):
                capabilities.add(capability)

        old_capabilities = self.robot_state["available_capabilities"]
        if capabilities != old_capabilities:
            added = capabilities - old_capabilities
            removed = old_capabilities - capabilities

            self.robot_state["available_capabilities"] = capabilities

            if added or removed:
                self.get_logger().info(
                    f"[CAPABILITIES] Mise à jour : "
                    f"ajoutées={list(added)}, retirées={list(removed)}, "
                    f"disponibles={list(capabilities)}"
                )

    def _json_safe_robot_state(self) -> dict:
        """Prépare robot_state pour sérialisation JSON (convertit sets en listes)."""
        state = dict(self.robot_state)
        if "available_capabilities" in state:
            state["available_capabilities"] = list(state["available_capabilities"])
        return state

    def _publish_event(self, event_type: str, payload: dict = None, level_log: str = "info"):
        """Méthode générique pour publier un SocialEvent."""
        msg = SocialEvent()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = "qbo_social"
        msg.stamp = now
        msg.event_type = event_type
        msg.source = "system_mode_manager"
        msg.payload_json = json.dumps(payload or {})
        msg.person_id = ""
        msg.person_name = ""
        msg.confidence = 0.0

        self.pub_event.publish(msg)

        log = getattr(self.get_logger(), level_log, self.get_logger().info)
        log(f"[EVENT] {event_type}: {payload}")

    def _publish_status(self):
        """Publie l'état complet du système."""
        self._publish_event(
            "SYSTEM_STATUS",
            {
                "current_profile": self._current_profile,
                "current_mode": self._current_mode,
                "active_profiles": list(self.robot_state["active_profiles"].keys()),
                "external_power": self.robot_state.get("External Power", "Unknown"),
                "power_pc": self.robot_state.get("Power PC", "Unknown"),
                "battery_status_level": self.robot_state.get("battery_status_level", 0),
                "cpu_status_level": self.robot_state.get("cpu_status_level", 0),
            },
            level_log="debug"
        )

    # =========================================================================
    # ÉVALUATION DES RÈGLES
    # =========================================================================

    def _active_rules(self):
        """Retourne les règles actives triées par priorité décroissante."""
        return sorted(
            [r for r in SYSTEM_RULES if r["condition"](self.robot_state)],
            key=lambda r: r["priority"],
            reverse=True
        )

    def _evaluate_mode(self):
        """Réévalue le mode système et publie un événement si changement."""
        active = self._active_rules()
        new_mode = active[0]["mode"] if active else "NORMAL"

        if new_mode != self._current_mode:
            self.get_logger().info(
                f'[MODE] {self._current_mode} → {new_mode}'
            )
            self._current_mode = new_mode
            self._publish_mode_event(new_mode)

    def _publish_mode_event(self, mode: str):
        """Publie un changement de mode système."""
        self._publish_event(
            "SYSTEM_MODE_CHANGED",
            {"mode": mode, "robot_state": self._json_safe_robot_state()},
            level_log="debug"  # Log verbeux → debug seulement
        )

    # =========================================================================
    # EVENTS → robot_state
    # =========================================================================
    # Consomme les événements publiés par :
    #   - DiagnosticsAdapter : PROFILE_STATUS, SYSTEM_STATE_UPDATED, METRICS_UPDATED, DIAGNOSTIC
    #   - DiagnosticsInspector : NODE_MISSING, NODE_RECOVERED
    # =========================================================================

    def _payload(self, msg) -> dict:
        """Extrait le payload JSON d'un message de façon sécurisée.

        Fonctionne avec SocialEvent et BehaviorIntent.
        """
        if not hasattr(msg, 'payload_json') or not msg.payload_json:
            return {}
        try:
            return json.loads(msg.payload_json)
        except Exception as e:
            event_type = getattr(msg, 'event_type', None) or getattr(msg, 'intent_type', 'unknown')
            self.get_logger().warn(f"Invalid payload for {event_type}: {e}")
            return {}

    def _on_social_event(self, msg: SocialEvent):
        """Dispatch des événements vers les handlers appropriés."""
        handlers = {
            "PROFILE_STATUS": self._process_profile_status_event,
            "SYSTEM_STATE_UPDATED": self._process_system_state_event,
            # "METRICS_UPDATED": self._process_metrics_event,  # Désactivé (extracteurs commentés)
            "DIAGNOSTIC": self._process_diagnostic_event,
            "NODE_MISSING": self._process_node_presence_event,
            "NODE_RECOVERED": self._process_node_presence_event,
            # Gestion VOICE_INPUT pilotée par le visage
            "FACE_STABLE": self._on_face_stable,
            "FACE_LOST": self._on_face_lost,
        }

        handler = handlers.get(msg.event_type)
        if handler:
            handler(msg)

    def _process_node_presence_event(self, msg: SocialEvent):
        """Traite les événements NODE_MISSING et NODE_RECOVERED."""
        payload = self._payload(msg)
        hardware = payload.get("hardware")
        present = payload.get("present", False)

        if hardware:
            self.robot_state["nodes_present"][hardware] = present
            # Ne déclencher _check_profile() que si synchronisation initiale terminée
            if self._profile_state_received:
                self._check_profile()

    def _process_profile_status_event(self, msg: SocialEvent):
        """
        Traite les événements PROFILE_STATUS publiés par diagnostics_adapter.
        Met à jour robot_state avec les profils actifs et leurs capacités.

        Détecte aussi les arrêts externes (API, autre nœud) en comparant avant/après :
        - Profil disparu + pas dans _auto_stopped_profiles  → arrêt externe → inhibé
        - Profil disparu + dans _auto_stopped_profiles      → arrêt interne → ignoré
        - Profil apparu                                     → inhibition levée
        """
        payload = self._payload(msg)
        active_profiles = payload.get("active_profiles", [])
        active_set = set(active_profiles)

        # Profils précédemment actifs (avant cette mise à jour)
        previous_set = set(self.robot_state.get("active_profiles", {}).keys())

        # =====================================================================
        # HEURISTIQUE DE DÉMARRAGE (premier PROFILE_STATUS uniquement)
        # =====================================================================
        # Si des profils auto-gérés sont déjà actifs mais d'autres sont absents
        # → redémarrage de smm avec état partiel de bringup_manager
        # → inhiber les profils absents (ils ont probablement été arrêtés manuellement)
        #
        # Si seul MINIMAL tourne (ou rien) → cold boot normal → ne rien inhiber,
        # laisser AUTO_PROFILE_RULES démarrer les profils séquentiellement.
        # =====================================================================
        if not self._profile_state_received:
            non_minimal_active = active_set - {"MINIMAL"}
            if non_minimal_active:
                # Des profils auto-gérés tournent déjà → état partiel post-redémarrage
                auto_managed = set()
                for rule in AUTO_PROFILE_RULES:
                    auto_managed.update(rule.get("profiles", []))
                for profile in auto_managed - active_set:
                    self._manually_stopped_profiles.add(profile)
                    self.get_logger().info(
                        f"[STARTUP] Profil {profile!r} absent au démarrage (état partiel) → inhibé"
                    )

        # Détecter les changements
        disappeared = previous_set - active_set
        appeared = active_set - previous_set

        for profile in disappeared:
            if profile in self._auto_stopped_profiles:
                # Arrêt initié par _manage_auto_profiles → pas d'inhibition
                self._auto_stopped_profiles.discard(profile)
            else:
                # Arrêt externe (API, crash) → inhiber le redémarrage automatique
                self._manually_stopped_profiles.add(profile)
                self.get_logger().info(
                    f"[EXTERNAL_STOP] Profil {profile!r} inhibé — arrêt non initié par auto-rules"
                )

        for profile in appeared:
            if profile in self._manually_stopped_profiles:
                self._manually_stopped_profiles.discard(profile)
                self.get_logger().info(
                    f"[EXTERNAL_STOP] Profil {profile!r} réactivé — inhibition levée"
                )

        # Mettre à jour robot_state avec les profils actifs
        # Format compatible : {profile_name: "RUNNING"}
        self.robot_state["active_profiles"] = {
            profile: "RUNNING" for profile in active_profiles
        }

        # Marquer que l'état des profils a été reçu (synchronisation initiale terminée)
        if not self._profile_state_received:
            self._profile_state_received = True
            self.get_logger().info(
                "[PROFILE_STATUS] Synchronisation initiale terminée — watchdog activé"
            )

        # Synchroniser le profil actif principal
        self._sync_active_profile(active_profiles)

        # Mettre à jour les capacités disponibles
        self._update_available_capabilities()

        self.get_logger().debug(
            f"[PROFILE_STATUS] Profils actifs: {active_profiles}, "
            f"Capacités: {list(self.robot_state['available_capabilities'])}"
        )

    def _process_system_state_event(self, msg: SocialEvent):
        """
        Traite les événements SYSTEM_STATE_UPDATED publiés par diagnostics_adapter/inspector.
        Met à jour robot_state avec les clés critiques (valeurs brutes), la présence
        des nœuds, et réévalue les règles système.

        Format attendu du payload :
        {
            "system_state": {"External Power": "Yes", "Power PC": "Ok", ...},
            "nodes_present": {"qbo_dynamixel": true, "orin-nx-16g": false, ...},
            "changes": {"External Power": {"old": "No", "new": "Yes"}, ...}  # optionnel
        }
        """
        payload = self._payload(msg)
        system_state = payload.get("system_state", {})
        nodes_present = payload.get("nodes_present", {})
        changes = payload.get("changes", {})

        state_changed = False

        # Mettre à jour robot_state avec tous les états système (clés critiques)
        for key, value in system_state.items():
            old_value = self.robot_state.get(key)
            if value != old_value:
                self.robot_state[key] = value
                state_changed = True
                self.get_logger().info(
                    f"[SYSTEM_STATE] {key}: {old_value} → {value}"
                )

        # Mettre à jour la présence des nœuds si fournie
        for hardware, present in nodes_present.items():
            old_present = self.robot_state["nodes_present"].get(hardware)
            if present != old_present:
                self.robot_state["nodes_present"][hardware] = present
                state_changed = True
                self.get_logger().info(
                    f"[SYSTEM_STATE] node {hardware}: {old_present} → {present}"
                )

        # Réévaluer le mode système si au moins un état a changé
        if state_changed or changes:
            self._evaluate_mode()

    # def _process_metrics_event(self, msg: SocialEvent):
    #     """
    #     Traite les événements METRICS_UPDATED publiés par diagnostics_adapter.
    #     Met à jour robot_state avec les métriques (battery_voltage, cpu_temp_c, etc.)
    #
    #     Format attendu du payload :
    #     {
    #         "metrics": {"battery_voltage": 12.5, "cpu_temp_c": 45.2, ...},
    #         "changes": {"battery_voltage": {"old": 12.6, "new": 12.5}}
    #     }
    #     """
    #     payload = self._payload(msg)
    #     metrics = payload.get("metrics", {})
    #     changes = payload.get("changes", {})
    #
    #     # Mettre à jour robot_state avec toutes les métriques
    #     for key, value in metrics.items():
    #         old_value = self.robot_state.get(key)
    #         if value != old_value:
    #             self.robot_state[key] = value
    #             self.get_logger().debug(
    #                 f"[METRICS] {key}: {old_value} → {value}"
    #             )
    #
    #     # Les AUTO_PROFILE_RULES utilisent battery_voltage
    #     # Elles seront réévaluées au prochain timer (10s), pas besoin d'action immédiate

    def _process_diagnostic_event(self, msg: SocialEvent):
        """
        Traite les événements DIAGNOSTIC publiés par diagnostics_adapter.
        Met à jour les niveaux de diagnostic (battery_status_level, cpu_status_level, etc.)
        et réévalue les règles système.

        Format attendu du payload :
        {
            "key": "hardware|category",
            "severity": "info|warning|error",
            "message": "...",
            "level": 0|1|2,
            "active": true|false,
            "hardware": "Qboard_3",
            "category": "Battery Status"
        }
        """
        payload = self._payload(msg)
        hardware = payload.get("hardware", "unknown")
        category = payload.get("category", "unknown")
        level = payload.get("level", 0)
        active = payload.get("active", False)
        severity = payload.get("severity", "info")

        # Mapping des diagnostics critiques vers robot_state
        # Format: (hardware, category) → clé robot_state
        diagnostic_mapping = {
            ("Qboard_3", "Battery Status"): "battery_status_level",
            ("orin-nx-16g", "A608 Temp"): "cpu_status_level",
            # Ajouter d'autres mappings si nécessaire
        }

        target_key = diagnostic_mapping.get((hardware, category))

        if target_key:
            old_level = self.robot_state.get(target_key, 0)
            new_level = level if active else 0  # Si résolu, niveau = 0

            if new_level != old_level:
                self.robot_state[target_key] = new_level
                self.get_logger().info(
                    f"[DIAGNOSTIC] {target_key} : niveau {old_level} → {new_level} ({severity})"
                )
                # Réévaluer le mode système
                self._evaluate_mode()
        else:
            # Log pour les diagnostics non mappés (info seulement)
            if severity in ("warning", "error") and active:
                self.get_logger().warn(
                    f"[DIAGNOSTIC] {hardware}/{category} : {severity.upper()} - {payload.get('message', '')}"
                )

    # =========================================================================
    # GESTION AUTOMATIQUE DES PROFILS
    # =========================================================================

    def _manage_auto_profiles(self):
        """
        Gère le démarrage/arrêt automatique des profils selon les règles AUTO_PROFILE_RULES.
        Appelé périodiquement (toutes les 10s).

        ⚠️  Ne touche PAS aux profils demandés par des goals autonomes (REQUEST_MODULE).
        ⚠️  PRIORITÉ : MINIMAL doit être actif avant de démarrer d'autres profils.
        """
        # Attendre la synchronisation initiale avec diagnostics_adapter
        if not self._profile_state_received:
            self.get_logger().debug(
                "[AUTO_PROFILE] En attente de synchronisation initiale (PROFILE_STATUS)"
            )
            return

        active_profiles = self.robot_state.get("active_profiles", {})

        # ===== VÉRIFICATION PRÉALABLE : MINIMAL doit être actif =====
        minimal_running = "MINIMAL" in active_profiles

        if not minimal_running:
            self.get_logger().debug(
                "[AUTO_PROFILE] MINIMAL non actif → skip (watchdog le démarrera)"
            )
            return  # Attendre que MINIMAL soit actif avant de continuer

        for rule in sorted(AUTO_PROFILE_RULES, key=lambda r: r["priority"], reverse=True):
            # Évaluer la condition
            should_be_active = rule["condition"](self.robot_state)

            for profile in rule["profiles"]:
                is_active = profile in active_profiles

                # Vérifier si le profil est demandé par un goal autonome
                is_requested_by_goal = any(
                    req["profile"] == profile
                    for req in self.requested_modules.values()
                )

                # Démarrer le profil si condition remplie et pas actif
                if should_be_active and not is_active:
                    if profile in self._manually_stopped_profiles:
                        self.get_logger().debug(
                            f"[AUTO_PROFILE:{rule['id']}] {profile} inhibé (arrêt manuel) → skip"
                        )
                    else:
                        self.get_logger().info(
                            f"[AUTO_PROFILE:{rule['id']}] Condition OK → Starting {profile}"
                        )
                        self._publish_start_profile(profile)

                elif not should_be_active and is_active:
                    if is_requested_by_goal:
                        self.get_logger().info(
                            f"[AUTO_PROFILE:{rule['id']}] Condition NOK MAIS {profile} demandé par goal → Pas d'arrêt"
                        )
                    else:
                        self.get_logger().info(
                            f"[AUTO_PROFILE:{rule['id']}] Condition NOK → Stopping {profile}"
                        )
                        self._publish_stop_profile(profile)

    def _publish_profile_intent(self, action: str, profile_name: str):
        """Méthode générique pour publier un intent de gestion de profil."""
        msg = BehaviorIntent()
        msg.intent_type = action
        msg.payload_json = json.dumps({"profile": profile_name})
        self.pub_intent.publish(msg)
        self.get_logger().info(f"[INTENT] Published {action} for {profile_name}")

    def _publish_stop_profile(self, profile_name: str):
        """Publie un intent STOP_PROFILE."""
        # Tracer cet arrêt pour qu'il ne soit pas traité comme externe dans PROFILE_STATUS
        self._auto_stopped_profiles.add(profile_name)
        self._publish_profile_intent("STOP_PROFILE", profile_name)

    # =========================================================================
    # WATCHDOG PROFIL
    # =========================================================================

    def _check_profile(self):
        """Vérifie que le profil actif est réellement en cours d'exécution.
        Publie un BehaviorIntent START_PROFILE si le profil n'est pas actif.

        ⚠️  PRIORITÉ : MINIMAL doit toujours être démarré en premier.
        """
        # Attendre la synchronisation initiale avec diagnostics_adapter
        if not self._profile_state_received:
            self.get_logger().debug(
                "[WATCHDOG] En attente de synchronisation initiale (PROFILE_STATUS)"
            )
            return

        profile_name = self._current_profile
        active_profiles = self.robot_state.get("active_profiles", {})

        # ===== PRIORITÉ ABSOLUE : S'assurer que MINIMAL est démarré =====
        minimal_running = "MINIMAL" in active_profiles

        if not minimal_running and profile_name != "MINIMAL":
            # MINIMAL n'est pas actif mais devrait l'être → le démarrer d'abord
            self.get_logger().warn(
                f"[PROFIL MINIMAL] Non actif mais requis → Requesting START (current: {profile_name})"
            )
            self._publish_start_profile("MINIMAL")
            return  # Attendre que MINIMAL démarre avant de continuer

        # Vérifier si le profil est dans la liste des profils actifs (synchronisé depuis bringup_manager)
        is_profile_running = profile_name in active_profiles

        if not is_profile_running:
            # Le profil n'est pas actif, demander son démarrage
            self.get_logger().warn(
                f"[PROFIL {profile_name}] Profil non actif (active_profiles: {list(active_profiles.keys())}) → Requesting START"
            )
            self._publish_start_profile(profile_name)
        else:
            self.get_logger().debug(
                f"[PROFIL {profile_name}] Profil actif et opérationnel"
            )

    def _publish_start_profile(self, profile_name: str):
        """Publie un intent START_PROFILE."""
        self._publish_profile_intent("START_PROFILE", profile_name)

    # =========================================================================
    # GESTION VOICE_INPUT — PILOTÉE PAR LE VISAGE
    # =========================================================================

    def _on_face_stable(self, msg: SocialEvent):
        """
        Démarre VOICE_INPUT quand un visage est stable.
        Annule le timer d'arrêt différé si une face réapparaît avant l'échéance.
        """
        if not self._profile_state_received:
            return

        # Annuler le timer d'arrêt si en cours (la face est de retour)
        if self._voice_input_stop_timer is not None:
            self._voice_input_stop_timer.cancel()
            self._voice_input_stop_timer = None
            self.get_logger().debug("[VOICE_INPUT] Face stable → annulation arrêt différé")

        active_profiles = self.robot_state.get("active_profiles", {})
        if "VOICE_INPUT" not in active_profiles:
            self.get_logger().info("[VOICE_INPUT] Face stable → démarrage VOICE_INPUT")
            self._publish_start_profile("VOICE_INPUT")

    def _on_face_lost(self, msg: SocialEvent):
        """
        Planifie l'arrêt de VOICE_INPUT après un debounce de _VOICE_INPUT_STOP_DELAY secondes.
        Si la face réapparaît dans ce délai (_on_face_stable), le timer est annulé.
        """
        if not self._profile_state_received:
            return

        active_profiles = self.robot_state.get("active_profiles", {})
        if "VOICE_INPUT" not in active_profiles:
            return  # Déjà arrêté, rien à faire

        # Annuler un éventuel timer précédent
        if self._voice_input_stop_timer is not None:
            self._voice_input_stop_timer.cancel()

        self.get_logger().info(
            f"[VOICE_INPUT] Face perdue → arrêt VOICE_INPUT dans {self._VOICE_INPUT_STOP_DELAY:.0f}s"
        )
        self._voice_input_stop_timer = self.create_timer(
            self._VOICE_INPUT_STOP_DELAY,
            self._stop_voice_input_delayed
        )

    def _stop_voice_input_delayed(self):
        """Callback du timer debounce : arrête effectivement VOICE_INPUT."""
        # Timer one-shot : se détruire immédiatement
        if self._voice_input_stop_timer is not None:
            self._voice_input_stop_timer.cancel()
            self._voice_input_stop_timer = None

        active_profiles = self.robot_state.get("active_profiles", {})
        if "VOICE_INPUT" in active_profiles:
            self.get_logger().info("[VOICE_INPUT] Debounce écoulé → arrêt VOICE_INPUT")
            self._publish_stop_profile("VOICE_INPUT")

    # =========================================================================
    # GESTION AUTONOME DES MODULES (goals proactifs)
    # =========================================================================

    def _handle_request_module(self, msg: BehaviorIntent):
        """Gère une demande de module depuis un goal autonome."""
        try:
            payload = self._payload(msg)
            module_name = payload.get("module")
            requestor = "behavior_engine"  # Intent vient toujours du behavior_engine

            if not module_name:
                self.get_logger().error("[REQUEST_MODULE] Module name manquant")
                return

            # Mapper module → profil ROS2
            profile_name = self.module_to_profile.get(module_name)
            if not profile_name:
                self.get_logger().error(
                    f"[REQUEST_MODULE] Module inconnu: {module_name}"
                )
                self._publish_module_failed(module_name, "unknown_module")
                return

            # Enregistrer la demande
            timestamp = self.get_clock().now().seconds_nanoseconds()[0]
            self.requested_modules[module_name] = {
                "requestor": requestor,
                "timestamp": timestamp,
                "profile": profile_name
            }

            self.get_logger().info(
                f"[REQUEST_MODULE] {module_name} demandé par {requestor} → Démarrage profil {profile_name}"
            )

            # Vérifier si le profil est déjà actif
            active_profiles = self.robot_state.get("active_profiles", {})
            if profile_name in active_profiles:
                # Déjà actif → MODULE_READY immédiat
                self.get_logger().info(
                    f"[REQUEST_MODULE] {module_name} déjà actif → MODULE_READY immédiat"
                )
                self._publish_module_ready(module_name)
                return

            # Demander le démarrage du profil
            self._publish_start_profile(profile_name)

            # Attendre PROFILE_CHANGED pour publier MODULE_READY (géré dans _sync_active_profile)

        except Exception as e:
            self.get_logger().error(f"[REQUEST_MODULE] Erreur: {e}")

    def _handle_release_module(self, msg: BehaviorIntent):
        """Gère la libération d'un module par un goal."""
        try:
            payload = self._payload(msg)
            module_name = payload.get("module")

            if not module_name:
                self.get_logger().error("[RELEASE_MODULE] Module name manquant")
                return

            if module_name not in self.requested_modules:
                self.get_logger().warn(
                    f"[RELEASE_MODULE] Module {module_name} non demandé (ignoré)"
                )
                return

            # Retirer de la liste des demandes
            profile_name = self.requested_modules[module_name]["profile"]
            del self.requested_modules[module_name]

            self.get_logger().info(
                f"[RELEASE_MODULE] {module_name} libéré → Arrêt profil {profile_name}"
            )

            # Arrêter le profil si plus aucun module demandé ne l'utilise
            still_needed = any(
                req["profile"] == profile_name
                for req in self.requested_modules.values()
            )

            if not still_needed:
                self.get_logger().info(
                    f"[RELEASE_MODULE] Profil {profile_name} plus nécessaire → Arrêt"
                )
                self._publish_stop_profile(profile_name)
            else:
                self.get_logger().info(
                    f"[RELEASE_MODULE] Profil {profile_name} encore nécessaire (autres demandes actives)"
                )

        except Exception as e:
            self.get_logger().error(f"[RELEASE_MODULE] Erreur: {e}")

    def _publish_module_ready(self, module_name: str):
        """Publie un event MODULE_READY."""
        self._publish_event("MODULE_READY", {"module": module_name})

    def _publish_module_failed(self, module_name: str, reason: str):
        """Publie un event MODULE_FAILED."""
        self._publish_event(
            "MODULE_FAILED",
            {"module": module_name, "reason": reason},
            level_log="error"
        )

    # =========================================================================
    # FILTRE D'INTENTS (REFACTORISÉ)
    # =========================================================================
    # Flux en 3 étapes :
    #   1. Check system rules : L'intention est-elle autorisée par l'état système ?
    #   2. Check capabilities : Les capacités nécessaires sont-elles disponibles ?
    #   3. Degrade/block/pass : Publier l'intent (éventuellement dégradé) ou le bloquer
    # =========================================================================

    def _on_intent_raw(self, msg: BehaviorIntent):
        # ===== Gestion autonome des modules (goals proactifs) =====
        if msg.intent_type == "REQUEST_MODULE":
            self._handle_request_module(msg)
            return  # Ne pas republier cet intent

        if msg.intent_type == "RELEASE_MODULE":
            self._handle_release_module(msg)
            return  # Ne pas republier cet intent

        # ===== Filtrage en 3 étapes =====

        # Étape 1 : Vérifier les règles système (état global : batterie, charging, etc.)
        system_decision = self._check_system_rules(msg.intent_type)

        if system_decision["action"] == "block":
            self.get_logger().warn(
                f'[{system_decision["rule_id"]}] BLOQUÉ {msg.intent_type} '
                f'(mode: {self._current_mode}, raison: {system_decision["reason"]})'
            )
            return

        if system_decision["action"] == "degrade":
            self.get_logger().info(
                f'[{system_decision["rule_id"]}] DÉGRADÉ {msg.intent_type} → '
                f'{system_decision["new_intent"]} (mode: {self._current_mode})'
            )
            msg.intent_type = system_decision["new_intent"]

        # Étape 2 : Vérifier les capacités requises
        capability_decision = self._check_required_capabilities(msg.intent_type)

        if capability_decision["action"] == "block":
            self.get_logger().warn(
                f'[CAPABILITIES] BLOQUÉ {msg.intent_type} '
                f'(capacités manquantes: {capability_decision["missing"]})'
            )
            return

        if capability_decision["action"] == "degrade":
            self.get_logger().info(
                f'[CAPABILITIES] DÉGRADÉ {msg.intent_type} → '
                f'{capability_decision["new_intent"]} '
                f'(capacités manquantes: {capability_decision["missing"]})'
            )
            msg.intent_type = capability_decision["new_intent"]

        # Étape 3 : Publier l'intent (éventuellement modifié)
        self.pub_intent.publish(msg)

    def _check_system_rules(self, intent_type: str) -> dict:
        """
        Vérifie si l'intent est autorisé selon les règles système.

        Retourne un dict avec :
            action: "pass" | "block" | "degrade"
            rule_id: identifiant de la règle appliquée (si applicable)
            reason: raison du blocage (si applicable)
            new_intent: nouvel intent en cas de dégradation (si applicable)
        """
        active_rules = self._active_rules()

        for rule in active_rules:
            # Blocage
            if intent_type in rule["block"]:
                return {
                    "action": "block",
                    "rule_id": rule["id"],
                    "reason": rule["mode"],
                }

            # Dégradation
            if intent_type in rule["degrade"]:
                return {
                    "action": "degrade",
                    "rule_id": rule["id"],
                    "new_intent": rule["degrade"][intent_type],
                }

        return {"action": "pass"}

    def _check_required_capabilities(self, intent_type: str) -> dict:
        """
        Vérifie si les capacités requises pour l'intent sont disponibles.

        Retourne un dict avec :
            action: "pass" | "block" | "degrade"
            missing: liste des capacités manquantes (si applicable)
            new_intent: nouvel intent en cas de fallback (si applicable)
        """
        requirements = INTENT_REQUIREMENTS.get(intent_type)

        # Si l'intent n'a pas de requirements définis, laisser passer
        if not requirements:
            return {"action": "pass"}

        required_caps = set(requirements.get("required_capabilities", []))
        available_caps = self.robot_state["available_capabilities"]

        missing_caps = required_caps - available_caps

        # Toutes les capacités sont disponibles
        if not missing_caps:
            return {"action": "pass"}

        # Des capacités manquent

        # Si un fallback est défini, dégrader
        if "fallback" in requirements:
            return {
                "action": "degrade",
                "missing": list(missing_caps),
                "new_intent": requirements["fallback"],
            }

        # Si block_if_missing est activé, bloquer
        if requirements.get("block_if_missing", False):
            return {
                "action": "block",
                "missing": list(missing_caps),
            }

        # Sinon, laisser passer avec un warning
        self.get_logger().warn(
            f"[CAPABILITIES] Intent {intent_type} autorisé malgré capacités manquantes: "
            f"{list(missing_caps)}"
        )
        return {"action": "pass"}


def main(args=None):
    rclpy.init(args=args)
    node = SystemModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()