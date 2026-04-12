#!/usr/bin/env python3

import json
import re

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray
from qbo_msgs.msg import BehaviorIntent, SocialEvent


# =============================================================================
# PROFILS DE NŒUDS
# =============================================================================
# Chaque profil liste les hardware_id (DiagnosticStatus.hardware_id) qui
# DOIVENT être présents pour que le profil soit considéré opérationnel.
#
# Ces identifiants correspondent à ce que chaque nœud publie dans
# /diagnostics (champ hardware_id du DiagnosticStatus).
#
# Hiérarchie : FULL inclut tout, MINIMAL est le sous-ensemble de base.
# Un profil peut inclure les nœuds d'un autre via la clé "includes".
#
# ⚠️  SYNCHRONISATION AUTOMATIQUE :
#     Le profil actif est synchronisé automatiquement depuis les diagnostics
#     du qbo_bringup_manager (topic /diagnostics, name="qbo_bringup/profile/*").
#     Le profil le plus "complet" parmi ceux actifs est sélectionné :
#     Priorité : FULL > VISION/NAVIGATION > MINIMAL > VOICE_* > CONVERSATION_ENGINE
# =============================================================================

# hardware_id réels publiés dans /diagnostics (vérifiés depuis les sources) :
#
#   qbo_arduqbo package :
#     "System"         → qbo_arduqbo.cpp  (Arduqbo Status)
#     "Qboard_1"       → base_controller  (Base Status) + sensor_controller (Sensors Status)
#     "Qboard_3"       → battery_controller (Battery Status)
#     "Qboard_4"       → imu_controller   (IMU Status)
#     "Qboard_5"       → nose + mouth controllers
#     "LCD"            → lcd_controller   (LCD Status)
#     "qbo_dynamixel"  → dynamixel_controller (head_pan_joint, head_tilt_joint)
#
#   qbo_driver package :
#     "orin-nx-16g"    → hardwareOrinA608.py (A608 Power/Temp/Fan/Network)
#
#   qbo_vision package :
#     "qbo_vision"     → face_tracker.cpp (face_tracker)
#
# ⚠️  Ces identifiants peuvent différer selon la version du firmware Arduino.
#     Le DiagnosticsInspector (robot_state["nodes_present"]) permet de les
#     vérifier en live. Ajuster si nécessaire.

NODE_PROFILES = {
    "MINIMAL": {
        "required": [
            "System",           # qbo_arduqbo.cpp — contrôleur principal
            "Qboard_1",         # base + capteurs
            "Qboard_3",         # battery_controller
            "Qboard_4",         # imu_controller
            "Qboard_5",         # nose + mouth (LEDs)
            "LCD",              # lcd_controller
            "qbo_dynamixel",    # head_pan + head_tilt (suivi visage)
            "orin-nx-16g",      # Orin NX — TTS, audio, diag système
        ],
        "includes": [],
        "description": "Fonctionnement minimal : moteurs + batterie + IMU + nose + mouth + écran + dynamixel + Orin NX ",
    },
    "VISION": {
        "required": [
            "qbo_vision",       # face_tracker.cpp
            "qbo_dynamixel",    # head_pan + head_tilt (suivi visage)
        ],
        "includes": ["MINIMAL"],
        "description": "Minimal + vision et suivi de visages",
    },
    "NAVIGATION": {
        "required": [
            # À compléter quand le package de navigation publie ses diagnostics
            # ex: "qbo_navigation", "lidar"
        ],
        "includes": ["MINIMAL"],
        "description": "Minimal + navigation autonome",
    },
    "VOICE_OUTPUT": {
        "required": [
            # Le node TTS PICO devra publier ses diagnostics
            # ex: "qbo_tts_pico"
        ],
        "includes": ["MINIMAL"],
        "description": "Synthèse vocale via PICO TTS",
    },
    "CONVERSATION_ENGINE": {
        "required": [
            # Le node AIML devra publier ses diagnostics
            # ex: "qbo_aiml"
        ],
        "includes": ["MINIMAL, VOICE_OUTPUT"],
        "description": "Moteur de conversation AIML",
    },
    "VOICE_INPUT": {
        "required": [
            # Le node Whisper STT devra publier ses diagnostics
            # ex: "qbo_listen"
        ],
        "includes": ["MINIMAL, VOICE_OUTPUT, CONVERSATION_ENGINE"],
        "description": "Écoute vocale via Whisper STT",
    },
}


def resolve_required_nodes(profile_name: str) -> set:
    """Retourne l'ensemble des nœuds requis pour un profil (récursif)."""
    profile = NODE_PROFILES.get(profile_name)
    if not profile:
        return set()
    nodes = set(profile["required"])
    for included in profile.get("includes", []):
        nodes |= resolve_required_nodes(included)
    return nodes


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
        "id": "VOICE_BATTERY_OK",
        "profiles": ["VOICE_OUTPUT", "CONVERSATION_ENGINE"],
        "condition": lambda s: s.get("battery_voltage", 13.2) >= 12.2,  # Au-dessus du seuil LOW_BATTERY
        "priority": 10,
    },
    # Exemple : activer VISION si batterie > 12.5V
    # {
    #     "id": "VISION_GOOD_BATTERY",
    #     "profiles": ["VISION"],
    #     "condition": lambda s: s.get("battery_voltage", 13.2) >= 12.5,
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

# Seuils batterie (volts LiFePO4, cf. qboards_config.yaml) :
#   error_battery_level : 11.5 V
#   warn_battery_level  : 12.2 V
# Charge Mode : 1=CC, 2=CV, 3=fully charged, 4=battery operation
#   → charging = True si Charge Mode ∈ {1, 2}

SYSTEM_RULES = [
    # {
    #     "id": "VISION_MISSING",
    #     "condition": lambda s: s.get("nodes_present", {}).get("qbo_vision") is False,
    #     "mode": "DEGRADED_VISION",
    #     "block": ["TRACK_FACE_FULL", "TRACK_FACE_HEAD_ONLY", "START_PERSON_SEARCH"],
    #     "degrade": {},
    #     "priority": 60,
    # },
    {
        "id": "CRITICAL_BATTERY",
        "condition": lambda s: 0.0 < s.get("battery_voltage", 13.2) < 11.5,
        "mode": "CRITICAL",
        "block": ["GREET_PERSON", "DANCE", "TRACK_FACE_FULL",
                  "TRACK_FACE_HEAD_ONLY", "MOVE_BASE", "START_PERSON_SEARCH"],
        "degrade": {},
        "priority": 100,
    },
    {
        "id": "LOW_BATTERY",
        "condition": lambda s: 0.0 < s.get("battery_voltage", 13.2) < 12.2,
        "mode": "LOW_BATTERY",
        "block": ["DANCE", "START_PERSON_SEARCH"],
        "degrade": {
            "TRACK_FACE_FULL": "TRACK_FACE_HEAD_ONLY",
        },
        "priority": 50,
    },
    {
        "id": "CHARGING",
        "condition": lambda s: s.get("charging", False),
        "mode": "CHARGING",
        "block": ["MOVE_BASE"],
        "degrade": {},
        "priority": 30,
    },
    {
        "id": "CPU_OVERLOAD",
        "condition": lambda s: s.get("cpu_temp_c", 0.0) >= 70.0,
        "mode": "THERMAL_LIMIT",
        "block": ["DANCE"],
        "degrade": {
            "TRACK_FACE_FULL": "TRACK_FACE_HEAD_ONLY",
        },
        "priority": 40,
    },
]


# =============================================================================
# EXTRACTEURS DE DIAGNOSTICS
# =============================================================================
# Chaque extracteur lit un champ dans DiagnosticStatus.values et met à jour
# robot_state.  Les nœuds diagnostics publient des KeyValue dans status.values
# (ex: {key: "Percentage", value: "75.3"}).
#
#   name_pattern : regex sur status.name (insensible à la casse)
#   value_key    : clé cherchée dans la liste status.values[]
#   target       : clé dans self.robot_state à mettre à jour
#   transform    : callable(str) → type Python
#   fallback     : valeur utilisée si extraction échoue (None = ignorer)
# =============================================================================

# Clés réelles publiées par battery_controller.cpp (CBatteryController::diagnosticCallback) :
#   "Voltage"              → tension en volts  (ex: "12.60")
#   "Charge Mode"          → int str : 1=CC, 2=CV, 3=fully charged, 4=battery op
#   "External Power"       → "Yes" / "No"
#   "Charge Mode Description" → texte lisible
#   "Estimated Runtime"    → minutes
# Clés Orin (hardwareOrinA608.py) :
#   name "A608 Temp"   → "CPU °C", "GPU °C"
#   name "A608 Fan/RAM/CPU" → "RAM %", "CPU %", "GPU %"

DIAGNOSTIC_EXTRACTORS = [
    # --- Batterie ---
    {
        "name_pattern": r"Battery Status",
        "value_key": "Voltage",
        "target": "battery_voltage",
        "transform": lambda v: float(v.strip()),
        "fallback": None,
    },
    {
        "name_pattern": r"Battery Status",
        "value_key": "Charge Mode",
        "target": "charging",
        # charge_mode 1=CC charging, 2=CV charging → True ; 3=full, 4=battery → False
        "transform": lambda v: v.strip() in ("1", "2"),
        "fallback": None,
    },
    {
        "name_pattern": r"Battery Status",
        "value_key": "Estimated Runtime",
        "target": "battery_runtime_min",
        "transform": lambda v: float(v.strip()),
        "fallback": None,
    },
    # --- Orin NX ---
    {
        "name_pattern": r"A608 Temp",
        "value_key": "CPU °C",
        "target": "cpu_temp_c",
        "transform": lambda v: float(v.split()[0].strip()),
        "fallback": None,
    },
    {
        "name_pattern": r"A608 Temp",
        "value_key": "GPU °C",
        "target": "gpu_temp_c",
        "transform": lambda v: float(v.split()[0].strip()),
        "fallback": None,
    },
    {
        "name_pattern": r"A608 Fan",
        "value_key": "RAM %",
        "target": "ram_percent",
        "transform": lambda v: float(v.strip()),
        "fallback": None,
    },
    {
        "name_pattern": r"A608 Fan",
        "value_key": "CPU %",
        "target": "cpu_percent",
        "transform": lambda v: float(v.strip()),
        "fallback": None,
    },
    {
        "name_pattern": r"A608 Power",
        "value_key": "VDD_IN W",
        "target": "total_power_w",
        "transform": lambda v: float(v.split()[0].strip()),
        "fallback": None,
    },
]


# =============================================================================
# NŒUD
# =============================================================================

class SystemModeManager(Node):

    def __init__(self):
        super().__init__('qbo_system_mode_manager')

        # ===== Paramètre : profil actif =====
        self.declare_parameter('active_profile', 'MINIMAL')

        # ===== État robot (alimenté par /diagnostics et events) =====
        self.robot_state = {
            "battery_voltage": 13.2,   # V — valeur neutre LiFePO4 chargé
            "battery_runtime_min": -1.0,
            "charging": False,
            "cpu_temp_c": 0.0,
            "gpu_temp_c": 0.0,
            "cpu_percent": 0.0,
            "ram_percent": 0.0,
            "total_power_w": 0.0,
            "nodes_present": {},        # {hardware_id: bool} — alimenté par DiagnosticsInspector
            "missing_required_nodes": [],  # calculé à chaque watchdog
            "active_profiles": {},      # {profile_name: state} — synchronisé depuis qbo_bringup_manager
        }
        self._current_mode = "NORMAL"
        self._current_profile = self.get_parameter('active_profile').value

        # ===== Gestion autonome des modules (pour goals proactifs) =====
        self.requested_modules = {}  # {module_name: {"requestor": str, "timestamp": float}}
        self.module_to_profile = {   # Mapping module → profil ROS2
            "vision": "VISION",
            "navigation": "NAVIGATION",
            "voice_output": "VOICE_OUTPUT",
            "voice_input": "VOICE_INPUT",
            "conversation": "CONVERSATION_ENGINE",
        }

        # ===== Subscriptions =====
        self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self._on_diagnostics,
            10
        )

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
        self.create_timer(15.0, self._publish_status)

        self.get_logger().info(
            f"SystemModeManager started — profil initial : {self._current_profile} "
            f"(synchronisation automatique depuis /diagnostics)"
        )
        self.get_logger().info("  Watchdog: checking profile every 5s")
        self.get_logger().info("  Auto profiles: managing every 10s")
        self.get_logger().info("  Status publisher: every 15s on /qbo_social/events")
        self.get_logger().info("  Intent publisher: /qbo_social/intent")

        # Publier l'état initial immédiatement
        self._publish_status()

    # =========================================================================
    # DIAGNOSTICS → robot_state
    # =========================================================================

    def _on_diagnostics(self, msg: DiagnosticArray):
        changed = False
        profile_changed = False

        for status in msg.status:
            # ===== Synchronisation des profils actifs depuis bringup_manager =====
            if status.name.startswith("qbo_bringup/profile/"):
                # Extraire le nom du profil : "qbo_bringup/profile/VISION" → "VISION"
                profile_name = status.name.split("/")[-1]

                # Extraire l'état depuis les KeyValues
                profile_state = None
                for kv in status.values:
                    if kv.key == "state":
                        profile_state = kv.value  # "RUNNING", "STARTING", "STOPPED", etc.
                        break

                if profile_state:
                    # Mettre à jour l'état du profil
                    old_state = self.robot_state["active_profiles"].get(profile_name)
                    if profile_state == "RUNNING":
                        self.robot_state["active_profiles"][profile_name] = profile_state
                        if old_state != profile_state:
                            profile_changed = True
                    else:
                        # Si le profil n'est plus RUNNING, le retirer
                        if profile_name in self.robot_state["active_profiles"]:
                            del self.robot_state["active_profiles"][profile_name]
                            profile_changed = True
                continue

            # ===== Extraction des métriques système =====
            for extractor in DIAGNOSTIC_EXTRACTORS:
                if not re.search(extractor["name_pattern"], status.name, re.IGNORECASE):
                    continue

                raw = None
                for kv in status.values:
                    if kv.key == extractor["value_key"]:
                        raw = kv.value
                        break

                if raw is None:
                    continue

                try:
                    value = extractor["transform"](raw)
                except Exception:
                    if extractor["fallback"] is not None:
                        value = extractor["fallback"]
                    else:
                        continue

                if self.robot_state.get(extractor["target"]) != value:
                    self.robot_state[extractor["target"]] = value
                    changed = True

        # ===== Synchroniser le profil actif si changement =====
        if profile_changed:
            self._sync_active_profile()

        if changed:
            self._evaluate_mode()

    # =========================================================================
    # SYNCHRONISATION DU PROFIL ACTIF
    # =========================================================================

    def _sync_active_profile(self):
        """
        Synchronise le profil actif avec les profils détectés dans les diagnostics.
        Sélectionne le profil le plus "complet" parmi ceux actifs.
        Priorité : FULL > VISION/NAVIGATION > MINIMAL > VOICE_* > CONVERSATION_ENGINE
        """
        active_profiles = list(self.robot_state["active_profiles"].keys())

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

            # Publier un événement de changement de profil
            self._publish_profile_event(new_profile, old_profile, active_profiles)

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

    def _publish_profile_event(self, new_profile: str, old_profile: str, active_profiles: list):
        """Publie un événement PROFILE_CHANGED sur /qbo_social/events."""
        msg = SocialEvent()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'qbo_social'
        msg.stamp = now
        msg.event_type = 'PROFILE_CHANGED'
        msg.source = 'system_mode_manager'
        msg.payload_json = json.dumps({
            "new_profile": new_profile,
            "old_profile": old_profile,
            "active_profiles": active_profiles,
        })
        self.pub_event.publish(msg)

    def _publish_status(self):
        """
        Publie périodiquement l'état complet du système sur /qbo_social/events.
        Permet au SBE et aux autres composants de monitorer l'état sans attendre un changement.
        """
        msg = SocialEvent()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'qbo_social'
        msg.stamp = now
        msg.event_type = 'SYSTEM_STATUS'
        msg.source = 'system_mode_manager'
        msg.payload_json = json.dumps({
            "current_profile": self._current_profile,
            "current_mode": self._current_mode,
            "active_profiles": list(self.robot_state["active_profiles"].keys()),
            "missing_required_nodes": self.robot_state.get("missing_required_nodes", []),
            "battery_voltage": self.robot_state.get("battery_voltage", 0.0),
            "charging": self.robot_state.get("charging", False),
            "cpu_temp_c": self.robot_state.get("cpu_temp_c", 0.0),
            "cpu_percent": self.robot_state.get("cpu_percent", 0.0),
            "ram_percent": self.robot_state.get("ram_percent", 0.0),
        })
        self.pub_event.publish(msg)

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
                f'[MODE] {self._current_mode} → {new_mode} '
                f'(state: {self.robot_state})'
            )
            self._current_mode = new_mode
            self._publish_mode_event(new_mode)

    def _publish_mode_event(self, mode: str):
        msg = SocialEvent()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'qbo_social'
        msg.stamp = now
        msg.event_type = 'SYSTEM_MODE_CHANGED'
        msg.source = 'system_mode_manager'
        msg.payload_json = json.dumps({
            "mode": mode,
            "robot_state": self.robot_state
        })
        self.pub_event.publish(msg)

    # =========================================================================
    # EVENTS → robot_state (nodes_present depuis DiagnosticsInspector)
    # =========================================================================

    def _on_social_event(self, msg: SocialEvent):
        if msg.event_type not in ("NODE_MISSING", "NODE_RECOVERED"):
            return

        try:
            payload = json.loads(msg.payload_json)
        except Exception:
            return

        hardware = payload.get("hardware")
        present = payload.get("present", False)

        if hardware:
            self.robot_state["nodes_present"][hardware] = present
            self._check_profile()

    # =========================================================================
    # GESTION AUTOMATIQUE DES PROFILS
    # =========================================================================

    def _manage_auto_profiles(self):
        """
        Gère le démarrage/arrêt automatique des profils selon les règles AUTO_PROFILE_RULES.
        Appelé périodiquement (toutes les 10s).
        
        ⚠️  Ne touche PAS aux profils demandés par des goals autonomes (REQUEST_MODULE).
        """
        active_profiles = self.robot_state.get("active_profiles", {})

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
                    self.get_logger().info(
                        f"[AUTO_PROFILE:{rule['id']}] Condition OK → Starting {profile} "
                        f"(battery: {self.robot_state.get('battery_voltage', 0):.2f}V)"
                    )
                    self._publish_start_profile(profile, missing_nodes=[])

                # Arrêter le profil si condition non remplie et actif
                # MAIS seulement si PAS demandé par un goal autonome
                elif not should_be_active and is_active:
                    if is_requested_by_goal:
                        self.get_logger().info(
                            f"[AUTO_PROFILE:{rule['id']}] Condition NOK MAIS {profile} demandé par goal → Pas d'arrêt "
                            f"(battery: {self.robot_state.get('battery_voltage', 0):.2f}V)"
                        )
                    else:
                        self.get_logger().info(
                            f"[AUTO_PROFILE:{rule['id']}] Condition NOK → Stopping {profile} "
                            f"(battery: {self.robot_state.get('battery_voltage', 0):.2f}V)"
                        )
                        self._publish_stop_profile(profile)

    def _publish_stop_profile(self, profile_name: str):
        """Publie un intent STOP_PROFILE."""
        msg = BehaviorIntent()
        msg.intent_type = "STOP_PROFILE"
        msg.payload_json = json.dumps({
            "profile": profile_name,
        })
        self.pub_intent.publish(msg)
        self.get_logger().info(f"[INTENT] Published STOP_PROFILE for {profile_name}")

    # =========================================================================
    # WATCHDOG PROFIL
    # =========================================================================

    def _check_profile(self):
        """Vérifie que le profil actif est réellement en cours d'exécution.
        Publie un BehaviorIntent START_PROFILE si le profil n'est pas actif."""
        profile_name = self._current_profile
        active_profiles = self.robot_state.get("active_profiles", {})

        # Vérifier si le profil est dans la liste des profils actifs (synchronisé depuis bringup_manager)
        is_profile_running = profile_name in active_profiles

        if not is_profile_running:
            # Le profil n'est pas actif, demander son démarrage
            self.get_logger().warn(
                f"[PROFIL {profile_name}] Profil non actif (active_profiles: {list(active_profiles.keys())}) → Requesting START"
            )
            self._publish_start_profile(profile_name, missing_nodes=[])
        else:
            self.get_logger().debug(
                f"[PROFIL {profile_name}] Profil actif et opérationnel"
            )

        # Mettre à jour missing_required_nodes pour info (basé sur nodes_present)
        required = resolve_required_nodes(profile_name)
        nodes_present = self.robot_state.get("nodes_present", {})
        missing = [
            n for n in required
            if nodes_present.get(n) is False   # explicitement absent
        ]
        self.robot_state["missing_required_nodes"] = missing

    def _publish_start_profile(self, profile_name: str, missing_nodes: list):
        msg = BehaviorIntent()
        msg.intent_type = "START_PROFILE"
        msg.payload_json = json.dumps({
            "profile": profile_name,
            "missing_nodes": missing_nodes,
        })
        self.pub_intent.publish(msg)
        self.get_logger().info(f"[INTENT] Published START_PROFILE for {profile_name}")

    # =========================================================================
    # GESTION AUTONOME DES MODULES (goals proactifs)
    # =========================================================================

    def _handle_request_module(self, msg: BehaviorIntent):
        """Gère une demande de module depuis un goal autonome."""
        try:
            payload = json.loads(msg.payload_json) if msg.payload_json else {}
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
            self._publish_start_profile(profile_name, missing_nodes=[])

            # Attendre PROFILE_CHANGED pour publier MODULE_READY (géré dans _sync_active_profile)

        except Exception as e:
            self.get_logger().error(f"[REQUEST_MODULE] Erreur: {e}")

    def _handle_release_module(self, msg: BehaviorIntent):
        """Gère la libération d'un module par un goal."""
        try:
            payload = json.loads(msg.payload_json) if msg.payload_json else {}
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
        msg = SocialEvent()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'qbo_social'
        msg.stamp = now
        msg.event_type = 'MODULE_READY'
        msg.source = 'system_mode_manager'
        msg.payload_json = json.dumps({
            "module": module_name,
        })
        self.pub_event.publish(msg)
        self.get_logger().info(f"[EVENT] MODULE_READY: {module_name}")

    def _publish_module_failed(self, module_name: str, reason: str):
        """Publie un event MODULE_FAILED."""
        msg = SocialEvent()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'qbo_social'
        msg.stamp = now
        msg.event_type = 'MODULE_FAILED'
        msg.source = 'system_mode_manager'
        msg.payload_json = json.dumps({
            "module": module_name,
            "reason": reason,
        })
        self.pub_event.publish(msg)
        self.get_logger().error(f"[EVENT] MODULE_FAILED: {module_name} (raison: {reason})")

    # =========================================================================
    # FILTRE D'INTENTS
    # =========================================================================

    def _on_intent_raw(self, msg: BehaviorIntent):
        # ===== Gestion autonome des modules (goals proactifs) =====
        if msg.intent_type == "REQUEST_MODULE":
            self._handle_request_module(msg)
            return  # Ne pas republier cet intent

        if msg.intent_type == "RELEASE_MODULE":
            self._handle_release_module(msg)
            return  # Ne pas republier cet intent

        # ===== Filtre d'intents normal =====
        active = self._active_rules()

        for rule in active:
            # Blocage
            if msg.intent_type in rule["block"]:
                self.get_logger().warn(
                    f'[{rule["id"]}] BLOQUÉ  {msg.intent_type}  (mode: {self._current_mode})'
                )
                return

            # Dégradation
            if msg.intent_type in rule["degrade"]:
                replacement = rule["degrade"][msg.intent_type]
                self.get_logger().info(
                    f'[{rule["id"]}] DÉGRADÉ {msg.intent_type} → {replacement}  (mode: {self._current_mode})'
                )
                msg.intent_type = replacement
                break  # une seule dégradation par intent

        self.pub_intent.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()