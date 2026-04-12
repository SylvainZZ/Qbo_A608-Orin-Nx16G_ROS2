#!/usr/bin/env python3
"""
QBO Bringup Manager — Orchestrateur de profils de nœuds

Mimique le comportement des Lifecycle Nodes avec une gestion centralisée
des launch files modulaires. Expose un service ROS 2 pour :
  - Démarrer/arrêter des profils complets (MINIMAL, VISION, FULL)
  - Interroger l'état des nœuds actifs
  - Gérer le cycle de vie des processus launch
  - Publier l'état du système sur /diagnostics (toutes les 2s)

Architecture :
  Service: /qbo_bringup/manage_profile
  Request: { profile_name, action: START|STOP|RESTART|STATUS }
  Response: { success, message, active_nodes[], profile_state }

  Topic: /diagnostics (diagnostic_msgs/DiagnosticArray)
  - Status global du manager (OK/WARN/ERROR)
  - Status de chaque profil actif (état, PID, dépendances)

Usage:
    ros2 run qbo_bringup qbo_bringup_manager

    # Exemple d'appel service :
    ros2 service call /qbo_bringup/manage_profile qbo_msgs/srv/ManageProfile \
        "{profile_name: 'VISION', action: 'START'}"
    # Écouter les diagnostics :
    ros2 topic echo /diagnostics
"""

import os
import sys
import subprocess
import signal
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

# Import service
from qbo_msgs.srv import ManageProfile

# Import diagnostics
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


# =============================================================================
# CONFIGURATION DES PROFILS
# =============================================================================
# Chaque profil référence un launch file dans qbo_bringup/launch/
# Les launch files sont modulaires et peuvent inclure d'autres profiles

PROFILE_LAUNCH_FILES = {
    "MINIMAL": "profile_minimal.launch.py",
    "VISION": "profile_vision.launch.py",
    "NAVIGATION": "profile_navigation.launch.py",
    "VOICE_OUTPUT": "voice_output.launch.py",
    "VOICE_INPUT": "voice_input.launch.py",
    "CONVERSATION_ENGINE": "conversation_engine.launch.py",
}

# Dépendances entre profils (ce qui est inclus automatiquement)
PROFILE_DEPENDENCIES = {
    "MINIMAL": [],                      # Pas de dépendances
    "VISION": ["MINIMAL"],              # VISION inclut MINIMAL
    "NAVIGATION": ["MINIMAL"],          # NAVIGATION inclut MINIMAL
    "VOICE_OUTPUT": ["MINIMAL"],        # VOICE_OUTPUT inclut MINIMAL
    "VOICE_INPUT": ["MINIMAL"],         # VOICE_INPUT inclut MINIMAL
    "CONVERSATION_ENGINE": ["MINIMAL"], # CONVERSATION_ENGINE inclut MINIMAL
}

# États des profils (mimique Lifecycle)
STATE_STOPPED = 0
STATE_STARTING = 1
STATE_RUNNING = 2
STATE_STOPPING = 3
STATE_ERROR = 4


# =============================================================================
# PROFILE PROCESS WRAPPER
# =============================================================================

class ProfileProcess:
    """Encapsule un processus subprocess pour un profil donné."""

    def __init__(self, profile_name: str, launch_file_path: str, logger):
        self.profile_name = profile_name
        self.launch_file_path = launch_file_path
        self.logger = logger

        self.process: Optional[subprocess.Popen] = None
        self.state = STATE_STOPPED

    def start(self):
        """Démarre le launch file via subprocess."""
        if self.state == STATE_RUNNING:
            self.logger.warn(f"Profile {self.profile_name} already running")
            return False

        try:
            self.state = STATE_STARTING
            self.logger.info(f"Starting profile {self.profile_name} from {self.launch_file_path}")

            # Construire la commande ros2 launch
            cmd = [
                'ros2', 'launch',
                self.launch_file_path
            ]

            # Lancer le processus
            # IMPORTANT : Ne PAS capturer stdout/stderr dans des pipes non-lus !
            # Cela fait bloquer les processus enfants quand les buffers sont pleins
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,  # Évite le blocage des buffers
                stderr=subprocess.DEVNULL,  # Évite le blocage des buffers
                start_new_session=True  # Créer un nouveau groupe (compatible Windows + Linux)
            )

            self.state = STATE_RUNNING
            self.logger.info(f"Profile {self.profile_name} started successfully (PID: {self.process.pid})")
            return True

        except Exception as e:
            self.logger.error(f"Failed to start profile {self.profile_name}: {e}")
            self.state = STATE_ERROR
            return False

    def stop(self):
        """Arrête proprement le processus."""
        if self.state != STATE_RUNNING:
            self.logger.warn(f"Profile {self.profile_name} not running")
            return False

        try:
            self.state = STATE_STOPPING
            self.logger.info(f"Stopping profile {self.profile_name}")

            if self.process:
                # Envoyer SIGTERM (Windows: terminate, Linux: au groupe si possible)
                try:
                    if sys.platform != 'win32':
                        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                    else:
                        self.process.terminate()
                except (ProcessLookupError, AttributeError):
                    self.process.terminate()

                # Attendre max 5 secondes
                try:
                    self.process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    self.logger.warn(f"Profile {self.profile_name} didn't stop gracefully, forcing...")
                    try:
                        if sys.platform != 'win32':
                            os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                        else:
                            self.process.kill()
                    except (ProcessLookupError, AttributeError):
                        self.process.kill()
                    self.process.wait()

            self.state = STATE_STOPPED
            self.logger.info(f"Profile {self.profile_name} stopped successfully")
            return True

        except Exception as e:
            self.logger.error(f"Failed to stop profile {self.profile_name}: {e}")
            self.state = STATE_ERROR
            return False

    def restart(self):
        """Redémarre le profil."""
        self.stop()
        return self.start()

    def get_state(self) -> int:
        """Retourne l'état actuel du profil."""
        # Vérifier si le processus est toujours en vie
        if self.state == STATE_RUNNING and self.process:
            poll_result = self.process.poll()
            if poll_result is not None:
                # Le processus s'est terminé
                self.logger.warn(f"Profile {self.profile_name} process terminated unexpectedly (code: {poll_result})")
                self.state = STATE_ERROR
            else:
                # Vérifier que le processus n'est pas zombie
                try:
                    # Sur Linux, envoyer signal 0 ne tue pas mais vérifie l'existence
                    if sys.platform != 'win32':
                        os.kill(self.process.pid, 0)
                except (OSError, ProcessLookupError):
                    self.logger.warn(f"Profile {self.profile_name} process PID {self.process.pid} not found (zombie/dead)")
                    self.state = STATE_ERROR

        return self.state


# =============================================================================
# BRINGUP MANAGER NODE
# =============================================================================

class QboBringupManager(Node):
    """
    Gestionnaire centralisé des profils de nœuds QBO.

    Expose un service ROS 2 pour contrôler les profils de manière similaire
    aux Lifecycle Nodes, sans nécessiter que chaque nœud implémente l'interface
    Lifecycle.
    """

    def __init__(self):
        super().__init__('qbo_bringup_manager')

        # ===== Stockage des profils actifs =====
        self.active_profiles: Dict[str, ProfileProcess] = {}

        # ===== Résolution des chemins des launch files =====
        try:
            self.bringup_pkg_path = get_package_share_directory('qbo_bringup')
            self.launch_dir = os.path.join(self.bringup_pkg_path, 'launch')
        except Exception as e:
            self.get_logger().error(f"Failed to find qbo_bringup package: {e}")
            self.launch_dir = None

        # ===== Service ROS 2 =====
        self.srv = self.create_service(
            ManageProfile,
            '/qbo_bringup/manage_profile',
            self._handle_manage_profile
        )

        # ===== Publisher Diagnostics =====
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )

        # ===== Timer pour publier l'état régulièrement =====
        self.diagnostics_timer = self.create_timer(
            2.0,  # Publier toutes les 2 secondes
            self._publish_diagnostics
        )

        self.get_logger().info("=" * 70)
        self.get_logger().info("QBO Bringup Manager started")
        self.get_logger().info(f"  Service: /qbo_bringup/manage_profile")
        self.get_logger().info(f"  Diagnostics: /diagnostics (published every 2s)")
        self.get_logger().info(f"  Launch directory: {self.launch_dir}")
        self.get_logger().info(f"  Available profiles: {list(PROFILE_LAUNCH_FILES.keys())}")
        self.get_logger().info("  Profile dependencies:")
        for profile, deps in PROFILE_DEPENDENCIES.items():
            if deps:
                self.get_logger().info(f"    {profile} includes: {deps}")
        self.get_logger().info("=" * 70)

    # =========================================================================
    # SERVICE CALLBACK
    # =========================================================================

    def _handle_manage_profile(self, request, response):
        """
        Callback du service ManageProfile.

        Args:
            request.profile_name: Nom du profil (MINIMAL, VISION, FULL, etc.)
            request.action: START | STOP | RESTART | STATUS
            request.target_nodes: (optionnel) Liste de nœuds spécifiques

        Returns:
            response avec success, message, active_nodes[], profile_state
        """
        profile = request.profile_name.upper()
        action = request.action.upper()

        self.get_logger().info(f"Service request: {action} profile={profile}")

        # ===== Validation du profil =====
        if profile not in PROFILE_LAUNCH_FILES:
            response.success = False
            response.message = f"Unknown profile: {profile}. Available: {list(PROFILE_LAUNCH_FILES.keys())}"
            response.active_nodes = list(self.active_profiles.keys())
            response.profile_state = STATE_ERROR
            self.get_logger().warn(response.message)
            return response

        # ===== ACTION: STATUS =====
        if action == "STATUS":
            return self._handle_status(profile, response)

        # ===== ACTION: START =====
        elif action == "START":
            return self._handle_start(profile, response)

        # ===== ACTION: STOP =====
        elif action == "STOP":
            return self._handle_stop(profile, response)

        # ===== ACTION: RESTART =====
        elif action == "RESTART":
            return self._handle_restart(profile, response)

        # ===== ACTION inconnue =====
        else:
            response.success = False
            response.message = f"Unknown action: {action}. Use START|STOP|RESTART|STATUS"
            response.active_nodes = list(self.active_profiles.keys())
            response.profile_state = STATE_ERROR
            return response

    # =========================================================================
    # ACTIONS HANDLERS
    # =========================================================================

    def _handle_status(self, profile: str, response):
        """Retourne l'état d'un profil (en tenant compte des dépendances)."""
        # Vérifier si le profil tourne directement
        if profile in self.active_profiles:
            proc = self.active_profiles[profile]
            response.success = True
            response.message = f"Profile {profile} state: {self._state_name(proc.get_state())}"
            response.active_nodes = list(self.active_profiles.keys())
            response.profile_state = proc.get_state()
            return response

        # Vérifier si le profil tourne via une dépendance
        running_via = self._is_running_via_dependency(profile)
        if running_via:
            response.success = True
            response.message = f"Profile {profile} is RUNNING (included by {running_via})"
            response.active_nodes = list(self.active_profiles.keys())
            response.profile_state = STATE_RUNNING
            return response

        # Le profil ne tourne pas
        response.success = True
        response.message = f"Profile {profile} is not running (STOPPED)"
        response.active_nodes = list(self.active_profiles.keys())
        response.profile_state = STATE_STOPPED
        return response

    def _handle_start(self, profile: str, response):
        """Démarre un profil (vérifie que les dépendances sont actives)."""
        # Vérifier si déjà actif directement
        if profile in self.active_profiles:
            proc = self.active_profiles[profile]
            if proc.get_state() == STATE_RUNNING:
                response.success = False
                response.message = f"Profile {profile} is already running"
                response.active_nodes = list(self.active_profiles.keys())
                response.profile_state = STATE_RUNNING
                return response

        # Vérifier si déjà actif via une dépendance
        running_via = self._is_running_via_dependency(profile)
        if running_via:
            response.success = False
            response.message = f"Profile {profile} is already running (included by {running_via}). Cannot start independently."
            response.active_nodes = list(self.active_profiles.keys())
            response.profile_state = STATE_RUNNING
            return response

        # ===== VÉRIFICATION DES DÉPENDANCES =====
        required_deps = PROFILE_DEPENDENCIES.get(profile, [])
        missing_deps = []

        for dep in required_deps:
            # Vérifier si la dépendance est active (directement ou via un autre profil)
            if dep not in self.active_profiles:
                # La dépendance n'est pas active directement, vérifier si elle l'est via un autre profil
                if not self._is_running_via_dependency(dep):
                    missing_deps.append(dep)

        if missing_deps:
            response.success = False
            response.message = f"Cannot start {profile}: missing required dependencies: {missing_deps}. Start them first."
            response.active_nodes = list(self.active_profiles.keys())
            response.profile_state = STATE_ERROR
            self.get_logger().error(response.message)
            return response

        # Résoudre le chemin du launch file
        launch_file = PROFILE_LAUNCH_FILES[profile]
        launch_path = os.path.join(self.launch_dir, launch_file)

        if not os.path.exists(launch_path):
            response.success = False
            response.message = f"Launch file not found: {launch_path}"
            response.active_nodes = list(self.active_profiles.keys())
            response.profile_state = STATE_ERROR
            self.get_logger().error(response.message)
            return response

        # Créer et démarrer le ProfileProcess
        proc = ProfileProcess(profile, launch_path, self.get_logger())
        success = proc.start()

        if success:
            self.active_profiles[profile] = proc
            response.success = True
            response.message = f"Profile {profile} started successfully"
            response.active_nodes = list(self.active_profiles.keys())
            response.profile_state = STATE_RUNNING
        else:
            response.success = False
            response.message = f"Failed to start profile {profile}"
            response.active_nodes = list(self.active_profiles.keys())
            response.profile_state = STATE_ERROR

        return response

    def _handle_stop(self, profile: str, response):
        """Arrête un profil."""
        if profile not in self.active_profiles:
            response.success = False
            response.message = f"Profile {profile} is not running"
            response.active_nodes = list(self.active_profiles.keys())
            response.profile_state = STATE_STOPPED
            return response

        proc = self.active_profiles[profile]
        success = proc.stop()

        if success:
            del self.active_profiles[profile]
            response.success = True
            response.message = f"Profile {profile} stopped successfully"
            response.active_nodes = list(self.active_profiles.keys())
            response.profile_state = STATE_STOPPED
        else:
            response.success = False
            response.message = f"Failed to stop profile {profile}"
            response.active_nodes = list(self.active_profiles.keys())
            response.profile_state = STATE_ERROR

        return response

    def _handle_restart(self, profile: str, response):
        """Redémarre un profil."""
        # Arrêter si actif
        if profile in self.active_profiles:
            stop_resp = ManageProfile.Response()
            self._handle_stop(profile, stop_resp)
            if not stop_resp.success:
                return stop_resp

        # Démarrer
        return self._handle_start(profile, response)

    # =========================================================================
    # HELPERS
    # =========================================================================

    def _state_name(self, state: int) -> str:
        """Convertit un code d'état en nom lisible."""
        states = {
            STATE_STOPPED: "STOPPED",
            STATE_STARTING: "STARTING",
            STATE_RUNNING: "RUNNING",
            STATE_STOPPING: "STOPPING",
            STATE_ERROR: "ERROR",
        }
        return states.get(state, "UNKNOWN")

    def _is_running_via_dependency(self, profile: str) -> Optional[str]:
        """
        Vérifie si un profil tourne via une dépendance d'un autre profil actif.

        Returns:
            Nom du profil parent si trouvé, None sinon
        """
        for active_profile in self.active_profiles.keys():
            deps = PROFILE_DEPENDENCIES.get(active_profile, [])
            if profile in deps:
                return active_profile
        return None

    def _publish_diagnostics(self):
        """
        Publie l'état du manager et de tous les profils actifs sur /diagnostics.
        Appelé périodiquement par le timer.
        """
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        # ===== Diagnostic global du manager =====
        manager_status = DiagnosticStatus()
        manager_status.name = "qbo_bringup/manager"
        manager_status.hardware_id = "qbo_bringup_manager"

        # Vérifier si tous les profils actifs sont en état RUNNING
        all_running = True
        error_count = 0
        for proc in self.active_profiles.values():
            state = proc.get_state()
            if state == STATE_ERROR:
                error_count += 1
                all_running = False
            elif state != STATE_RUNNING:
                all_running = False

        if error_count > 0:
            manager_status.level = DiagnosticStatus.ERROR
            manager_status.message = f"{error_count} profile(s) in ERROR state"
        elif not all_running and len(self.active_profiles) > 0:
            manager_status.level = DiagnosticStatus.WARN
            manager_status.message = "Some profiles are transitioning"
        elif len(self.active_profiles) == 0:
            manager_status.level = DiagnosticStatus.OK
            manager_status.message = "No active profiles (IDLE)"
        else:
            manager_status.level = DiagnosticStatus.OK
            manager_status.message = f"{len(self.active_profiles)} profile(s) running normally"

        # Ajouter des infos détaillées
        manager_status.values.append(KeyValue(key="active_profiles_count", value=str(len(self.active_profiles))))
        manager_status.values.append(KeyValue(key="active_profiles", value=", ".join(self.active_profiles.keys()) if self.active_profiles else "none"))
        manager_status.values.append(KeyValue(key="available_profiles", value=", ".join(PROFILE_LAUNCH_FILES.keys())))

        msg.status.append(manager_status)

        # ===== Diagnostic pour chaque profil actif =====
        for profile_name, proc in self.active_profiles.items():
            profile_status = DiagnosticStatus()
            profile_status.name = f"qbo_bringup/profile/{profile_name}"
            profile_status.hardware_id = profile_name.lower()

            state = proc.get_state()
            state_name = self._state_name(state)

            # Définir le niveau de diagnostic selon l'état
            if state == STATE_RUNNING:
                profile_status.level = DiagnosticStatus.OK
                profile_status.message = f"Profile {profile_name} running"
            elif state == STATE_ERROR:
                profile_status.level = DiagnosticStatus.ERROR
                profile_status.message = f"Profile {profile_name} in ERROR state"
            elif state in [STATE_STARTING, STATE_STOPPING]:
                profile_status.level = DiagnosticStatus.WARN
                profile_status.message = f"Profile {profile_name} {state_name.lower()}"
            else:
                profile_status.level = DiagnosticStatus.STALE
                profile_status.message = f"Profile {profile_name} state: {state_name}"

            # Ajouter des infos détaillées
            profile_status.values.append(KeyValue(key="state", value=state_name))
            profile_status.values.append(KeyValue(key="launch_file", value=proc.launch_file_path))

            if proc.process:
                profile_status.values.append(KeyValue(key="pid", value=str(proc.process.pid)))
                poll_result = proc.process.poll()
                profile_status.values.append(KeyValue(key="alive", value="true" if poll_result is None else "false"))

            # Ajouter les dépendances
            deps = PROFILE_DEPENDENCIES.get(profile_name, [])
            if deps:
                profile_status.values.append(KeyValue(key="dependencies", value=", ".join(deps)))

            msg.status.append(profile_status)

        # Publier le message
        self.diagnostics_pub.publish(msg)

    def shutdown_all_profiles(self):
        """Arrête proprement tous les profils actifs (appelé au shutdown du node)."""
        self.get_logger().info("Shutting down all active profiles...")
        for profile_name in list(self.active_profiles.keys()):
            self.get_logger().info(f"  Stopping {profile_name}...")
            proc = self.active_profiles[profile_name]
            proc.stop()
        self.get_logger().info("All profiles stopped")


# =============================================================================
# MAIN
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = QboBringupManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down...")
    finally:
        node.shutdown_all_profiles()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
