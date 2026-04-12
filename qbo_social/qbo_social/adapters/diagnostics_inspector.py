from diagnostic_msgs.msg import DiagnosticArray
import time


# =============================================================================
# DIAGNOSTICS INSPECTOR
# =============================================================================
# Surveille la présence des nœuds ROS sur /diagnostics.
# Chaque nœud connu qui arrête de publier pendant plus de `timeout_s` secondes
# déclenche un event NODE_MISSING.  Dès que le nœud revient, un event
# NODE_RECOVERED est publié.
#
# L'inspecteur écrit aussi dans robot_state["nodes_present"] un dict :
#   {hardware_id: True/False}
# disponible pour les règles de system_mode_manager.py.
#
# Paramètres (passés au constructeur) :
#   timeout_s  : secondes sans message avant NODE_MISSING  (défaut : 10.0)
#   check_hz   : fréquence de vérification watchdog        (défaut : 2.0 Hz)
# =============================================================================


class DiagnosticsInspector:

    def __init__(self, node, timeout_s: float = 10.0, check_hz: float = 2.0):
        self.node = node
        self.timeout_s = timeout_s

        # {hardware_id: last_seen_timestamp}
        self._last_seen: dict[str, float] = {}

        # {hardware_id: bool}  — False = nœud considéré absent
        self._present: dict[str, bool] = {}

        self.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self._callback,
            10,
        )

        self.node.create_timer(1.0 / check_hz, self._watchdog)

        self.node.get_logger().info(
            f"[DiagnosticsInspector] démarré — timeout={timeout_s}s, "
            f"check={check_hz}Hz"
        )

    # =========================================================================
    # CALLBACK /diagnostics
    # =========================================================================

    def _callback(self, msg: DiagnosticArray):
        now = time.time()

        for status in msg.status:
            hardware = status.hardware_id

            # Ignorer les diagnostics sans hardware_id valide
            if not hardware or hardware.strip() == "":
                continue

            was_present = self._present.get(hardware)  # None = jamais vu

            self._last_seen[hardware] = now

            if was_present is not False:
                # Première apparition OU déjà présent → pas de changement à signaler
                self._present[hardware] = True
                self._sync_robot_state()
                continue

            # -- Nœud qui revient après une absence --
            self._present[hardware] = True
            self._sync_robot_state()

            self.node.get_logger().info(
                f"[DiagnosticsInspector] NODE_RECOVERED : {hardware!r}"
            )
            self._publish_event("NODE_RECOVERED", hardware, present=True)

    # =========================================================================
    # WATCHDOG TIMER
    # =========================================================================

    def _watchdog(self):
        now = time.time()

        for hardware, last in list(self._last_seen.items()):
            elapsed = now - last

            if elapsed > self.timeout_s:
                if self._present.get(hardware) is True:
                    # Transition présent → absent
                    self._present[hardware] = False
                    self._sync_robot_state()

                    self.node.get_logger().warn(
                        f"[DiagnosticsInspector] NODE_MISSING : {hardware!r} "
                        f"— absent depuis {elapsed:.1f}s"
                    )
                    self._publish_event("NODE_MISSING", hardware, present=False,
                                        elapsed_s=elapsed)

    # =========================================================================
    # SYNCHRONISATION robot_state
    # =========================================================================

    def _sync_robot_state(self):
        """Met à jour robot_state['nodes_present'] sur le nœud parent."""
        diag_adapter = getattr(self.node, "diagnostics_adapter", None)
        if diag_adapter is not None:
            diag_adapter.robot_state["nodes_present"] = dict(self._present)

    # =========================================================================
    # PUBLICATION ÉVÉNEMENT
    # =========================================================================

    def _publish_event(self, event_type: str, hardware: str,
                       present: bool, elapsed_s: float = 0.0):
        payload = {
            "hardware": hardware,
            "present": present,
            "elapsed_s": round(elapsed_s, 1),
            "known_nodes": list(self._present.keys()),
        }

        msg = self.node._create_event_msg(
            event_type=event_type,
            source="diagnostics_inspector",
            payload=payload,
        )
        self.node.pub_event.publish(msg)
