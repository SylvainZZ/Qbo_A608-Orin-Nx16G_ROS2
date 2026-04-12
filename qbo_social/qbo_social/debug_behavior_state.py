#!/usr/bin/env python3
"""
QBO Social Dashboard — vue temps réel inspirée du SMACH Viewer (ROS 1).

Affiche en temps réel :
  - Mode système + profil actif + nœuds manquants
  - État batterie, thermique Orin NX
  - Présence de chaque nœud hardware (watchdog)
  - WorldState courant
  - Pipeline intent (raw → filtré : bloqué / dégradé / ok)
  - Log circulaire des 14 derniers events/intents/traces

Contrôles clavier interactifs :
  - 'g' : Set goal FIND_AND_GREET
  - 't' : Set goal avec timeout personnalisé
  - 'c' : Cancel goal actuel
  - 'h' : Afficher aide
  - 'q' : Quitter

Usage:
    ros2 run qbo_social debug_behavior_state            # dashboard (défaut)
    ros2 run qbo_social debug_behavior_state --scroll   # défilement classique
"""

import json
import re
import sys
import threading
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters
from qbo_msgs.msg import WorldState, SocialEvent, BehaviorIntent, DecisionTrace

# Gestion clavier multi-plateforme
try:
    import msvcrt  # Windows
    HAS_MSVCRT = True
except ImportError:
    import select  # Linux/Unix
    HAS_MSVCRT = False

# ── ANSI ──────────────────────────────────────────────────────────────────────
CLR = "\033[0m"
B   = "\033[1m"
DIM = "\033[2m"
RED = "\033[31m"
GRN = "\033[32m"
YEL = "\033[33m"
CYN = "\033[36m"
CLEAR_SCREEN = "\033[2J\033[H"

# ── Layout ────────────────────────────────────────────────────────────────────
W  = 115   # largeur totale (bordures incluses) 110
LW = 45    # largeur contenu colonne gauche 40
RW = 63    # largeur contenu colonne droite  → LW + RW + 7 = W

# ── Nœuds connus (hardware_id → description) ─────────────────────────────────
KNOWN_NODES = {
    "System":        "qbo_arduqbo principal",
    "Qboard_1":      "base + capteurs",
    "Qboard_3":      "batterie",
    "Qboard_4":      "IMU",
    "Qboard_5":      "nez + bouche",
    "LCD":           "ecran LCD",
    "qbo_dynamixel": "head_pan / head_tilt",
    "orin-nx-16g":   "Orin NX (diag syst.)",
    "qbo_vision":    "face_tracker",
}


# ── Helpers ───────────────────────────────────────────────────────────────────

def _strip(text: str) -> str:
    """Retire les codes ANSI pour mesurer la largeur visible."""
    return re.sub(r'\033\[[0-9;]*m', '', text)


def _pad(text: str, width: int) -> str:
    """Complète jusqu'à width caractères visibles."""
    return text + ' ' * max(0, width - len(_strip(text)))


def _bar(ratio: float, width: int = 7,
         warn: float = 0.65, crit: float = 0.85) -> str:
    """Barre de progression colorée (0.0–1.0)."""
    ratio  = max(0.0, min(1.0, ratio))
    filled = round(ratio * width)
    color  = RED if ratio >= crit else YEL if ratio >= warn else GRN
    return f"{color}{'█' * filled}{DIM}{'░' * (width - filled)}{CLR}"


def _vbar(voltage: float, lo: float = 11.0,
          hi: float = 14.0, width: int = 7) -> str:
    """Barre batterie : plein = bien."""
    ratio  = max(0.0, min(1.0, (voltage - lo) / (hi - lo))) if hi > lo else 0.0
    filled = round(ratio * width)
    color  = RED if ratio < 0.2 else YEL if ratio < 0.4 else GRN
    return f"{color}{'█' * filled}{DIM}{'░' * (width - filled)}{CLR}"


def _fv(raw: str) -> float:
    """Parse flottant depuis une chaîne type '12.6' ou '7.1 W'."""
    try:
        return float(raw.split()[0].replace(',', '.').strip())
    except Exception:
        return 0.0


def _hline(char: str = '─', w: int = W - 4) -> str:
    return f"{DIM}{char * w}{CLR}"


# =============================================================================
class QboDashboard(Node):
# =============================================================================

    def __init__(self, scroll_mode: bool = False):
        super().__init__('qbo_dashboard')
        self._scroll = scroll_mode

        # ── Contrôle interactif ──────────────────────────────────────────
        self._running = True
        self._status_msg = f"{DIM}En attente de commande...{CLR}"  # Message de statut pour les commandes

        # ── Système ──────────────────────────────────────────────────────
        self._mode    = "UNKNOWN"
        self._profile = "?"
        self._missing: list[str] = []

        # ── Batterie ──────────────────────────────────────────────────────
        self._battery_v   = 0.0
        self._runtime_min = -1.0
        self._charging    = False

        # ── Thermique ────────────────────────────────────────────────────
        self._cpu_temp = 0.0
        self._gpu_temp = 0.0
        self._ram_pct  = 0.0
        self._cpu_pct  = 0.0
        self._power_w  = 0.0

        # ── Nœuds ────────────────────────────────────────────────────────
        self._nodes: dict[str, bool] = {}

        # ── WorldState ────────────────────────────────────────────────────
        self._face_present = False
        self._face_stable  = False
        self._person_name  = ""
        self._engagement   = 0.0
        self._tracking_on  = False
        self._conv_active  = False
        self._ws_health    = ""

        # ── Pipeline intent ───────────────────────────────────────────────
        self._raw_intent      = ""
        self._filtered_intent = ""
        self._intent_reason   = ""

        # ── Compteurs ────────────────────────────────────────────────────
        self._event_count   = 0
        self._intent_count  = 0
        self._blocked_count = 0

        # ── Log circulaire ────────────────────────────────────────────────
        self._log: deque[str] = deque(maxlen=14)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(WorldState,      '/qbo_social/world_state',    self._on_world,  10)
        self.create_subscription(SocialEvent,     '/qbo_social/events',         self._on_event,  10)
        self.create_subscription(BehaviorIntent,  '/qbo_social/intent',         self._on_intent, 10)
        self.create_subscription(BehaviorIntent,  '/qbo_social/intent_raw',     self._on_raw,    10)
        self.create_subscription(DecisionTrace,   '/qbo_social/decision_trace', self._on_trace,  10)
        self.create_subscription(DiagnosticArray, '/diagnostics',               self._on_diag,   10)

        # ── Publisher pour les commandes de goal ──────────────────────────
        self.pub_event = self.create_publisher(
            SocialEvent,
            '/qbo_social/events',
            10
        )

        # ── Récupération du profil actif (paramètre distant) ──────────────
        self._profile_client = self.create_client(
            GetParameters, '/qbo_system_mode_manager/get_parameters')
        self._profile_fetched = False
        self.create_timer(2.0, self._try_fetch_profile)

        # ── Timer de rafraîchissement (dashboard uniquement) ──────────────
        if not scroll_mode:
            self.create_timer(1.0, self._draw)
            # Démarrer le thread d'écoute clavier
            self._keyboard_thread = threading.Thread(target=self._keyboard_listener, daemon=True)
            self._keyboard_thread.start()
        else:
            self._print_scroll_header()

    # =========================================================================
    # RÉCUPÉRATION PROFIL (one-shot)
    # =========================================================================

    def _try_fetch_profile(self):
        if self._profile_fetched:
            return
        if not self._profile_client.service_is_ready():
            return
        req = GetParameters.Request()
        req.names = ['active_profile']
        future = self._profile_client.call_async(req)
        future.add_done_callback(self._on_profile_response)
        self._profile_fetched = True

    def _on_profile_response(self, future):
        try:
            result = future.result()
            val = result.values[0].string_value if result.values else ''
            if val:
                self._profile = val
        except Exception:
            self._profile_fetched = False  # réessayer

    # =========================================================================
    # CONTRÔLE CLAVIER INTERACTIF
    # =========================================================================

    def _keyboard_listener(self):
        """Thread d'écoute clavier pour commandes interactives."""
        while self._running:
            try:
                key = self._get_key()
                if key:
                    self._handle_key(key)
            except Exception:
                pass

    def _get_key(self) -> str:
        """Récupère une touche pressée (multi-plateforme)."""
        try:
            if HAS_MSVCRT:  # Windows
                if msvcrt.kbhit():
                    return msvcrt.getch().decode('utf-8', errors='ignore').lower()
            else:  # Linux/Unix
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    return sys.stdin.read(1).lower()
        except Exception:
            pass
        return ''

    def _handle_key(self, key: str):
        """Gère les touches pressées."""
        if key == 'q':
            self._running = False
            self._status_msg = f"{RED}Arrêt demandé...{CLR}"

        elif key == 'g':
            self._set_goal("FIND_AND_GREET", 60.0)
            self._status_msg = f"{GRN}✓ Goal FIND_AND_GREET défini (timeout: 60s){CLR}"

        elif key == 't':
            self._set_goal("FIND_AND_GREET", 30.0)
            self._status_msg = f"{GRN}✓ Goal FIND_AND_GREET défini (timeout: 30s - test court){CLR}"

        elif key == 'c':
            self._cancel_goal()
            self._status_msg = f"{YEL}✓ Goal annulé{CLR}"

    def _set_goal(self, goal_name: str, timeout: float):
        """Publie un event SET_GOAL."""
        msg = SocialEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'dashboard'
        msg.stamp = msg.header.stamp
        msg.event_type = 'SET_GOAL'
        msg.source = 'dashboard_keyboard'
        msg.payload_json = json.dumps({
            "goal": goal_name,
            "timeout": timeout
        })
        self.pub_event.publish(msg)

    def _cancel_goal(self):
        """Publie un event CANCEL_GOAL."""
        msg = SocialEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'dashboard'
        msg.stamp = msg.header.stamp
        msg.event_type = 'CANCEL_GOAL'
        msg.source = 'dashboard_keyboard'
        msg.payload_json = '{}'
        self.pub_event.publish(msg)

    # =========================================================================
    # CALLBACKS
    # =========================================================================
    def safe_level(self, level):
        if isinstance(level, int):
            return level
        if isinstance(level, (bytes, bytearray)):
            return level[0]
        try:
            return int(level)
        except:
            return 0

    def _on_diag(self, msg: DiagnosticArray):
        for status in msg.status:
            vals = {kv.key: kv.value for kv in status.values}
            name = status.name

            # Mise à jour de l'état des nœuds hardware basé sur hardware_id
            if status.hardware_id:
                # Considérer le nœud présent si level < 2 (OK ou WARNING)
                self._nodes[status.hardware_id] = (self.safe_level(status.level) < 2)

            if 'Battery Status' in name:
                if 'Voltage'           in vals: self._battery_v   = _fv(vals['Voltage'])
                if 'Charge Mode'       in vals: self._charging    = vals['Charge Mode'].strip() in ('1', '2')
                if 'Estimated Runtime' in vals: self._runtime_min = _fv(vals['Estimated Runtime'])
            elif 'A608 Temp' in name:
                if 'CPU \u00b0C' in vals: self._cpu_temp = _fv(vals['CPU \u00b0C'])
                if 'GPU \u00b0C' in vals: self._gpu_temp = _fv(vals['GPU \u00b0C'])
            elif 'A608 Fan' in name:
                if 'RAM %' in vals: self._ram_pct = _fv(vals['RAM %'])
                if 'CPU %' in vals: self._cpu_pct = _fv(vals['CPU %'])
            elif 'A608 Power' in name:
                if 'VDD_IN W' in vals: self._power_w = _fv(vals['VDD_IN W'])

    def _on_world(self, msg: WorldState):
        self._face_present = msg.face_present
        self._face_stable  = msg.face_stable
        self._person_name  = msg.focus_person_name or ''
        self._engagement   = msg.engagement_level
        self._tracking_on  = msg.tracking_enabled
        self._conv_active  = msg.conversation_active
        self._ws_health    = msg.health_state or ''

    def _on_event(self, msg: SocialEvent):
        self._event_count += 1
        ts = datetime.now().strftime('%H:%M:%S')
        try:
            pl = json.loads(msg.payload_json) if msg.payload_json else {}
        except Exception:
            pl = {}

        et = msg.event_type

        if et == 'SYSTEM_MODE_CHANGED':
            new_mode = pl.get('mode', '?')
            rs       = pl.get('robot_state', {})
            self._missing = rs.get('missing_required_nodes', [])
            if rs.get('nodes_present'):
                self._nodes.update(rs['nodes_present'])
            old, self._mode = self._mode, new_mode
            line = f"{ts}  MODE CHANGE  {DIM}{old}{CLR} -> {B}{new_mode}{CLR}"

        elif et == 'NODE_MISSING':
            hw  = pl.get('hardware', '?')
            ela = pl.get('elapsed_s', 0.0)
            self._nodes[hw] = False
            line = f"{ts}  NODE MISSING   {B}{hw}{CLR}  absent depuis {ela:.0f}s"

        elif et == 'NODE_RECOVERED':
            hw = pl.get('hardware', '?')
            self._nodes[hw] = True
            line = f"{ts}  {GRN}NODE RECOV{CLR}     {B}{hw}{CLR}"

        elif et == 'DIAGNOSTIC':
            key    = pl.get('key', '?')
            lvl    = pl.get('level', 0)
            active = pl.get('active', False)
            msg_t  = pl.get('message', '')
            if active:
                sev = f"{RED}[ERROR]{CLR}" if lvl >= 2 else f"{YEL}[WARN]{CLR}"
                line = f"{ts}  DIAG {sev}  {DIM}{key}{CLR}  {msg_t}"
            else:
                line = f"{ts}  {GRN}DIAG RESOLVED{CLR}  {DIM}{key}{CLR}"

        elif 'FACE' in et:
            extra = ''
            if pl.get('distance'): extra += f"  dist={pl['distance']:.2f}m"
            if msg.person_name:    extra += f"  {CYN}{msg.person_name}{CLR}"
            line = f"{ts}  {CYN}{et:<24}{CLR}{extra}"

        elif et == 'START_PROFILE':
            self._profile = pl.get('profile', self._profile)
            missing_n     = pl.get('missing_nodes', [])
            line = f"{ts}  {YEL}START_PROFILE  {self._profile}{CLR}  manquants={missing_n}"

        else:
            extra = f"  {CYN}{msg.person_name}{CLR}" if msg.person_name else ''
            line  = f"{ts}  {DIM}{et:<24}{CLR}  {DIM}{msg.source}{CLR}{extra}"

        self._log.append(line)
        if self._scroll:
            print(line)

    def _on_intent(self, msg: BehaviorIntent):
        self._intent_count   += 1
        self._filtered_intent = msg.intent_type
        self._intent_reason   = msg.reason or ''
        ts     = datetime.now().strftime('%H:%M:%S')
        person = f"  -> {CYN}{msg.target_person_name}{CLR}" if msg.target_person_name else ''
        line   = f"{ts}  {GRN}INTENT  {B}{msg.intent_type:<26}{CLR}  {DIM}{self._intent_reason}{CLR}{person}"
        self._log.append(line)
        if self._scroll:
            print(line)

    def _on_raw(self, msg: BehaviorIntent):
        self._raw_intent = msg.intent_type

    def _on_trace(self, msg: DecisionTrace):
        if msg.suppressed_by_cooldown:
            self._blocked_count += 1
        ts      = datetime.now().strftime('%H:%M:%S')
        rel_b   = _bar(msg.relevance_score, width=6)
        sup_str = f"{RED}[COOLDOWN]{CLR}" if msg.suppressed_by_cooldown else f"{GRN}[OK]{CLR}"
        line    = (f"{ts}  TRACE  trigger={DIM}{msg.triggering_event!r}{CLR}"
                   f"  -> {B}{msg.chosen_intent or '—'}{CLR}"
                   f"  rel={rel_b}  {sup_str}")
        self._log.append(line)
        if self._scroll:
            print(line)

    # =========================================================================
    # DASHBOARD
    # =========================================================================

    def _draw(self):
        ts    = datetime.now().strftime('%d/%m/%Y  %H:%M:%S')
        title = f"QBO SOCIAL DASHBOARD  [{ts}]"
        pad   = (W - 2 - len(title)) // 2
        title_line = ' ' * pad + title + ' ' * (W - 2 - pad - len(title))

        out = [CLEAR_SCREEN]
        out.append('╔' + '═' * (W - 2) + '╗')
        out.append(f'║{B}{title_line}{CLR}║')
        out.append('╠' + '═' * (LW + 2) + '╦' + '═' * (RW + 2) + '╣')

        left  = self._build_left()
        right = self._build_right()
        for i in range(max(len(left), len(right))):
            l = left[i]  if i < len(left)  else ''
            r = right[i] if i < len(right) else ''
            out.append(f'║ {_pad(l, LW)} ║ {_pad(r, RW)} ║')

        out.append('╠' + '═' * (W - 2) + '╣')
        out.append('║ ' + _pad(self._world_line(),  W - 4) + ' ║')
        out.append('║ ' + _pad(self._intent_line(), W - 4) + ' ║')

        out.append('╠' + '═' * (W - 2) + '╣')

        # Ligne de contrôle interactif (toujours visible)
        help_line = (f"{B}CONTROLES{CLR}  "
                    f"{CYN}[g]{CLR}=Goal 60s  "
                    f"{CYN}[t]{CLR}=Goal 30s  "
                    f"{CYN}[c]{CLR}=Cancel goal  "
                    f"{CYN}[q]{CLR}=Quitter")
        out.append('║ ' + _pad(help_line, W - 4) + ' ║')

        # Message de statut (toujours visible)
        if self._status_msg:
            out.append('║ ' + _pad(f"  {self._status_msg}", W - 4) + ' ║')

        out.append('╠' + '═' * (W - 2) + '╣')
        hdr = (f"{B}EVENTS{CLR}  "
               f"{DIM}#{self._event_count} events  "
               f"#{self._intent_count} intents  "
               f"#{self._blocked_count} bloques{CLR}")
        out.append('║ ' + _pad(hdr, W - 4) + ' ║')
        out.append('║ ' + _hline() + ' ║')

        for ev in self._log:
            out.append('║  ' + _pad(ev, W - 5) + '║')
        for _ in range(14 - len(self._log)):
            out.append('║' + ' ' * (W - 2) + '║')

        out.append('╚' + '═' * (W - 2) + '╝')
        print('\n'.join(out), end='', flush=True)

    # ── Colonne gauche ────────────────────────────────────────────────────────

    def _build_left(self) -> list[str]:
        ln = []

        mode_c = (GRN if self._mode == 'NORMAL' else
                  RED if 'CRITICAL' in self._mode or 'DEGRADED' in self._mode else YEL)
        ln.append(f'{B}SYSTEME (SBE){CLR}')
        ln.append(f'  MODE    {mode_c}● {self._mode}{CLR}')
        ln.append(f'  PROFIL  {CYN}{self._profile}{CLR}')
        if self._missing:
            ln.append(f'  {RED}! manquants : {", ".join(self._missing[:3])}{CLR}')
        else:
            ln.append(f'  {GRN}v profil complet{CLR}')
        ln.append('')

        ln.append(f'{B}BATTERIE{CLR}')
        v_str = f'{self._battery_v:.2f} V' if self._battery_v > 0 else 'N/A'
        ln.append(f'  {_vbar(self._battery_v)} {B}{v_str}{CLR}')
        rt = f'{CYN}{self._runtime_min:.0f} min{CLR}' if self._runtime_min > 0 else f'{DIM}---{CLR}'
        ln.append(f'  Runtime  {rt}')
        chg = f'{GRN}En charge{CLR}' if self._charging else f'{DIM}Sur batterie{CLR}'
        ln.append(f'  Charge   {chg}')
        ln.append('')

        ln.append(f'{B}THERMIQUE  (Orin NX){CLR}')
        ln.append(f'  CPU {self._cpu_temp:5.1f}C  {_bar(self._cpu_temp/80)}'
                  f'  GPU {self._gpu_temp:5.1f}C  {_bar(self._gpu_temp/80)}')
        ln.append(f'  RAM {self._ram_pct:5.1f}%  {_bar(self._ram_pct/100)}'
                  f'  Pwr  {self._power_w:5.1f} W')
        return ln

    # ── Colonne droite ────────────────────────────────────────────────────────

    def _build_right(self) -> list[str]:
        ln = []
        ln.append(f'{B}NOEUDS HARDWARE{CLR}')
        ln.append(f'  {DIM}{"hardware_id":<18}  {"etat":<10}  description{CLR}')
        ln.append(f'  {"─" * 56}')

        # Nœuds connus + découverts dynamiquement
        all_nodes = {k: self._nodes.get(k) for k in KNOWN_NODES}
        for hw, state in self._nodes.items():
            if hw not in all_nodes:
                all_nodes[hw] = state

        for hw, present in all_nodes.items():
            desc = KNOWN_NODES.get(hw, 'decouvert dynamiquement')
            if present is True:
                icon, st = f'{GRN}v{CLR}', f'{GRN}OK{CLR}'
            elif present is False:
                icon, st = f'{RED}x{CLR}', f'{RED}ERROR{CLR}'
            else:
                icon, st = f'{DIM}?{CLR}', f'{DIM}attente{CLR}'
            ln.append(f'  {icon} {_pad(hw, 17)}  {_pad(st, 12)}  {DIM}{desc}{CLR}')

        return ln

    # ── Lignes full-width ─────────────────────────────────────────────────────

    def _world_line(self) -> str:
        face_s = (f'{GRN}STABLE{CLR}'   if self._face_stable  else
                  f'{YEL}DETECTE{CLR}'  if self._face_present else
                  f'{DIM}aucun{CLR}')
        person = f'{CYN}{self._person_name}{CLR}' if self._person_name else f'{DIM}---{CLR}'
        eng_b  = _bar(self._engagement, warn=0.4, crit=0.8)
        trk    = f'{GRN}ON{CLR}'  if self._tracking_on else f'{DIM}OFF{CLR}'
        conv   = f'{GRN}OUI{CLR}' if self._conv_active  else f'{DIM}non{CLR}'
        health = f'  sante={DIM}{self._ws_health}{CLR}' if self._ws_health else ''
        return (f'{B}WORLD{CLR}'
                f'  Visage {face_s}'
                f'  Personne {person}'
                f'  Engagement {eng_b} {self._engagement:.2f}'
                f'  Tracking {trk}'
                f'  Conv {conv}'
                f'{health}')

    def _intent_line(self) -> str:
        raw = self._raw_intent      or '---'
        fil = self._filtered_intent or '---'
        if raw == '---':
            flow = f'{DIM}(aucun intent recus){CLR}'
        elif raw == fil:
            flow = f'{GRN}OK{CLR}  filtered={B}{fil}{CLR}'
        elif fil == '---':
            self._blocked_count  # already counted in trace
            flow = f'{RED}BLOQUE{CLR}'
        else:
            flow = f'{YEL}DEGRADE{CLR}  filtered={B}{fil}{CLR}'
        reason = f'  {DIM}{self._intent_reason}{CLR}' if self._intent_reason else ''
        return (f'{B}INTENT{CLR}'
                f'  raw={CYN}{raw}{CLR}'
                f'  ->  {flow}'
                f'{reason}')

    # ── Scroll mode header ───────────────────────────────────────────────────

    def _print_scroll_header(self):
        print('\n' + '=' * W)
        label = 'QBO SOCIAL DASHBOARD -- MODE DEFILEMENT'
        pad   = (W - len(label)) // 2
        print(' ' * pad + label)
        print('=' * W)
        print('  Events  |  Intents  |  Traces  |  Diagnostics  |  Nodes')
        print('=' * W + '\n')


# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    scroll = '--scroll' in sys.argv
    node   = QboDashboard(scroll_mode=scroll)

    try:
        # Mode dashboard : boucle personnalisée pour détecter arrêt par touche 'q'
        if not scroll:
            while rclpy.ok() and node._running:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node._running = False  # Arrêter le thread clavier
        if not scroll:
            print(CLEAR_SCREEN)
        print(f'\n{"=" * W}')
        print(f'  BILAN  |  Events: {node._event_count}  '
              f'Intents: {node._intent_count}  '
              f'Bloques: {node._blocked_count}')
        print(f'{"=" * W}\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
