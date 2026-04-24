#!/usr/bin/env python3
"""
QBO Social Dashboard — Version Web

Dashboard temps réel accessible via navigateur web sur le port 8080.
Utilise aiohttp pour le serveur web et websockets pour les mises à jour temps réel.

Structure du projet:
    app_qbo.py         - Serveur backend Python/ROS2
    static/
      ├── index.html   - Page HTML principale
      ├── style.css    - Styles CSS
      └── dashboard.js - Logique JavaScript/WebSocket

Usage:
    python3 app_qbo.py
    Puis ouvrir http://localhost:8080 dans un navigateur
"""

import json
import asyncio
from datetime import datetime
from collections import deque
from typing import Dict, Set

from aiohttp import web
import aiohttp

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters
from qbo_msgs.msg import WorldState, SocialEvent, BehaviorIntent, DecisionTrace


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


# =============================================================================
class QboDashboardNode(Node):
    """Nœud ROS2 qui collecte les données et les diffuse via websocket."""
# =============================================================================

    def __init__(self):
        super().__init__('qbo_dashboard_web')

        # Référence à la boucle asyncio (sera définie dans main())
        self.asyncio_loop = None

        # Flag pour indiquer que le nœud est en cours d'arrêt
        self._shutting_down = False

        # WebSocket clients connectés
        self.websocket_clients: Set[web.WebSocketResponse] = set()

        # ── Données système ──────────────────────────────────────────────
        self.state = {
            # Système
            'mode': 'UNKNOWN',
            'profile': '?',
            'missing_nodes': [],

            # Batterie
            'battery_v': 0.0,
            'runtime_min': -1.0,
            'charging': False,

            # Thermique
            'cpu_temp': 0.0,
            'gpu_temp': 0.0,
            'ram_pct': 0.0,
            'cpu_pct': 0.0,
            'power_w': 0.0,

            # Nœuds hardware
            'nodes': {},

            # WorldState
            'face_present': False,
            'face_stable': False,
            'person_name': '',
            'engagement': 0.0,
            'tracking_on': False,
            'conv_active': False,
            'ws_health': '',

            # Pipeline intent
            'raw_intent': '',
            'filtered_intent': '',
            'intent_reason': '',

            # Compteurs
            'event_count': 0,
            'intent_count': 0,
            'blocked_count': 0,

            # Log circulaire (14 dernières entrées)
            'log': deque(maxlen=14),
        }

        # ── Position 3D du visage (dernière connue) ──────────────────────
        self.face_position = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'distance': 0.0,
            'timestamp': None,
        }

        # ── Tracking de session (reconstitué depuis événements) ──────────
        self.session = {
            'active': False,           # Session en cours ?
            'person_name': '',         # Nom de la personne
            'started_at': None,        # Timestamp début (float)
            'last_recognition': None,  # Dernière fois vue (float)
            'face_lost_at': None,      # Quand le visage a été perdu
            'status': 'idle',          # idle | greeted | lost | timeout
        }

        # ── Subscriptions ROS2 ────────────────────────────────────────────
        self.create_subscription(WorldState, '/qbo_social/world_state', self._on_world, 10)
        self.create_subscription(SocialEvent, '/qbo_social/events', self._on_event, 10)
        self.create_subscription(BehaviorIntent, '/qbo_social/intent', self._on_intent, 10)
        self.create_subscription(BehaviorIntent, '/qbo_social/intent_raw', self._on_raw, 10)
        self.create_subscription(DecisionTrace, '/qbo_social/decision_trace', self._on_trace, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._on_diag, 10)

        # ── Publisher pour les commandes ──────────────────────────────────
        self.pub_event = self.create_publisher(SocialEvent, '/qbo_social/events', 10)
        self.pub_intent = self.create_publisher(BehaviorIntent, '/qbo_social/intent', 10)

        # ── Récupération du profil actif ─────────────────────────────────
        self._profile_client = self.create_client(
            GetParameters, '/qbo_system_mode_manager/get_parameters')
        self._profile_fetched = False
        self.create_timer(2.0, self._try_fetch_profile)

        # Timer pour broadcast périodique (toutes les secondes)
        self.create_timer(1.0, self._broadcast_state)

    # =========================================================================
    # GESTION WEBSOCKET
    # =========================================================================

    async def add_websocket_client(self, ws: web.WebSocketResponse):
        """Ajoute un client websocket et lui envoie l'état initial."""
        self.websocket_clients.add(ws)
        if not self._shutting_down:
            self.get_logger().info(f'Client websocket connecté. Total: {len(self.websocket_clients)}')
        # Envoyer l'état complet immédiatement
        await self._send_to_client(ws, self._get_full_state())

    async def remove_websocket_client(self, ws: web.WebSocketResponse):
        """Retire un client websocket."""
        self.websocket_clients.discard(ws)
        if not self._shutting_down:
            self.get_logger().info(f'Client websocket déconnecté. Total: {len(self.websocket_clients)}')

    async def close_all_websockets(self):
        """Ferme proprement tous les websockets."""
        print(f'⏹️  Fermeture de {len(self.websocket_clients)} client(s) websocket...')
        for ws in list(self.websocket_clients):
            try:
                await ws.close()
            except Exception:
                pass
        self.websocket_clients.clear()

    def _broadcast_state(self):
        """Diffuse l'état actuel à tous les clients (appelé par timer ROS2)."""
        if not self._shutting_down and self.websocket_clients and self.asyncio_loop:
            asyncio.run_coroutine_threadsafe(self._async_broadcast(), self.asyncio_loop)

    async def _async_broadcast(self):
        """Envoie l'état à tous les clients websocket."""
        if self._shutting_down:
            return

        data = self._get_full_state()
        dead_clients = set()

        for ws in self.websocket_clients:
            try:
                await self._send_to_client(ws, data)
            except Exception as e:
                if not self._shutting_down:
                    self.get_logger().warn(f'Erreur envoi websocket: {e}')
                dead_clients.add(ws)

        # Nettoyer les clients morts
        for ws in dead_clients:
            self.websocket_clients.discard(ws)

    async def _send_to_client(self, ws: web.WebSocketResponse, data: dict):
        """Envoie des données JSON à un client websocket."""
        await ws.send_json(data)

    def _get_full_state(self) -> dict:
        """Retourne l'état complet en format JSON-friendly."""
        return {
            'timestamp': datetime.now().isoformat(),
            'mode': self.state['mode'],
            'profile': self.state['profile'],
            'missing_nodes': self.state['missing_nodes'],
            'battery': {
                'voltage': self.state['battery_v'],
                'runtime_min': self.state['runtime_min'],
                'charging': self.state['charging'],
            },
            'thermal': {
                'cpu_temp': self.state['cpu_temp'],
                'gpu_temp': self.state['gpu_temp'],
                'ram_pct': self.state['ram_pct'],
                'cpu_pct': self.state['cpu_pct'],
                'power_w': self.state['power_w'],
            },
            'nodes': self.state['nodes'],
            'world': {
                'face_present': self.state['face_present'],
                'face_stable': self.state['face_stable'],
                'person_name': self.state['person_name'],
                'engagement': self.state['engagement'],
                'tracking_on': self.state['tracking_on'],
                'conv_active': self.state['conv_active'],
                'health': self.state['ws_health'],
            },
            'intent': {
                'raw': self.state['raw_intent'],
                'filtered': self.state['filtered_intent'],
                'reason': self.state['intent_reason'],
            },
            'counters': {
                'events': self.state['event_count'],
                'intents': self.state['intent_count'],
                'blocked': self.state['blocked_count'],
            },
            'session': {
                'active': self.session['active'],
                'person_name': self.session['person_name'],
                'started_at': self.session['started_at'],
                'last_recognition': self.session['last_recognition'],
                'face_lost_at': self.session['face_lost_at'],
                'status': self.session['status'],
            },
            'face_position': {
                'x': self.face_position['x'],
                'y': self.face_position['y'],
                'z': self.face_position['z'],
                'distance': self.face_position['distance'],
                'timestamp': self.face_position['timestamp'],
            },
            'log': list(self.state['log']),
        }

    # =========================================================================
    # COMMANDES (appelées depuis websocket)
    # =========================================================================

    def set_goal(self, goal_name: str, timeout: float):
        """Publie un event SET_GOAL."""
        msg = SocialEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'dashboard_web'
        msg.stamp = msg.header.stamp
        msg.event_type = 'SET_GOAL'
        msg.source = 'dashboard_web'
        msg.payload_json = json.dumps({
            "goal": goal_name,
            "timeout": timeout
        })
        self.pub_event.publish(msg)
        if not self._shutting_down:
            self.get_logger().info(f'Goal défini: {goal_name} (timeout: {timeout}s)')

    def cancel_goal(self):
        """Publie un event CANCEL_GOAL."""
        msg = SocialEvent()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'dashboard_web'
        msg.stamp = msg.header.stamp
        msg.event_type = 'CANCEL_GOAL'
        msg.source = 'dashboard_web'
        msg.payload_json = '{}'
        self.pub_event.publish(msg)
        if not self._shutting_down:
            self.get_logger().info('Goal annulé')

    def send_intent(self, intent_type: str, payload: dict):
        """Publie un BehaviorIntent depuis le dashboard web."""
        try:
            self.get_logger().info(f"📤 Création intent: {intent_type}")

            msg = BehaviorIntent()
            self.get_logger().info(f"  → BehaviorIntent() créé")

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.stamp = msg.header.stamp
            self.get_logger().info(f"  → Timestamp défini")

            msg.intent_type = intent_type
            msg.reason = "web_dashboard"
            msg.priority = 1.0
            self.get_logger().info(f"  → Champs basiques définis")

            # Gérer les champs optionnels du payload
            msg.target_person_name = payload.get('name', '')
            msg.target_person_id = payload.get('person_id', '')
            self.get_logger().info(f"  → Champs optionnels définis")

            # Encoder le reste du payload en JSON
            msg.payload_json = json.dumps(payload)
            self.get_logger().info(f"  → Payload JSON encodé: {msg.payload_json}")

            self.get_logger().info(f"📤 Publication vers /qbo_social/intent: {intent_type} | {payload}")
            self.pub_intent.publish(msg)
            self.get_logger().info(f"✅ Intent publié: {intent_type}")

        except Exception as e:
            self.get_logger().error(f"❌ ERREUR dans send_intent(): {type(e).__name__}: {e}")
            import traceback
            self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
            raise

    # =========================================================================
    # CALLBACKS ROS2 (identiques au code original)
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
                self.state['profile'] = val
        except Exception:
            self._profile_fetched = False

    @staticmethod
    def safe_level(level):
        if isinstance(level, int):
            return level
        if isinstance(level, (bytes, bytearray)):
            return level[0]
        try:
            return int(level)
        except:
            return 0

    @staticmethod
    def _fv(raw: str) -> float:
        """Parse flottant depuis une chaîne."""
        try:
            return float(raw.split()[0].replace(',', '.').strip())
        except Exception:
            return 0.0

    def _on_diag(self, msg: DiagnosticArray):
        for status in msg.status:
            vals = {kv.key: kv.value for kv in status.values}
            name = status.name

            # État des nœuds hardware
            if status.hardware_id:
                self.state['nodes'][status.hardware_id] = (self.safe_level(status.level) < 2)

            if 'Battery Status' in name:
                if 'Voltage' in vals:
                    self.state['battery_v'] = self._fv(vals['Voltage'])
                if 'Charge Mode' in vals:
                    self.state['charging'] = vals['Charge Mode'].strip() in ('1', '2')
                if 'Estimated Runtime' in vals:
                    self.state['runtime_min'] = self._fv(vals['Estimated Runtime'])
            elif 'A608 Temp' in name:
                if 'CPU \u00b0C' in vals:
                    self.state['cpu_temp'] = self._fv(vals['CPU \u00b0C'])
                if 'GPU \u00b0C' in vals:
                    self.state['gpu_temp'] = self._fv(vals['GPU \u00b0C'])
            elif 'A608 Fan' in name:
                if 'RAM %' in vals:
                    self.state['ram_pct'] = self._fv(vals['RAM %'])
                if 'CPU %' in vals:
                    self.state['cpu_pct'] = self._fv(vals['CPU %'])
            elif 'A608 Power' in name:
                if 'VDD_IN W' in vals:
                    self.state['power_w'] = self._fv(vals['VDD_IN W'])

    def _on_world(self, msg: WorldState):
        self.state['face_present'] = msg.face_present
        self.state['face_stable'] = msg.face_stable
        self.state['person_name'] = msg.focus_person_name or ''
        self.state['engagement'] = msg.engagement_level
        self.state['tracking_on'] = msg.tracking_enabled
        self.state['conv_active'] = msg.conversation_active
        self.state['ws_health'] = msg.health_state or ''

    def _on_event(self, msg: SocialEvent):
        self.state['event_count'] += 1
        ts = datetime.now().strftime('%H:%M:%S')

        try:
            pl = json.loads(msg.payload_json) if msg.payload_json else {}
        except Exception:
            pl = {}

        et = msg.event_type
        log_entry = {'time': ts, 'type': 'event', 'event_type': et, 'data': {}}

        if et == 'SYSTEM_MODE_CHANGED':
            new_mode = pl.get('mode', '?')
            rs = pl.get('robot_state', {})
            self.state['missing_nodes'] = rs.get('missing_required_nodes', [])
            if rs.get('nodes_present'):
                self.state['nodes'].update(rs['nodes_present'])
            old = self.state['mode']
            self.state['mode'] = new_mode
            log_entry['data'] = {'old_mode': old, 'new_mode': new_mode}

        elif et == 'NODE_MISSING':
            hw = pl.get('hardware', '?')
            ela = pl.get('elapsed_s', 0.0)
            self.state['nodes'][hw] = False
            log_entry['data'] = {'hardware': hw, 'elapsed_s': ela}

        elif et == 'NODE_RECOVERED':
            hw = pl.get('hardware', '?')
            self.state['nodes'][hw] = True
            log_entry['data'] = {'hardware': hw}

        elif et == 'START_PROFILE':
            self.state['profile'] = pl.get('profile', self.state['profile'])
            log_entry['data'] = {'profile': self.state['profile'], 'missing': pl.get('missing_nodes', [])}

        elif et == 'FACE_APPEARED':
            # Mise à jour position 3D
            self.face_position['x'] = pl.get('face_x', 0.0)
            self.face_position['y'] = pl.get('face_y', 0.0)
            self.face_position['z'] = pl.get('face_z', 0.0)
            self.face_position['distance'] = pl.get('distance', 0.0)
            self.face_position['timestamp'] = datetime.now().timestamp()

            log_entry['data'] = {
                'person_name': pl.get('person_name', '?'),
                'person_id': pl.get('person_id', ''),
            }

        elif et == 'PERSON_RECOGNIZED':
            # Mise à jour session : dernière reconnaissance
            # Les données sont dans les champs du message, pas dans le payload
            person = msg.person_name or ''
            if person and self.session['active'] and self.session['person_name'] == person:
                self.session['last_recognition'] = datetime.now().timestamp()
                if self.session['status'] == 'lost':
                    self.session['status'] = 'greeted'  # Retour après perte courte

            # Données pour le log depuis les champs du message SocialEvent
            log_entry['data'] = {
                'person_name': msg.person_name or '?',
                'person_id': msg.person_id or '',
                'confidence': f"{msg.confidence:.2f}",
                'known': pl.get('known', False),
            }

        elif et == 'FACE_STABLE':
            # Mise à jour position 3D
            self.face_position['x'] = pl.get('face_x', 0.0)
            self.face_position['y'] = pl.get('face_y', 0.0)
            self.face_position['z'] = pl.get('face_z', 0.0)
            self.face_position['distance'] = pl.get('distance', 0.0)
            self.face_position['timestamp'] = datetime.now().timestamp()

            log_entry['data'] = {
                'person_name': pl.get('person_name', '?'),
                'person_id': pl.get('person_id', ''),
                'distance': f"{pl.get('distance', 0.0):.2f}m",
            }

        elif et == 'FACE_LOST':
            # Mise à jour session : visage perdu
            if self.session['active']:
                self.session['face_lost_at'] = datetime.now().timestamp()
                self.session['status'] = 'lost'

            # Invalider la position (plus de visage)
            self.face_position['timestamp'] = None

            elapsed = pl.get('elapsed_s', 0.0)
            log_entry['data'] = {
                'elapsed': f"{elapsed:.1f}s",
                'person_name': pl.get('person_name', '?'),
            }

        log_entry['source'] = msg.source
        log_entry['person_name'] = msg.person_name
        log_entry['payload'] = pl

        self.state['log'].append(log_entry)

    def _on_intent(self, msg: BehaviorIntent):
        self.state['intent_count'] += 1
        self.state['filtered_intent'] = msg.intent_type
        self.state['intent_reason'] = msg.reason or ''

        # Mise à jour session : début de session sur GREET_PERSON
        if msg.intent_type == 'GREET_PERSON' and msg.target_person_name:
            now = datetime.now().timestamp()
            self.session['active'] = True
            self.session['person_name'] = msg.target_person_name
            self.session['started_at'] = now
            self.session['last_recognition'] = now
            self.session['face_lost_at'] = None
            self.session['status'] = 'greeted'

        ts = datetime.now().strftime('%H:%M:%S')
        log_entry = {
            'time': ts,
            'type': 'intent',
            'intent_type': msg.intent_type,
            'reason': msg.reason,
            'person_name': msg.target_person_name,
        }
        self.state['log'].append(log_entry)

    def _on_raw(self, msg: BehaviorIntent):
        self.state['raw_intent'] = msg.intent_type

    def _on_trace(self, msg: DecisionTrace):
        if msg.suppressed_by_cooldown:
            self.state['blocked_count'] += 1

        # Mise à jour session : détection de timeout ou reset
        reason = msg.reason or ''
        if 'session_timeout_expired' in reason or 'long_loss' in reason:
            # Reset de session
            self.session['active'] = False
            self.session['person_name'] = ''
            self.session['started_at'] = None
            self.session['last_recognition'] = None
            self.session['face_lost_at'] = None
            self.session['status'] = 'idle'

        ts = datetime.now().strftime('%H:%M:%S')
        log_entry = {
            'time': ts,
            'type': 'trace',
            'trigger': msg.triggering_event,
            'chosen_intent': msg.chosen_intent or '—',
            'relevance': msg.relevance_score,
            'suppressed': msg.suppressed_by_cooldown,
            'reason': msg.reason or '',
        }
        self.state['log'].append(log_entry)


# =============================================================================
# SERVEUR WEB AIOHTTP
# =============================================================================

async def websocket_handler(request):
    """Handler websocket pour les mises à jour temps réel."""
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    node = request.app['ros_node']
    await node.add_websocket_client(ws)

    try:
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                # Gérer les commandes du client
                try:
                    data = json.loads(msg.data)
                    cmd = data.get('command')

                    if cmd == 'set_goal':
                        goal = data.get('goal', 'FIND_AND_GREET')
                        timeout = data.get('timeout', 60.0)
                        node.set_goal(goal, timeout)
                        await ws.send_json({'status': 'ok', 'message': f'Goal {goal} défini'})

                    elif cmd == 'cancel_goal':
                        node.cancel_goal()
                        await ws.send_json({'status': 'ok', 'message': 'Goal annulé'})

                    elif cmd == 'send_intent':
                        intent_type = data.get('intent_type', '')
                        payload = data.get('payload', {})
                        node.get_logger().info(f'🌐 WS reçu: send_intent → {intent_type} | {payload}')
                        if intent_type:
                            node.send_intent(intent_type, payload)
                            await ws.send_json({
                                'status': 'ok',
                                'message': f'Intent {intent_type} envoyé'
                            })
                        else:
                            await ws.send_json({
                                'status': 'error',
                                'message': 'intent_type requis'
                            })

                except Exception as e:
                    node.get_logger().error(f'❌ Exception dans websocket_handler: {type(e).__name__}: {e}')
                    import traceback
                    node.get_logger().error(f'Traceback:\n{traceback.format_exc()}')
                    await ws.send_json({'status': 'error', 'message': str(e)})

            elif msg.type == aiohttp.WSMsgType.ERROR:
                if not node._shutting_down:
                    node.get_logger().warn(f'Erreur websocket: {ws.exception()}')

    finally:
        await node.remove_websocket_client(ws)

    return ws


async def index_handler(request):
    """Redirige vers le fichier HTML statique."""
    return web.FileResponse('./static/index.html')


async def start_web_server(node: QboDashboardNode, port: int = 8080):
    """Démarre le serveur web aiohttp."""
    app = web.Application()
    app['ros_node'] = node

    # Routes
    app.router.add_get('/', index_handler)
    app.router.add_get('/ws', websocket_handler)

    # Route pour les fichiers statiques (CSS, JS)
    app.router.add_static('/static', './static', name='static')

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, '0.0.0.0', port)
    await site.start()

    node.get_logger().info(f'Serveur web démarré sur http://0.0.0.0:{port}')
    node.get_logger().info(f'Fichiers statiques servis depuis: ./static/')
    return runner


# =============================================================================
# MAIN
# =============================================================================

def main():
    rclpy.init()

    # Créer le nœud ROS2
    node = QboDashboardNode()

    # Créer un executor pour ROS2
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Démarrer le serveur web dans la boucle asyncio
    loop = asyncio.get_event_loop()
    # Attacher la boucle asyncio au nœud pour les appels cross-thread
    node.asyncio_loop = loop
    runner = loop.run_until_complete(start_web_server(node, port=8080))

    # Thread ROS2
    import threading
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        # Boucle asyncio pour le serveur web
        loop.run_forever()
    except KeyboardInterrupt:
        print('\n🛑 Arrêt demandé...')
    finally:
        # Activer le flag de shutdown pour éviter les logs ROS2
        node._shutting_down = True

        # Fermer tous les websockets proprement
        loop.run_until_complete(node.close_all_websockets())

        print('⏹️  Destruction du nœud...')
        node.destroy_node()

        print('⏹️  Shutdown ROS2...')
        print('✅ Dashboard arrêté proprement.')


if __name__ == '__main__':
    main()
