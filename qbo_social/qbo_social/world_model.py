#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from qbo_msgs.msg import SocialEvent, WorldState
from builtin_interfaces.msg import Time

'''
Le world_model doit :
    agréger des événements venant de domaines différents
    mémoriser l'état courant du monde
    exposer ce contexte au reste du système
    rester simple et explicable

Le world_model ne doit pas :
    choisir une action
    piloter des modules
    arbitrer entre plusieurs comportements
    décider du mode social final
    déclencher directement des intentions
'''


class SocialWorldModel(Node):
    def __init__(self):
        super().__init__('qbo_social_world_model')

        self.sub_event = self.create_subscription(
            SocialEvent,
            '/qbo_social/events',
            self._on_event,
            10
        )

        self.pub_state = self.create_publisher(
            WorldState,
            '/qbo_social/world_state',
            10
        )

        self.face_present = False
        self.face_stable = False
        self.conversation_active = False
        self.tracking_enabled = False
        self.head_follow_enabled = False
        self.base_follow_enabled = False
        self.focus_person_id = ''
        self.focus_person_name = ''
        self.engagement_level = 0.0
        # self.mode = 'IDLE'
        # self.health_state = 'OK'

        # Temporal context
        self.time_of_day = ''  # MORNING, AFTERNOON, EVENING, NIGHT
        self.day_type = ''     # WEEKDAY, WEEKEND

        # Network context
        self.network_connected = False
        self.wifi_ssid = ''
        self.wifi_status = ''  # KNOWN, UNKNOWN
        self.ip_address = ''

        # Timestamps
        self.last_face_seen_time = None
        self.last_recognized_time = None
        self.last_network_change_time = None

        # Pas de timer périodique : publication uniquement sur changements significatifs (événements)

        self.get_logger().info('SocialWorldModel started')

    def _parse_payload(self, event: SocialEvent) -> dict:
        """Parse JSON payload from event, returns empty dict on error."""
        try:
            import json
            return json.loads(event.payload_json)
        except Exception:
            return {}

    def _update_follower_flags_from_payload(self, payload: dict):
        # tracking_state: 0=IDLE, 1=SEARCHING, 2=TRACKING, 3=BLOCKED
        tracking_state = payload.get('tracking_state', None)

        if tracking_state is not None:
            self.tracking_enabled = tracking_state in [1, 2, 3]

        if 'head_enabled' in payload:
            self.head_follow_enabled = bool(payload.get('head_enabled', False))

        if 'rotation_enabled' in payload:
            self.base_follow_enabled = bool(payload.get('rotation_enabled', False))

    def _on_event(self, event: SocialEvent):
        # Pas de log ici, sera loggé après mise à jour si changement

        if event.event_type == 'FACE_APPEARED':
            self.face_present = True
            self.last_face_seen_time = self.get_clock().now()

            payload = self._parse_payload(event)
            self._update_follower_flags_from_payload(payload)

        elif event.event_type == 'FACE_STABLE':
            self.face_present = True
            self.face_stable = True
            self.focus_person_id = event.person_id
            self.focus_person_name = event.person_name
            self.engagement_level = max(self.engagement_level, 0.4)
            self.last_face_seen_time = self.get_clock().now()

            payload = self._parse_payload(event)
            self._update_follower_flags_from_payload(payload)
            # self.mode = 'SOCIAL_ACTIVE'

        elif event.event_type == 'FACE_LOST':
            self.face_present = False
            self.face_stable = False
            self.focus_person_id = ''
            self.focus_person_name = ''
            self.engagement_level = 0.0
            # self.mode = 'IDLE'

        elif event.event_type == 'PERSON_RECOGNIZED':
            self.focus_person_id = event.person_id
            self.focus_person_name = event.person_name
            self.engagement_level = max(self.engagement_level, 0.7)
            self.last_recognized_time = self.get_clock().now()

        # Événements temporels (TIME_*)
        elif event.event_type == 'TIME_MORNING':
            self.time_of_day = 'MORNING'

        elif event.event_type == 'TIME_AFTERNOON':
            self.time_of_day = 'AFTERNOON'

        elif event.event_type == 'TIME_EVENING':
            self.time_of_day = 'EVENING'

        elif event.event_type == 'TIME_NIGHT':
            self.time_of_day = 'NIGHT'

        # Événements de type de jour (DAY_*)
        elif event.event_type == 'DAY_WEEKDAY':
            self.day_type = 'WEEKDAY'

        elif event.event_type == 'DAY_WEEKEND':
            self.day_type = 'WEEKEND'

        # Événements réseau (NETWORK_*)
        elif event.event_type == 'NETWORK_CONNECTED':
            self.network_connected = True
            # Extraire l'IP du payload si disponible
            payload = self._parse_payload(event)
            self.ip_address = payload.get('ip_address', '')
            self.last_network_change_time = self.get_clock().now()

        elif event.event_type == 'NETWORK_DISCONNECTED':
            self.network_connected = False
            self.wifi_ssid = ''  # Réinitialiser le WiFi si déconnecté
            self.wifi_status = ''
            self.ip_address = ''
            self.last_network_change_time = self.get_clock().now()

        # Événements WiFi (WIFI_*)
        elif event.event_type == 'WIFI_KNOWN':
            self.wifi_status = 'KNOWN'
            # Extraire le SSID du payload si disponible
            payload = self._parse_payload(event)
            self.wifi_ssid = payload.get('ssid', '')

        elif event.event_type == 'WIFI_UNKNOWN':
            self.wifi_status = 'UNKNOWN'
            # Extraire le SSID du payload si disponible
            payload = self._parse_payload(event)
            self.wifi_ssid = payload.get('ssid', '')

        self._publish_world_state()

    def _publish_world_state(self):
        msg = WorldState()
        now = self.get_clock().now().to_msg()

        msg.header.stamp = now
        msg.header.frame_id = 'qbo_social'
        msg.stamp = now

        msg.face_present = self.face_present
        msg.face_stable = self.face_stable
        msg.conversation_active = self.conversation_active
        msg.tracking_enabled = self.tracking_enabled
        msg.head_follow_enabled = self.head_follow_enabled
        msg.base_follow_enabled = self.base_follow_enabled
        msg.focus_person_id = self.focus_person_id
        msg.focus_person_name = self.focus_person_name
        msg.engagement_level = self.engagement_level
        msg.time_of_day = self.time_of_day
        msg.day_type = self.day_type
        msg.network_connected = self.network_connected
        msg.wifi_ssid = self.wifi_ssid
        msg.wifi_status = self.wifi_status
        msg.ip_address = self.ip_address

        # Timestamps (convertir None en Time vide)

        msg.last_face_seen_time = self.last_face_seen_time.to_msg() if self.last_face_seen_time else Time()
        msg.last_recognized_time = self.last_recognized_time.to_msg() if self.last_recognized_time else Time()
        msg.last_network_change_time = self.last_network_change_time.to_msg() if self.last_network_change_time else Time()

        self.pub_state.publish(msg)

        # Log uniquement si débug activé (sinon trop verbeux avec le timer)
        # Le debug_behavior_state.py affichera les WorldState avec --verbose


def main(args=None):
    rclpy.init(args=args)
    node = SocialWorldModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()