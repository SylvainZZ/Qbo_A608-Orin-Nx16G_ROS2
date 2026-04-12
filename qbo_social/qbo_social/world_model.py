#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from qbo_msgs.msg import SocialEvent, WorldState


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
        self.mode = 'IDLE'
        self.health_state = 'OK'

        # Temporal context
        self.time_of_day = ''  # MORNING, AFTERNOON, EVENING, NIGHT
        self.day_type = ''     # WEEKDAY, WEEKEND

        # Network context
        self.network_connected = False
        self.wifi_ssid = ''
        self.wifi_status = ''  # KNOWN, UNKNOWN
        self.ip_address = ''

        # Timer pour publier le WorldState périodiquement (permet au behavior engine de rester à jour)
        self.create_timer(0.5, self._publish_world_state)

        self.get_logger().info('SocialWorldModel started')

    def _on_event(self, event: SocialEvent):
        # Pas de log ici, sera loggé après mise à jour si changement

        # Début volontairement simple
        if event.event_type == 'FACE_APPEARED':
            self.face_present = True

        elif event.event_type == 'FACE_STABLE':
            self.face_present = True
            self.face_stable = True
            self.focus_person_id = event.person_id
            self.focus_person_name = event.person_name
            self.engagement_level = max(self.engagement_level, 0.4)
            self.mode = 'SOCIAL_ACTIVE'

        elif event.event_type == 'FACE_LOST':
            self.face_present = False
            self.face_stable = False
            self.focus_person_id = ''
            self.focus_person_name = ''
            self.engagement_level = 0.0
            self.mode = 'IDLE'

        elif event.event_type == 'PERSON_RECOGNIZED':
            self.focus_person_id = event.person_id
            self.focus_person_name = event.person_name
            self.engagement_level = max(self.engagement_level, 0.7)

        elif event.event_type == 'CONVERSATION_STARTED':
            self.conversation_active = True
            self.mode = 'ENGAGED'

        elif event.event_type == 'CONVERSATION_ENDED':
            self.conversation_active = False
            if self.face_present:
                self.mode = 'SOCIAL_ACTIVE'
            else:
                self.mode = 'IDLE'

        elif event.event_type == "PERSON_RECOGNIZED":
            self.focus_person_name = event.person_name
            self.engagement_level = 0.8

        elif event.event_type == "SYSTEM_MODE_CHANGED":
            try:
                import json
                payload = json.loads(event.payload_json)
                self.health_state = payload.get("mode", "NORMAL")
            except Exception:
                pass

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
            try:
                import json
                payload = json.loads(event.payload_json)
                self.ip_address = payload.get('ip_address', '')
            except Exception:
                pass
        
        elif event.event_type == 'NETWORK_DISCONNECTED':
            self.network_connected = False
            self.wifi_ssid = ''  # Réinitialiser le WiFi si déconnecté
            self.wifi_status = ''
            self.ip_address = ''
        
        # Événements WiFi (WIFI_*)
        elif event.event_type == 'WIFI_KNOWN':
            self.wifi_status = 'KNOWN'
            # Extraire le SSID du payload si disponible
            try:
                import json
                payload = json.loads(event.payload_json)
                self.wifi_ssid = payload.get('ssid', '')
            except Exception:
                pass
        
        elif event.event_type == 'WIFI_UNKNOWN':
            self.wifi_status = 'UNKNOWN'
            # Extraire le SSID du payload si disponible
            try:
                import json
                payload = json.loads(event.payload_json)
                self.wifi_ssid = payload.get('ssid', '')
            except Exception:
                pass

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
        msg.mode = self.mode
        msg.health_state = self.health_state
        msg.time_of_day = self.time_of_day
        msg.day_type = self.day_type
        msg.network_connected = self.network_connected
        msg.wifi_ssid = self.wifi_ssid
        msg.wifi_status = self.wifi_status
        msg.ip_address = self.ip_address

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
        rclpy.shutdown()