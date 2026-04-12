#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from datetime import datetime

from qbo_msgs.msg import SocialEvent


class TimeEventPublisher(Node):
    """
    Node qui publie des événements temporels sur /qbo_social/events

    Événements publiés :
    - TIME_MORNING, TIME_AFTERNOON, TIME_EVENING, TIME_NIGHT
    - DAY_WEEKDAY, DAY_WEEKEND
    """

    def __init__(self):
        super().__init__('qbo_time_event_publisher')

        self.pub_event = self.create_publisher(
            SocialEvent,
            '/qbo_social/events',
            10
        )

        # État actuel (pour détecter les changements)
        self.current_time_period = None
        self.current_day_type = None

        # Configuration des plages horaires (personnalisable via paramètres ROS2 si besoin)
        self.time_periods = {
            'MORNING': (6, 12),      # 6h-12h
            'AFTERNOON': (12, 18),   # 12h-18h
            'EVENING': (18, 22),     # 18h-22h
            'NIGHT': (22, 6),        # 22h-6h
        }

        # Timer pour vérifier les changements toutes les minutes
        self.create_timer(60.0, self._check_temporal_state)

        # Publier l'état initial immédiatement
        self._check_temporal_state()

        # Republier après 2 secondes pour s'assurer que tous les nodes sont prêts (one-shot)
        self.republish_timer = self.create_timer(2.0, self._republish_initial_state)

        self.get_logger().info('TimeEventPublisher started - monitoring temporal events')

    def _get_time_period(self):
        """Détermine la période de la journée actuelle"""
        now = datetime.now()
        hour = now.hour

        if 6 <= hour < 12:
            return 'MORNING'
        elif 12 <= hour < 18:
            return 'AFTERNOON'
        elif 18 <= hour < 22:
            return 'EVENING'
        else:  # 22h-6h
            return 'NIGHT'

    def _get_day_type(self):
        """Détermine si c'est un jour de semaine ou weekend"""
        now = datetime.now()
        weekday = now.weekday()  # 0=Lundi, 6=Dimanche

        if weekday < 5:  # Lundi à Vendredi
            return 'WEEKDAY'
        else:  # Samedi, Dimanche
            return 'WEEKEND'

    def _check_temporal_state(self):
        """Vérifie l'état temporel et publie des événements si changement"""
        time_period = self._get_time_period()
        day_type = self._get_day_type()

        # Publier événement TIME_* si changement de période
        if time_period != self.current_time_period:
            self._publish_time_event(time_period)
            self.current_time_period = time_period

        # Publier événement DAY_* si changement de type de jour
        if day_type != self.current_day_type:
            self._publish_day_event(day_type)
            self.current_day_type = day_type

    def _republish_initial_state(self):
        """Republie l'état initial une seule fois après le démarrage (pour les nodes qui démarrent lentement)"""
        # Forcer la republication en réinitialisant les états
        self.current_time_period = None
        self.current_day_type = None

        self._check_temporal_state()

        # Détruire ce timer pour qu'il ne se répète pas
        self.republish_timer.cancel()
        self.get_logger().info('Initial state republished for late subscribers')

    def _publish_time_event(self, time_period):
        """Publie un événement de type TIME_*"""
        event = SocialEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.header.frame_id = 'qbo_time'
        event.stamp = self.get_clock().now().to_msg()
        event.event_type = f'TIME_{time_period}'
        event.source = 'time_event_publisher'
        event.confidence = 1.0
        event.person_id = ''
        event.person_name = ''
        event.payload_json = f'{{"time_period": "{time_period}", "hour": {datetime.now().hour}}}'

        self.pub_event.publish(event)
        self.get_logger().info(f'Published temporal event: TIME_{time_period}')

    def _publish_day_event(self, day_type):
        """Publie un événement de type DAY_*"""
        event = SocialEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.header.frame_id = 'qbo_time'
        event.stamp = self.get_clock().now().to_msg()
        event.event_type = f'DAY_{day_type}'
        event.source = 'time_event_publisher'
        event.confidence = 1.0
        event.person_id = ''
        event.person_name = ''

        now = datetime.now()
        event.payload_json = f'{{"day_type": "{day_type}", "weekday": {now.weekday()}, "date": "{now.strftime("%Y-%m-%d")}"}}'

        self.pub_event.publish(event)
        self.get_logger().info(f'Published day event: DAY_{day_type}')


def main(args=None):
    rclpy.init(args=args)
    node = TimeEventPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
