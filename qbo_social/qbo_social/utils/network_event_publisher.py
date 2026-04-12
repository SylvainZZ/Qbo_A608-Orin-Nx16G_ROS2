#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess
import socket
import re

from qbo_msgs.msg import SocialEvent


class NetworkEventPublisher(Node):
    """
    Node qui publie des événements réseau sur /qbo_social/events

    Événements publiés :
    - NETWORK_CONNECTED, NETWORK_DISCONNECTED
    - WIFI_KNOWN, WIFI_UNKNOWN
    """

    def __init__(self):
        super().__init__('qbo_network_event_publisher')

        self.pub_event = self.create_publisher(
            SocialEvent,
            '/qbo_social/events',
            10
        )

        # État actuel (pour détecter les changements)
        self.current_network_status = None
        self.current_wifi_ssid = None
        self.current_wifi_status = None
        self.current_ip_address = None

        # Liste des réseaux WiFi connus (configurable via paramètres ROS2)
        self.declare_parameter('known_networks', ['Lune_Ext'])  # Personnalisable
        self.known_networks = self.get_parameter('known_networks').value

        # Serveur pour tester la connectivité internet (Google DNS par défaut)
        self.declare_parameter('test_host', '8.8.8.8')
        self.test_host = self.get_parameter('test_host').value

        # Timer pour vérifier l'état réseau toutes les 10 secondes
        self.create_timer(10.0, self._check_network_state)

        # Publier l'état initial immédiatement
        self._check_network_state()

        # Republier après 2 secondes pour les subscribers tardifs
        self.republish_timer = self.create_timer(2.0, self._republish_initial_state)

        self.get_logger().info(f'NetworkEventPublisher started - monitoring network (known networks: {self.known_networks})')

    def _is_connected_to_internet(self):
        """Teste la connectivité internet en tentant de contacter un serveur"""
        try:
            # Tente une connexion socket rapide (timeout 3 secondes)
            socket.create_connection((self.test_host, 53), timeout=3)
            return True
        except (socket.timeout, socket.error, OSError):
            return False

    def _get_local_ip(self):
        """Récupère l'adresse IP locale de l'interface active"""

        # Méthode 1 : ip addr (moderne)
        try:
            result = subprocess.run(
                ['ip', 'addr', 'show'],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0:
                # Cherche une interface UP avec une adresse IPv4 (exclude loopback)
                lines = result.stdout.split('\n')
                current_interface = None
                is_up = False

                for line in lines:
                    # Détecte une nouvelle interface
                    if re.match(r'^\d+:', line):
                        current_interface = line
                        # Vérifie si l'interface est UP et pas LOOPBACK
                        is_up = 'UP' in line and 'LOOPBACK' not in line and 'state UP' in line

                    # Si l'interface est UP, cherche l'adresse inet
                    if is_up and 'inet ' in line and 'scope global' in line:
                        match = re.search(r'inet (\d+\.\d+\.\d+\.\d+)', line)
                        if match:
                            return match.group(1)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass

        # Méthode 2 : ifconfig (legacy)
        try:
            result = subprocess.run(
                ['ifconfig'],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0:
                # Cherche une interface UP avec inet (exclude loopback lo)
                lines = result.stdout.split('\n')
                current_interface = None
                is_up = False

                for line in lines:
                    # Détecte une nouvelle interface
                    if line and not line.startswith(' '):
                        current_interface = line.split(':')[0]
                        is_up = 'UP' in line and 'RUNNING' in line and current_interface != 'lo'

                    # Si l'interface est UP, cherche l'adresse inet
                    if is_up and 'inet ' in line:
                        match = re.search(r'inet (\d+\.\d+\.\d+\.\d+)', line)
                        if match:
                            ip = match.group(1)
                            # Exclure localhost
                            if not ip.startswith('127.'):
                                return ip
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass

        return None

    def _get_wifi_ssid(self):
        """Récupère le SSID du réseau WiFi actuel (Linux)"""
        try:
            # Méthode 1 : iwgetid (simple et rapide)
            result = subprocess.run(
                ['iwgetid', '-r'],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0 and result.stdout.strip():
                return result.stdout.strip()
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass

        try:
            # Méthode 2 : nmcli (NetworkManager)
            result = subprocess.run(
                ['nmcli', '-t', '-f', 'active,ssid', 'dev', 'wifi'],
                capture_output=True,
                text=True,
                timeout=2
            )
            if result.returncode == 0:
                for line in result.stdout.strip().split('\n'):
                    if line.startswith('yes:'):
                        return line.split(':', 1)[1]
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass

        return None

    def _is_known_network(self, ssid):
        """Vérifie si le SSID fait partie des réseaux connus"""
        if not ssid:
            return False
        # Comparaison insensible à la casse
        return ssid.lower() in [net.lower() for net in self.known_networks]

    def _check_network_state(self):
        """Vérifie l'état réseau et publie des événements si changement"""
        # Vérifier la connectivité internet
        is_connected = self._is_connected_to_internet()
        network_status = 'CONNECTED' if is_connected else 'DISCONNECTED'

        # Récupérer l'adresse IP
        ip_address = self._get_local_ip() if is_connected else None

        if network_status != self.current_network_status or ip_address != self.current_ip_address:
            self._publish_network_event(network_status, ip_address)
            self.current_network_status = network_status
            self.current_ip_address = ip_address

        # Vérifier le WiFi uniquement si connecté
        if is_connected:
            wifi_ssid = self._get_wifi_ssid()

            # Détecter changement de SSID ou premier SSID détecté
            if wifi_ssid and wifi_ssid != self.current_wifi_ssid:
                self.current_wifi_ssid = wifi_ssid

                # Déterminer si connu ou inconnu
                is_known = self._is_known_network(wifi_ssid)
                wifi_status = 'KNOWN' if is_known else 'UNKNOWN'

                # Publier l'événement (toujours publier en cas de changement de SSID)
                self._publish_wifi_event(wifi_status, wifi_ssid)
                self.current_wifi_status = wifi_status
        else:
            # Déconnecté : réinitialiser l'état WiFi
            if self.current_wifi_ssid is not None:
                self.current_wifi_ssid = None
                self.current_wifi_status = None
                self.current_ip_address = None

    def _republish_initial_state(self):
        """Republie l'état initial pour les subscribers tardifs"""
        self.current_network_status = None
        self.current_wifi_ssid = None
        self.current_wifi_status = None
        self.current_ip_address = None

        self._check_network_state()

        self.republish_timer.cancel()
        self.get_logger().info('Initial network state republished for late subscribers')

    def _publish_network_event(self, status, ip_address=None):
        """Publie un événement NETWORK_CONNECTED ou NETWORK_DISCONNECTED"""
        event = SocialEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.header.frame_id = 'qbo_network'
        event.stamp = self.get_clock().now().to_msg()
        event.event_type = f'NETWORK_{status}'
        event.source = 'network_event_publisher'
        event.confidence = 1.0
        event.person_id = ''
        event.person_name = ''

        ip_str = ip_address if ip_address else ''
        event.payload_json = f'{{"status": "{status}", "ip_address": "{ip_str}", "test_host": "{self.test_host}"}}'

        self.pub_event.publish(event)
        log_msg = f'Published network event: NETWORK_{status}'
        if ip_address:
            log_msg += f' (IP: {ip_address})'
        self.get_logger().info(log_msg)

    def _publish_wifi_event(self, status, ssid):
        """Publie un événement WIFI_KNOWN ou WIFI_UNKNOWN"""
        event = SocialEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.header.frame_id = 'qbo_network'
        event.stamp = self.get_clock().now().to_msg()
        event.event_type = f'WIFI_{status}'
        event.source = 'network_event_publisher'
        event.confidence = 1.0
        event.person_id = ''
        event.person_name = ''
        event.payload_json = f'{{"status": "{status}", "ssid": "{ssid}"}}'

        self.pub_event.publish(event)
        self.get_logger().info(f'Published WiFi event: WIFI_{status} (SSID: {ssid})')


def main(args=None):
    rclpy.init(args=args)
    node = NetworkEventPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
