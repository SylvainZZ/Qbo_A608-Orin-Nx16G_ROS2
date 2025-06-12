import rclpy
from rclpy.node import Node
# from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
import csv
import os
import time
from datetime import datetime
from collections import OrderedDict


class DiagnosticLogger(Node):
    def __init__(self):
        super().__init__('diagnostic_logger')

        self.declare_parameter('log_level', 'info')
        level = self.get_parameter('log_level').get_parameter_value().string_value
        self.get_logger().set_level(getattr(rclpy.logging.LoggingSeverity, level.upper(), rclpy.logging.LoggingSeverity.INFO))


        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.listener_callback,
            10)

        # Répertoire local pour les logs de debug (modifiable plus tard pour une version déployée)
        base_log_dir = os.path.join(os.path.expanduser('~'), 'qbo_ws', 'src', 'qbo_driver', 'logs')
        os.makedirs(base_log_dir, exist_ok=True)
        self.log_dir = base_log_dir

        self.current_date = None
        self.csv_file = None
        self.writer = None
        self.fieldnames = ['timestamp', 'hardware_id', 'name', 'level', 'message', 'key', 'value']
        self.known_keys = set()
        # gestion des diagnostics statiques
        self.static_file = open(os.path.join(self.log_dir, 'static_info.csv'), 'w', newline='')
        self.static_writer = None
        self.static_keys = set()
        self.known_statics = set()  # (hardware_id, name)
        self.static_data = dict()  # Pour garder en mémoire les lignes déjà vues
        self.get_logger().info("Démarrage du script d'enregistrement des logs diagnostics...")

    def get_log_filename(self):
        date_str = datetime.now().strftime('%d_%m_%Y')
        return os.path.join(self.log_dir, f"{date_str}.csv")

    def open_csv_file(self):
        self.close_csv_file()  # always close current file first

        filename = self.get_log_filename()
        file_exists = os.path.isfile(filename)

        self.csv_file = open(filename, 'a', newline='')
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.fieldnames)

        if not file_exists:
            self.writer.writeheader()

    def close_csv_file(self):
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.writer = None

    def update_fieldnames(self, new_keys):
        added = False
        for key in new_keys:
            if key not in self.known_keys:
                self.known_keys.add(key)
                self.fieldnames.append(key)
                added = True

        if added:
            # Force file reopen with new header
            self.open_csv_file()


    def listener_callback(self, msg):
        today = datetime.now().strftime('%d_%m_%Y')
        if today != self.current_date:
            self.current_date = today
            self.open_csv_file()

        level_map = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}

        for status in msg.status:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            hardware_id = status.hardware_id
            name = status.name.split(':')[-1].strip()
            raw_level = status.level
            level_int = raw_level if isinstance(raw_level, int) else int.from_bytes(raw_level, byteorder='little')
            level = level_map.get(level_int, str(level_int))
            message = status.message

            # Enregistrement des valeurs statiques (si clé "_...")
            if any(kv.key.startswith('_') for kv in status.values):
                self.write_static_info(status)

            # Enregistrement des valeurs dynamiques seulement
            for kv in status.values:
                if not kv.key.startswith('_'):
                    row = {
                        'timestamp': timestamp,
                        'hardware_id': hardware_id,
                        'name': name,
                        'level': level,
                        'message': message,
                        'key': kv.key,
                        'value': kv.value
                    }
                    try:
                        self.writer.writerow(row)
                        self.csv_file.flush()
                    except Exception as e:
                        self.get_logger().error(f"Erreur écriture CSV: {e}")



    def write_static_info(self, status):
        key = (status.hardware_id, status.name)
        base_entry = self.static_data.get(key, OrderedDict({
            'hardware_id': status.hardware_id,
            'name': status.name
        }))

        updated = False

        for kv in status.values:
            if kv.key.startswith('_'):
                clean_key = kv.key.lstrip('_')
                if clean_key not in base_entry or base_entry[clean_key] != kv.value:
                    base_entry[clean_key] = kv.value
                    self.static_keys.add(clean_key)
                    updated = True

        if updated:
            self.static_data[key] = base_entry

            # Recrée le writer si nouvelles colonnes
            all_keys = ['hardware_id', 'name'] + sorted(self.static_keys)
            self.static_file.seek(0, os.SEEK_SET)
            self.static_file.truncate()  # on réecrit tout
            self.static_writer = csv.DictWriter(self.static_file, fieldnames=all_keys)
            self.static_writer.writeheader()

            for entry in self.static_data.values():
                self.static_writer.writerow(entry)
            self.static_file.flush()

            self.get_logger().info(f"[STATIC] Mise à jour des données statiques : {key}")


    def destroy_node(self):
        self.close_csv_file()
        if self.static_file:
            self.static_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    logger = DiagnosticLogger()
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        pass
    finally:
        self.get_logger().info("Arrêt du noeud, fermeture des fichiers.")
        logger.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
