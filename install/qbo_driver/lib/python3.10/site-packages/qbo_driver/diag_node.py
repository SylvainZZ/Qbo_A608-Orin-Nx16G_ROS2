import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_share_directory
from qbo_driver.hardware_info import SENSOR_FUNCTIONS
import os
import yaml

class DiagNode(Node):
    def __init__(self):
        super().__init__('diag_node')
        self.publisher_ = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.profile = 'orin-nx-16g'  # Tu pourras rendre ça paramétrable

        yaml_path = self.declare_parameter(
            'thresholds_path',
            os.path.join(
                get_package_share_directory('qbo_driver'),
                'thresholds.yaml'
            )
        ).get_parameter_value().string_value

        try:
            pkg_dir = get_package_share_directory('qbo_driver')
            thresholds_path = os.path.join(pkg_dir, 'thresholds.yaml')
            with open(thresholds_path, 'r') as f:
                self.thresholds = yaml.safe_load(f)
                self.get_logger().info(f'Loaded thresholds from: {yaml_path}')
                self.validate_thresholds()
        except Exception as e:
            self.get_logger().error(f'Failed to load or validate thresholds.yaml: {e}')
            raise RuntimeError('Invalid thresholds config. Shutting down.')

        param_names = ['thresholds_path']  # liste explicite si besoin
        for name in param_names:
            if self.has_parameter(name):
                value = self.get_parameter(name).value
                self.get_logger().info(f"Parameter: {name} = {value}")
            else:
                self.get_logger().warn(f"Parameter {name} not declared.")

    def timer_callback(self):
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = f"QBO Diagnostics [{self.profile}]"
        status.hardware_id = self.profile
        status.level = DiagnosticStatus.OK
        status.message = "OK"

        sensors = self.thresholds.get('diagnostic', {}).get(self.profile, {})

        for label, config in sensors.items():
            self.get_logger().debug(f'Processing sensor: {label}')
            if not config.get('enabled', False):
                continue

            func = SENSOR_FUNCTIONS.get(label)
            if not func:
                self.get_logger().warn(f'No function defined for sensor "{label}"')
                continue

            value = func()
            if value is None:
                self.get_logger().warn(f'No data from sensor "{label}"')
                continue

            threshold = config['threshold']
            name = config['name']
            unit = config['unit']

            status.values.append(KeyValue(key=f"{name} ({unit})", value=str(value)))
            if value > threshold:
                if status.level < DiagnosticStatus.WARN:
                    status.level = DiagnosticStatus.WARN
                    status.message = f'{name} above threshold!'

        diag_msg.status.append(status)
        self.publisher_.publish(diag_msg)
        level_value = status.level
        if isinstance(level_value, bytes):
            level_value = int.from_bytes(level_value, byteorder='little')

        self.get_logger().info(f'Published diagnostic status: level={level_value}')

    def validate_thresholds(self):
        required_fields = ['name', 'unit', 'threshold', 'enabled']
        errors = []
        sensors = self.thresholds.get('diagnostic', {}).get(self.profile, {})

        for label, config in sensors.items():
            for field in required_fields:
                if field not in config:
                    errors.append(f"Missing '{field}' in config for '{label}'")

            if config.get('enabled', False) and label not in SENSOR_FUNCTIONS:
                errors.append(f"No function defined for enabled sensor '{label}'")

        if errors:
            for err in errors:
                self.get_logger().error(f"[Config Error] {err}")
            raise RuntimeError("Invalid thresholds.yaml: missing fields or functions.")


def main(args=None):
    rclpy.init(args=args)
    node = DiagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()
