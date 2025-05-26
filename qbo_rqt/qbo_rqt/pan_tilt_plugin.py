import os
from rqt_gui_py.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QPushButton, QSlider, QLabel
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from qbo_msgs.srv import TorqueEnable
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import QTimer
from rclpy.executors import SingleThreadedExecutor

class PanTiltPlugin(Plugin):

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('PanTiltPlugin')

        self._widget = QWidget()
        ui_file = os.path.join(get_package_share_directory('qbo_rqt'), 'resource', 'pan_tilt_plugin_v1.ui')
        loadUi(ui_file, self._widget)
        print("PanTiltPlugin UI loaded.")

        if context.serial_number() > 1:
            self._widget.setWindowTitle(f'PanTiltPlugin ({context.serial_number()})')
        context.add_widget(self._widget)

        # Lancement différé de l'initialisation ROS (évite les erreurs RQt)
        QTimer.singleShot(0, lambda: self.init_ros(context.serial_number()))

        # UI Elements
        self.slider_pan = self._widget.findChild(QSlider, 'slider_pan')
        self.slider_tilt = self._widget.findChild(QSlider, 'slider_tilt')
        self.label_position = self._widget.findChild(QLabel, 'label_position')
        self.button_enable = self._widget.findChild(QPushButton, 'button_enable')
        self.button_disable = self._widget.findChild(QPushButton, 'button_disable')

        self._widget.setMinimumSize(400, 200)
        self._widget.adjustSize()

    def init_ros(self, serial):
        node_name = f'pan_tilt_plugin_node_{serial}'
        self.node = rclpy.create_node(node_name)

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        # Lancer le spin dans un timer Qt
        self.spin_timer = QTimer()
        self.spin_timer.timeout.connect(self.executor.spin_once)
        self.spin_timer.start(10)  # 10 ms intervalle pour écouter ROS

        # ROS
        self.pub_cmd = self.node.create_publisher(JointState, '/cmd_joints', 10)
        self.sub_joint_states = self.node.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.torque_pan = self.node.create_client(TorqueEnable, '/head_pan_joint/torque_enable')
        self.torque_tilt = self.node.create_client(TorqueEnable, '/head_tilt_joint/torque_enable')

        # Connexions UI -> ROS
        self.slider_pan.valueChanged.connect(self.send_joint_command)
        self.slider_tilt.valueChanged.connect(self.send_joint_command)
        self.button_enable.clicked.connect(lambda: self.send_torque(True))
        self.button_disable.clicked.connect(lambda: self.send_torque(False))
        self.button_reset = self._widget.findChild(QPushButton, 'button_reset')
        self.button_reset.clicked.connect(self.reset_servos)
        self.node.get_logger().info("PanTiltPlugin ROS node initialized")

    def send_joint_command(self):
        msg = JointState()
        msg.name = ['head_pan_joint', 'head_tilt_joint']
        msg.position = [
            self.slider_pan.value() / 100.0,  # example scale -1.0 to 1.0
            self.slider_tilt.value() / 100.0
        ]
        self.pub_cmd.publish(msg)

    def send_torque(self, enable):
        req = TorqueEnable.Request()
        req.torque_enable = enable
        if self.torque_pan.service_is_ready():
            self.torque_pan.call_async(req)
        if self.torque_tilt.service_is_ready():
            self.torque_tilt.call_async(req)

    def joint_states_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        pan = positions.get('head_pan_joint', 0.0)
        tilt = positions.get('head_tilt_joint', 0.0)
        self.label_position.setText(f'Pan: {pan:.2f}  Tilt: {tilt:.2f}')

    def reset_servos(self):
        self.slider_pan.setValue(0)
        self.slider_tilt.setValue(0)
        self.send_joint_command()  # réutilise la fonction existante

    def shutdown_plugin(self):
      if hasattr(self, 'node'):
        self.node.destroy_node()
        # rclpy.shutdown()
