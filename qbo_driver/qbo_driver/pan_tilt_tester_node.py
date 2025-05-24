import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray
from qbo_msgs.msg import MotorState
from qbo_msgs.srv import TorqueEnable

import sys
import termios
import tty
import threading
import time
import select


class PanTiltTester(Node):
    def __init__(self):
        super().__init__('pan_tilt_tester')

        self.publisher_ = self.create_publisher(JointState, '/cmd_joints', 10)

        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.create_subscription(MotorState, '/dynamixel_state', self.dynamixel_callback, 10)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10)

        self.pan = 0.0
        self.tilt = 0.0
        self.step = 0.05

        self.pan_limits = (-1.2, 1.2)
        self.tilt_limits = (-0.2, 0.8)

        self.enable_client_pan = self.create_client(TorqueEnable, '/head_pan_joint/torque_enable')
        self.enable_client_tilt = self.create_client(TorqueEnable, '/head_tilt_joint/torque_enable')

        self.timer = self.create_timer(0.02, self.publish_joint_commands)

        self._running = True
        self._keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self._keyboard_thread.daemon = True
        self._keyboard_thread.start()

    def publish_joint_commands(self):
        msg = JointState()
        msg.name = ['head_pan_joint', 'head_tilt_joint']
        msg.position = [self.pan, self.tilt]
        msg.velocity = []
        self.publisher_.publish(msg)

    def joint_states_callback(self, msg):
        positions = dict(zip(msg.name, msg.position))
        print(f"\r[JointState] Pan: {positions.get('head_pan_joint', 0.0):.2f}, Tilt: {positions.get('head_tilt_joint', 0.0):.2f}      ", end="")

    def dynamixel_callback(self, msg):
        self.get_logger().info(f"[DynamixelState] Motor {msg.id} Pos: {msg.position:.2f}")

    def diagnostics_callback(self, msg):
        # Minimal output — customize as needed
        for status in msg.status:
            self.get_logger().info(f"[Diagnostics] {status.name}: {status.message}")

    def call_torque_enable(self, enable: bool):
        while not self.enable_client_pan.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for torque_enable service (pan)...')
        while not self.enable_client_tilt.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for torque_enable service (tilt)...')

        req = TorqueEnable.Request()
        req.torque_enable = enable
        self.enable_client_pan.call_async(req)
        self.enable_client_tilt.call_async(req)
        state = "ENABLED" if enable else "DISABLED"
        self.get_logger().info(f"Torque {state} on both joints.")

    def keyboard_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)
            while self._running:
                if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                    c = sys.stdin.read(1)
                    if c == '\x1b':  # Arrow keys start with ESC sequence
                        sys.stdin.read(1)  # skip [
                        direction = sys.stdin.read(1)
                        if direction == 'A':  # Up
                            self.tilt += self.step
                        elif direction == 'B':  # Down
                            self.tilt -= self.step
                        elif direction == 'C':  # Right
                            self.pan += self.step
                        elif direction == 'D':  # Left
                            self.pan -= self.step
                    elif c == 'e':
                        self.call_torque_enable(True)
                    elif c == 'd':
                        self.call_torque_enable(False)
                    elif c == 'r':
                        self.pan = 0.0
                        self.tilt = 0.0
                        self.get_logger().info("Reset positions to 0.")
                    elif c == 'q':
                        self._running = False
                        rclpy.shutdown()
                        break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import select
    main()
