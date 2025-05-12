import os
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray


@pytest.fixture(scope='module')
def rclpy_node():
    rclpy.init()
    node = rclpy.create_node('test_diag_listener')
    yield node
    rclpy.shutdown()


def test_receive_diagnostics(rclpy_node):
    messages = []

    def callback(msg):
        messages.append(msg)

    _ = rclpy_node.create_subscription(
        DiagnosticArray,
        '/diagnostics',
        callback,
        10
    )

    os.system("ros2 run qbo_driver diag_node &")

    end_time = rclpy_node.get_clock().now().nanoseconds + 3 * 1e9
    while rclpy_node.get_clock().now().nanoseconds < end_time and len(messages) < 1:
        rclpy.spin_once(rclpy_node, timeout_sec=0.1)

    assert len(messages) > 0, "No diagnostics message received"
