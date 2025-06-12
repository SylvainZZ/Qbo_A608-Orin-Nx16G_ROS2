#!/usr/bin/env python3
import rclpy
import random
import time
from rclpy.node import Node
from builtin_interfaces.msg import Time
from sensor_msgs.msg import JointState
import math

class JointLatencyTester(Node):
    def __init__(self):
        super().__init__('joint_latency_tester')

        # Configuration des joints
        self.joints = {
            'head_pan_joint': {'min': -70.0, 'max': 70.0},
            'head_tilt_joint': {'min': -30.0, 'max': 20.0},
        }

        # Publieur sur /cmd_joints
        self.cmd_pub = self.create_publisher(JointState, '/cmd_joints', 10)

        # Abonné à /joint_states
        self.state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)

        # Timer de publication régulière
        timer_period = 2.0  # toutes les 2 secondes
        self.timer = self.create_timer(timer_period, self.send_random_command)

        # Pour traquer les mouvements
        self.last_commands = {}  # joint_name: {'position': ..., 'timestamp': ...}
        self.started = False

    def send_random_command(self):
        msg = JointState()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now

        for joint_name, limits in self.joints.items():
            deg = random.uniform(limits['min'], limits['max'])
            pos = math.radians(deg)
            msg.name.append(joint_name)
            msg.position.append(pos)
            msg.velocity.append(3.5)  # vitesse fixe

            # Enregistrement du timestamp d'envoi
            self.last_commands[joint_name] = {
                'position': pos,
                'timestamp': time.time()
            }

            self.get_logger().info(f"[ENVOI] {joint_name}: {deg:.2f} deg -> {pos:.2f} rad")

        # ✅ Publication du message
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"[MSG] {msg}")


    def joint_state_cb(self, msg: JointState):
        now = time.time()
        for i, name in enumerate(msg.name):
            if name in self.last_commands:
                target_pos = self.last_commands[name]['position']
                sent_time = self.last_commands[name]['timestamp']
                current_pos = msg.position[i]

                # Détection mouvement (delta simple, à améliorer si besoin)
                if abs(current_pos - target_pos) < 0.02:
                    latency = now - sent_time
                    self.get_logger().info(f"[OK] {name}: atteint {current_pos:.2f} rad (latence {latency:.3f} s)")
                    del self.last_commands[name]  # éviter les doublons

def main(args=None):
    rclpy.init(args=args)
    node = JointLatencyTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
