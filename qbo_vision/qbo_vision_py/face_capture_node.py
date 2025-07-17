#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import os
import time

class FaceCaptureNode(Node):
    def __init__(self):
        super().__init__('qbo_face_capture')

        # 📥 Paramètres
        self.declare_parameter('name', 'unknown')
        self.declare_parameter('path_faces_learn', '/home/qbo-v2/qbo_ws/src/qbo_vision/faces/learn')
        self.declare_parameter('capture_interval', 2.0)

        self.name = self.get_parameter('name').value
        self.path_faces_learn = self.get_parameter('path_faces_learn').value
        self.capture_interval = self.get_parameter('capture_interval').value

        # 🔧 Créer le dossier si besoin
        self.save_dir = os.path.join(self.path_faces_learn, self.name)
        os.makedirs(self.save_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.last_capture_time = 0

        self.subscription = self.create_subscription(
            Image,
            '/camera_left/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info(f"📸 Enregistrement des visages de [{self.name}] dans {self.save_dir}")

    def image_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_capture_time < self.capture_interval:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f"Erreur conversion image : {e}")
            return

        # 🔄 Prétraitement identique à ton LBPH
        face_resized = cv2.resize(cv_image, (140, 140))
        face_equalized = cv2.equalizeHist(face_resized)
        face_blurred = cv2.GaussianBlur(face_equalized, (3, 3), 0)

        timestamp = str(int(current_time * 1e6))
        base_path = os.path.join(self.save_dir, f"face_{timestamp}")

        cv2.imwrite(base_path + ".jpg", face_blurred)
        cv2.imwrite(base_path + "_flip.jpg", cv2.flip(face_blurred, 1))

        brighter = cv2.convertScaleAbs(face_blurred, alpha=1.1, beta=15)
        cv2.imwrite(base_path + "_bright.jpg", brighter)

        self.last_capture_time = current_time
        self.get_logger().info(f"✅ Image capturée : {base_path}.jpg (+ variantes)")

def main(args=None):
    rclpy.init(args=args)
    node = FaceCaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
