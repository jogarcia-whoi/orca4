#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from pyzbar.pyzbar import decode

class QRCodeDetector(Node):

    def __init__(self):
        super().__init__('qr_code_detector')
        self.subscription = self.create_subscription(
            Image,
            '/stereo_left',  # Adjust if needed
            self.image_callback,
            10)
        self.publisher = self.create_publisher(String, '/qr_code_detected', 10)
        self.cv_bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            decoded_objects = decode(gray)
            for obj in decoded_objects:
                qr_data = obj.data.decode('utf-8')
                self.get_logger().info(f'QR Code detected: {qr_data}')
                
                qr_msg = String()
                qr_msg.data = qr_data
                self.publisher.publish(qr_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    qr_detector = QRCodeDetector()
    rclpy.spin(qr_detector)
    qr_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
