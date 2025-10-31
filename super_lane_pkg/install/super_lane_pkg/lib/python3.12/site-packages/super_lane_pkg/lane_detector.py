#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import numpy as np
import cv2
from cv_bridge import CvBridge

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        self.subscription = self.create_subscription(Image, '/road_camera/image_raw', self.image_callback, 10)
        self.error_pub = self.create_publisher(Int32, '/lane_error', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        column_means = np.mean(img, axis=(0, 2))  # (512,)
        center_index = np.argmax(column_means)
        error = center_index - 256
        self.get_logger().info(f'Error de carril: {error}')
        self.error_pub.publish(Int32(data=error))

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


