#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.subscription = self.create_subscription(
            Image,
            '/car/road_camera/image_color',
            self.image_callback,
            qos
        )
        self.error_pub = self.create_publisher(Int32, '/lane_error', 10)
        self.bridge = CvBridge()


        #Umbral de confianza
        self.LINE_THRESHOLD = 170.0


    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        if img is None:
            self.get_logger().warn('Img es None')
            return

 
        
        # Media 
        column_means = np.mean(img, axis=(0, 2))

        max_brightness = np.max(column_means)

        if max_brightness < self.LINE_THRESHOLD:
            error = 0
            #Línea no detectada
        else:
            center_index = int(np.argmax(column_means))  
            error = int(center_index - 256)       
           #Línea detectada 

         #Centro detectado
        self.error_pub.publish(Int32(data=-error))

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutdown lane_detector')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



