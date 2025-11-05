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
        self.get_logger().info('LaneDetector iniciado, esperando imágenes en /car/road_camera/image_color')

    def image_callback(self, msg):
        try:
            # Forzar encoding conocido; prueba 'rgb8' o 'bgr8' según tu driver
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        if img is None:
            self.get_logger().warn('Img es None')
            return

        # Asegurarse de la forma esperada (height, width, channels)
        self.get_logger().debug(f'Imagen recibida shape={getattr(img, "shape", None)} dtype={getattr(img, "dtype", None)}')

        # Media por columna sobre filas (axis=0) y canales (axis=2) -> vector de width valores
        column_means = np.mean(img, axis=(0, 2))
        center_index = int(np.argmax(column_means))    # <-- convertir a int Python nativo
        error = int(center_index - 256)                # <-- int Python nativo

         #self.get_logger().info(f'Centro detectado: {center_index}, Error de carril: {error}')
        self.error_pub.publish(Int32(data=error))

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



