#!/usr/bin/env pyhton3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2


class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_det')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        topic_name = '/car/car_camera/image_color'

        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            qos
        )

        self.image_saved = False
        self.save_path = "imagen_camara_test.jpg"

        self.bridge = CvBridge()
        self.get_logger().info(f'SignDetector iniciado, esperadando imagenes en {topic_name}')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
        except Exception as e:
            self.get_logger().info(f'Error de CvBridge: {e}')
            return
        
        if not self.image_saved:
            try:
                cv2.imwrite(self.save_path, cv_image)
                self.get_logger().warn(f'Img guardada en : {self.save_path}')
                self.image_saved = True

            except Exception as e:
                self.get_logger().error(f'No se pudo guardar la imagen: {e}')


        if cv_image is None:
            self.get_logger().info(f'Imagen recibida es None')
            return

        h, w, c = cv_image.shape
        self.get_logger().info(f'Imagen recibida con exito: {w}x{h} pixeles')




def main():
    rclpy.init(args = args)
    node = SignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Cerrando controlador señal')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
