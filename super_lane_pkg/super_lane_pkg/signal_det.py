#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import os

from ament_index_python.packages import get_package_share_directory


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

        self.bridge = CvBridge()
        self.last_cv_image = None

        self.detection_image_saved = False
        self.log_once = False
        

        home_dir = os.path.expanduser("~")
        self.save_path = os.path.join(home_dir, "imagen_camara_webots.png")
        self.det_path = os.path.join(home_dir, "ultima_deteccion.png")

        self.templates = []
        self.load_templates()


    def load_templates(self):
        try:
            package_name = 'super_lane_pkg' # CAMBIAR ESTO SI CAMBIAMOS EL NOMBRE DEL PAQUETE
            #PATHS DE LAS IMAGENES
            path_stop = "templates/stop.png"
            path_ceda = "templates/yield.png"
            path_vel = "templates/speed.png"
            
            template_stop = cv2.imread(path_stop, cv2.IMREAD_COLOR)
            template_ceda = cv2.imread(path_ceda, cv2.IMREAD_COLOR)
            template_vel = cv2.imread(path_vel, cv2.IMREAD_COLOR)

            if template_stop is not None:
                self.templates.append(("STOP", template_stop, 0.6)) # Se define el umbral
            
            if template_ceda is not None:
                self.templates.append(("Ceda", template_ceda, 0.6))

            if template_vel is not None:
                self.templates.append(("Velocidad", template_vel, 0.6))

        except Exception as e:
            self.get_logger().error(f'Error cargando plantillas: {e}')



    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
        except Exception as e:
            self.get_logger().info(f'Error de CvBridge: {e}')
            return
        
        if cv_image is None:
            return

        self.last_cv_image = cv_image.copy()

        if not self.log_once:
            h, w, c = cv_image.shape
            self.get_logger().info(f'Imagen recibida con exito: {w}x{h} pixeles')
            self.log_once = True

        self.detect_signals(cv_image)


    def detect_signals(self, frame):
        detection_made = False
        for (name, template, threshold) in self.templates:
            res = cv2.matchTemplate(frame, template, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            if max_val > threshold:
                (h, w) = template.shape[:2]
                top_left = max_loc
                bottom_right = (top_left[0] + w, top_left[1] + h)
                cv2.rectangle(frame, top_left, bottom_right, (0,0,255), 3)
                detection_made = True

        if detection_made and not self.detection_image_saved:
            try:
                cv2.imwrite(self.det_path, frame)
                self.get_logger().warn(f'¡Imagen con detección guardada en: {self.det_path}!')
                self.detection_image_saved = True # Ponemos el flag a True
            except Exception as e:
                self.get_logger().error(f'No se pudo guardar la imagen de detección: {e}')     

        


def main(args=None):
    rclpy.init(args=args)
    node = SignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.last_cv_image is not None:
            try:
                cv2.imwrite(node.save_path, node.last_cv_image)
            except Exception as e:
                node.get_logger().error(f'No se puedo guardar la ultima imagen: {e}')


        node.get_logger().info('Cerrando controlador señal')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
