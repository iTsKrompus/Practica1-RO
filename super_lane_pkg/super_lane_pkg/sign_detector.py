#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import os

# 1. --- Imports AÑADIDOS ---
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32 # 👈 Importar para publicar la velocidad

class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_det')

        # --- Parámetros de Velocidad ---
        self.MAX_SPEED = 25.0  # (Debe coincidir con el LaneController)
        self.YIELD_SPEED = self.MAX_SPEED/2 # (Mitad de velocidad)
        self.STOP_DURATION = 6.0
        self.STOP_SPEED = 0.001
        
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
        
        # 2. --- NUEVO PUBLICADOR de Velocidad ---
        self.speed_pub = self.create_publisher(Float32, '/speed_command', 10)

        self.bridge = CvBridge()
        self.templates = []
        self.load_templates()

        self.stop_timer = None
        self.is_stopped = False


    def load_templates(self):
        """Carga las plantillas de señales desde los archivos."""
        try:
            package_name = 'super_lane_pkg'
            share_dir = get_package_share_directory(package_name)

            # 3. --- RUTAS CORREGIDAS (Usando el path absoluto) ---
            path_stop = os.path.join(share_dir, 'templates', 'stop.png')
            path_ceda = os.path.join(share_dir, 'templates', 'yield.png')
            path_vel = os.path.join(share_dir, 'templates', 'speed.png')
            
            template_stop = cv2.imread(path_stop, cv2.IMREAD_COLOR)
            template_ceda = cv2.imread(path_ceda, cv2.IMREAD_COLOR)
            template_vel = cv2.imread(path_vel, cv2.IMREAD_COLOR)

            if template_stop is not None:
                self.templates.append(("STOP", template_stop, 0.45))
            else:
                self.get_logger().error(f"¡FALLO AL CARGAR! No se pudo leer el archivo: {path_stop}")
            
            if template_ceda is not None:
                self.templates.append(("Ceda", template_ceda, 0.6))
            else:
                self.get_logger().error(f"¡FALLO AL CARGAR! No se pudo leer el archivo: {path_ceda}")

            if template_vel is not None:
                self.templates.append(("Velocidad", template_vel, 0.6))
            else:
                self.get_logger().error(f"¡FALLO AL CARGAR! No se pudo leer el archivo: {path_vel}")
            
            self.get_logger().info(f'{len(self.templates)} plantillas de señales cargadas.')

        except Exception as e:
            self.get_logger().error(f'Error cargando plantillas: {e}')
    
    def image_callback(self, msg):
        if self.is_stopped:
            return
            
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().info(f'Error de CvBridge: {e}')
            return
        
        detected_sign = None
        max_confidence = 0.0

        for (name, template, threshold) in self.templates:
            res = cv2.matchTemplate(frame, template, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            
            if max_val > threshold and max_val > max_confidence:
                max_confidence = max_val
                detected_sign = name

        # 4. --- LÓGICA DE PUBLICACIÓN DE VELOCIDAD ---
        speed_msg = Float32()

        if detected_sign == "STOP":
            self.get_logger().warn("¡STOP DETECTADO! Parando 1 segundo.")
            self.is_stopped = True
            speed_msg.data =  self.STOP_SPEED
            self.speed_pub.publish(speed_msg)
            
            if self.stop_timer is None:
                self.stop_timer = self.create_timer(self.STOP_DURATION, self.on_stop_timer_complete)
        
        elif detected_sign == "Ceda":
            self.get_logger().info("Ceda detectado. Reduciendo a media velocidad.")
            speed_msg.data = self.YIELD_SPEED
            self.speed_pub.publish(speed_msg)

        elif detected_sign == "Velocidad":
            self.get_logger().info("Señal de Velocidad detectada. Reanudando velocidad máxima.")
            speed_msg.data = self.MAX_SPEED
            self.speed_pub.publish(speed_msg)
        
        # (Si no detecta nada, no publica nada, y el LaneController
        # mantendrá la última velocidad que tuviera)

    def on_stop_timer_complete(self):
        """Se llama 1 segundo después de ver un STOP."""
        self.get_logger().warn("1 segundo completado. Reanudando marcha.")
        self.is_stopped = False
        
        speed_msg = Float32()
        speed_msg.data = self.MAX_SPEED
        self.speed_pub.publish(speed_msg)

        if self.stop_timer is not None:
            self.stop_timer.cancel()
            self.stop_timer = None
        
def main(args=None):
    rclpy.init(args=args)
    node = SignDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Cerrando SignDetector')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()