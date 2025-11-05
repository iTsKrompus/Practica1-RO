#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist

# --- CONSTANTES DE CINEMÁTICA Y CONTROL ---
# Estos valores son críticos para que el coche se mueva correctamente.
WHEEL_RADIUS = 0.28675      
MAX_WHEEL_VELOCITY = 35.0   # Velocidad máxima de la rueda en rad/s (límite común en Webots)
MAX_STEERING_ANGLE = 0.631  # Ángulo máximo de giro (delimitado en el PROTO)
STEERING_K = 0.5            # Factor de conversión de velocidad angular (w) a ángulo de dirección

class MyRobotDriver:
    def init(self, webots_node, properties):
        # 1. Inicialización de la clase base (¡OBLIGATORIO!)
        super().init(webots_node, properties) 
        
        self.__robot = webots_node.robot
        
        # 2. ADQUISICIÓN DE ACTUADORES (Nombres confirmados del PROTO)
        
        # Propulsión (Velocidad) -> Ruedas Traseras (motores de velocidad)
        self.__rear_left_motor = self.__robot.getDevice('rear left wheel')
        self.__rear_right_motor = self.__robot.getDevice('rear right wheel')
        
        # Dirección (Posición/Ángulo) -> Ruedas Delanteras (motores de posición)
        self.__front_left_motor = self.__robot.getDevice('front left wheel')
        self.__front_right_motor = self.__robot.getDevice('front right wheel')

        # 3. HABILITACIÓN DE SENSORES (Cámaras)
        time_step = int(self.__robot.getBasicTimeStep())
        
        self.__road_camera = self.__robot.getDevice('road_camera') 
        self.__car_camera = self.__robot.getDevice('car_camera')   
        self.__road_camera.enable(time_step)
        self.__car_camera.enable(time_step)
        
        # 4. CONFIGURACIÓN INICIAL DE ACTUADORES
        # Propulsión: Modo velocidad infinita (movimiento) y velocidad 0
        self.__rear_left_motor.setPosition(float('inf'))
        self.__rear_right_motor.setPosition(float('inf'))
        self.__rear_left_motor.setVelocity(0.0)
        self.__rear_right_motor.setVelocity(0.0)

        # Dirección: Modo posición (ángulo) y posición inicial 0
        self.__front_left_motor.setPosition(0.0)
        self.__front_right_motor.setPosition(0.0)

        # 5. CONFIGURACIÓN ROS 2 (¡CORRECCIÓN CRÍTICA!)
        self.__target_twist = Twist()
        
        # 🚨 Accede al nodo ROS 2 ya creado por el WebotsDriver
        self.__node = rclpy.create_node('my_robot_driver') 
        
        # Suscripción al tópico de comandos de velocidad del LaneController
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

        self.__node.get_logger().info('Driver Citroen C-Zero iniciado. Esperando comandos en /cmd_vel.')


    def __cmd_vel_callback(self, twist):
        """Recibe el comando Twist publicado por el LaneController."""
        self.__target_twist = twist

    def step(self):
        """Se llama en cada paso de simulación. Aplica los comandos Twist a los motores."""
        
        # NECESARIO: Procesa mensajes pendientes de ROS (incluyendo el cmd_vel)
        rclpy.spin_once(self.__node, timeout_sec=0) 

        # Obtener comandos Twist
        v = self.__target_twist.linear.x  # Velocidad lineal (Avance/Retroceso)
        w = self.__target_twist.angular.z # Velocidad angular (Giro)

        # 1. Control de Propulsión (Ruedas Traseras)
        # Ambas ruedas traseras se mueven a la misma velocidad (cinemática de coche).
        target_velocity = min(abs(v) / WHEEL_RADIUS, MAX_WHEEL_VELOCITY)
        direction = 1 if v >= 0 else -1 
        
        self.__rear_left_motor.setVelocity(target_velocity * direction)
        self.__rear_right_motor.setVelocity(target_velocity * direction)

        # 2. Control de Dirección (Ruedas Delanteras)
        # Convierte w (giro) a un ángulo (posición) para los actuadores delanteros.
        steering_angle = w * STEERING_K 
        
        # Limita el ángulo
        steering_angle = max(-MAX_STEERING_ANGLE, min(steering_angle, MAX_STEERING_ANGLE))

        # Asigna la posición (ángulo) a los motores de dirección
        self.__front_left_motor.setPosition(steering_angle)
        self.__front_right_motor.setPosition(steering_angle)