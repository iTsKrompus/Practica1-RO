#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32 # 👈 Importar para recibir la velocidad

class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')
        
        # --- Publicador (Único que publica en /cmd_vel) ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # --- Suscripciones ---
        # 1. Suscripción al error de carril
        self.error_sub = self.create_subscription(
            Int32, 
            '/lane_error', 
            self.error_callback, 
            10
        )
        # 2. Suscripción al comando de velocidad
        self.speed_sub = self.create_subscription(
            Float32,
            '/speed_command',
            self.speed_callback,
            10
        )

        # --- Controlador PID (de tu código) ---
        self.Kp = 0.001
        self.Ki = 0.0
        self.Kd = 0.005
        self.integral = 0.0
        self.integral_max = 400.0
        self.last_error = 0.0
        
        # --- Estado del Vehículo ---
        self.MAX_SPEED = 25.0 # Velocidad por defecto (la que usabas)
        self.current_speed_target = self.MAX_SPEED
        self.current_angular_target = 0.0

        self.get_logger().info('LaneController (Cerebro) iniciado.')


    def speed_callback(self, msg):
        """Actualiza la velocidad objetivo cuando el SignDetector lo ordena."""
        self.get_logger().info(f'Nuevo objetivo de velocidad recibido: {msg.data:.1f}')
        self.current_speed_target = msg.data
        
        # Si la velocidad es 0 (STOP), reseteamos el PID
        if self.current_speed_target == 0.0:
            self.integral = 0.0
            
        # Publicar inmediatamente el cambio de velocidad (con el giro que ya teníamos)
        self.publish_command()


    def error_callback(self, msg):
        """Calcula el giro (PID) cuando el LaneDetector envía un error."""
        
        # 🚨 CORRECCIÓN: 
        # Hemos eliminado el 'if self.current_speed_target == 0.0:'.
        # El giro se calcula SIEMPRE, incluso si estamos frenando.
        
        error = float(msg.data)
        prop = self.Kp * error
        
        # Si la velocidad es 0, no acumulamos integral
        if self.current_speed_target != 0.0:
            self.integral += error
            if self.integral > self.integral_max: self.integral = self.integral_max
            elif self.integral < -self.integral_max: self.integral = -self.integral_max
        
        int_term = self.Ki * self.integral

        derivative = self.Kd * (error - self.last_error)
        self.last_error = error
        
        self.current_angular_target = -(prop + int_term + derivative)
        
        # Publicar el comando combinado
        self.publish_command()

    
    def publish_command(self):
        """Publica el estado actual (velocidad y giro) en /cmd_vel."""
        twist = Twist()
        twist.linear.x = self.current_speed_target
        twist.angular.z = self.current_angular_target
        
        self.cmd_pub.publish(twist)
        self.get_logger().info(f'Publicando: Vel={twist.linear.x:.1f}, Giro={twist.angular.z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Cerrando lane_controller (Cerebro)')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()