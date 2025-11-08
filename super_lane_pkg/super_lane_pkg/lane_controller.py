#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')
        self.subscription = self.create_subscription(Int32, '/lane_error', self.error_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        #--- Controlador PID ---
        # Estas son las ganancias "suaves" que discutimos, 
        # pero puedes ajustarlas para la nueva velocidad de 10.0
        self.Kp = 0.001  # Ganancia proporcional
        self.Ki = 0.0    # Ganancia integral (0 para empezar)
        self.Kd = 0.005  # Ganancia derivativa (amortiguador)

        self.integral = 0.0
        self.integral_max = 400.0
        self.last_error = 0.0


    def error_callback(self, msg):
        error = float(msg.data)

        # --- CÁLCULO PID ---
        
        # Proporcional (presente)
        prop = self.Kp * error

        # Integral (pasado)
        self.integral += error
        # Límite anti-windup
        if self.integral > self.integral_max:
            self.integral = self.integral_max
        elif self.integral < -self.integral_max:
            self.integral = -self.integral_max
        # 🚨 CORRECCIÓN: Multiplicamos por la ganancia Ki
        int_term = self.Ki * self.integral

        # Derivativo (futuro)
        derivative = self.Kd * (error - self.last_error)
        self.last_error = error

        # --- CÁLCULO FINAL ---
        # Sumamos los tres términos
        angular = -(prop + int_term + derivative)

        twist = Twist()
        twist.linear.x = 10.0  # 🚨 VELOCIDAD FIJADA A 10.0
        twist.angular.z = angular
        
        self.cmd_pub.publish(twist)
        self.get_logger().info(f'Error={error:.0f}, Giro={angular:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Cerrando lane_controller (PID)')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()