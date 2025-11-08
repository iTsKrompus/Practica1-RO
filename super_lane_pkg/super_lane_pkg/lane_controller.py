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

        #controlador PID
        self.Kp = 0.002  # Ganancia proporcional
        self.Ki = 0.0001
        self.Kd = 0.001

        self.integral = 0.0
        self.integral_max = 400.0
        self.last_error = 0.0


    def error_callback(self, msg):
        error = float(msg.data)

        prop = self.Kp * error
        self.integral += error
        
        if self.integral > self.integral_max:
            self.integral = self.integral_max
        elif self.integral < -self.integral_max:
            self.integral = -self.integral_max

        derivative = self.Kd * (error - self.last_error)
        self.last_error = error

        angular = -(prop + self.integral + derivative)

        twist = Twist()
        twist.linear.x = 20.0  # velocidad constante
        twist.angular.z = angular
        self.cmd_pub.publish(twist)
        #self.get_logger().info(f'Controlando: error={error}, giro={angular:.2f}')

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
