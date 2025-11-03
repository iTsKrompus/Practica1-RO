#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class LaneController(Node):
    def __init__(self):
        super().__init__('lane_controller')
        self.subscription = self.create_subscription(Int32, '/lane_error', self.error_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.Kp = 0.01  # Ganancia proporcional

    def error_callback(self, msg):
        error = msg.data
        angular = -self.Kp * error

        twist = Twist()
        twist.linear.x = 3.0  # velocidad constante
        twist.angular.z = angular

        self.cmd_pub.publish(twist)
        self.get_logger().info(f'Controlando: error={error}, giro={angular:.2f}')
def main(args=None):
    rclpy.init(args=args)
    node = LaneController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
