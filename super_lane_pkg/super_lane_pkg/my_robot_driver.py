import rclpy
from geometry_msgs.msg import Twist
import math 


WHEEL_RADIUS = 0.28675      
MAX_STEERING_ANGLE = 0.631  
STEERING_K = 0.5            
MAX_WHEEL_VELOCITY = 35.0   

class MyRobotDriver:
    def init(self, webots_node, properties):
        rclpy.init(args=None)
        
        self.__robot = webots_node.robot
        time_step = int(self.__robot.getBasicTimeStep())
        
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.get_logger().info('Driver C-Zero (4 Ruedas) INICIADO CORRECTAMENTE')

        self.__road_camera = self.__robot.getDevice('road_camera') 
        self.__car_camera = self.__robot.getDevice('car_camera')   
       
        self.__road_camera.enable(time_step)
        self.__car_camera.enable(time_step)
        self.__node.get_logger().info('Camaras "road_camera" y "car_camera" actuvadas')


        self.__target_twist = Twist()
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.get_logger().info('Suscripción a /cmd_vel CREADA')

        self.__robot.setCruisingSpeed(0.0)
        self.__robot.setSteeringAngle(0.0)
        
        self.__node.get_logger().info('API "Car" inicializada  en 0.0 m/s y 0.0 rad.')

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        v = self.__target_twist.linear.x
        desired_angle = self.__target_twist.angular.z
        
        steering_angle = max(-MAX_STEERING_ANGLE, min(desired_angle, MAX_STEERING_ANGLE))

        self.__robot.setCruisingSpeed(v)
        self.__robot.setSteeringAngle(steering_angle)
        
