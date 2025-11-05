import rclpy
from geometry_msgs.msg import Twist
import math 


# --- CONSTANTES ---
WHEEL_RADIUS = 0.28675      
MAX_STEERING_ANGLE = 0.631  
STEERING_K = 0.5            
MAX_WHEEL_VELOCITY = 35.0   

# 2. HEREDAR DE WEBOTSDRIVER
class MyRobotDriver:
    def init(self, webots_node, properties):
        # 3. INICIALIZAR LA CLASE BASE (OBLIGATORIO)
        self.__robot = webots_node.robot
        
        time_step = int(self.__robot.getBasicTimeStep())
        
        # 4. OBTENER EL NODO (NO CREARLO)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.get_logger().info('Driver C-Zero (4 Ruedas) INICIADO CORRECTAMENTE')

        # --- Motores ---
        self.__rear_left_motor = self.__robot.getDevice('rear left wheel')
        self.__rear_right_motor = self.__robot.getDevice('rear right wheel')
        self.__front_left_motor = self.__robot.getDevice('front left wheel')
        self.__front_right_motor = self.__robot.getDevice('front right wheel')

        self.__rear_left_motor.setPosition(float('inf'))
        self.__rear_right_motor.setPosition(float('inf'))
        self.__rear_left_motor.setVelocity(0)
        self.__rear_right_motor.setVelocity(0)
        self.__front_left_motor.setPosition(0)
        self.__front_right_motor.setPosition(0)

        # --- Cámaras ---
        self.__road_camera = self.__robot.getDevice('road_camera') 
        self.__car_camera = self.__robot.getDevice('car_camera')   
        self.__road_camera.enable(time_step)
        self.__car_camera.enable(time_step)

        # --- ROS 2 ---
        self.__target_twist = Twist()
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.get_logger().info('Suscripción a /cmd_vel CREADA')


    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        # 5. NO SE NECESITA spin_once() (WebotsDriver lo hace)
        rclpy.spin_once(self.__node, timeout_sec=0)
        v = self.__target_twist.linear.x
        w = self.__target_twist.angular.z

        target_velocity = min(abs(v) / WHEEL_RADIUS, MAX_WHEEL_VELOCITY)
        direction = 1 if v >= 0 else -1 
        self.__rear_left_motor.setVelocity(target_velocity * direction)
        self.__rear_right_motor.setVelocity(target_velocity * direction)

        steering_angle = w * STEERING_K 
        steering_angle = max(-MAX_STEERING_ANGLE, min(steering_angle, MAX_STEERING_ANGLE))
        self.__front_left_motor.setPosition(steering_angle)
        self.__front_right_motor.setPosition(steering_angle)
