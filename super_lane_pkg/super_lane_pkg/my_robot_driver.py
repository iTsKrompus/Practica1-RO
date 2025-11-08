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

    def _get_device_or_crash(self, name):
        device = self.__robot.getDevice(name)
        if device is None:
            self.__node.get_logger().fatal(f'CRITICO: No se pudo encontrar el dispositivo Webots: "{name}"')
            raise RuntimeError(f'Dispositivo "{name}" no encontrado. Revisa el archivo .wbt.')
        return device

    def init(self, webots_node, properties):
        #Cambios: Faltaba iniciar rclpy
        try:
            rclpy.init(args=None)
        except Exception as e:
            pass
        
        self.__robot = webots_node.robot
        time_step = int(self.__robot.getBasicTimeStep())
        
        #Creamos el nodo
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.get_logger().info('Driver C-Zero (4 Ruedas) INICIADO CORRECTAMENTE')

        # --- Motores ---
        try:
            self.__rear_left_motor = self._get_device_or_crash('rear left wheel')
            self.__rear_right_motor = self._get_device_or_crash('rear right wheel')

            self.__front_left_motor = self._get_device_or_crash('front left wheel')
            self.__front_right_motor = self._get_device_or_crash('front right wheel')

            self.__road_camera = self._get_device_or_crash('road_camera') 
            self.__car_camera = self._get_device_or_crash('car_camera')   

            self.__front_left_steering = None # Se mantiene la variable, pero sin adquirir
            self.__front_right_steering = None

        #self.__rear_left_motor.setPosition(float('inf'))
        #self.__rear_right_motor.setPosition(float('inf'))
        #self.__rear_left_motor.setVelocity(0)
        #self.__rear_right_motor.setVelocity(0)
        #self.__front_left_motor.setPosition(0)
        #self.__front_right_motor.setPosition(0)
        except RuntimeError:
            return
        except Exception as a:
            self.__node.get_logger().error(f'Fallo inesperado al obtener motores : {a}')
            raise a

        self.__rear_left_motor.setPosition(float('inf'))
        self.__rear_right_motor.setPosition(float('inf'))

        self.__rear_left_motor.setVelocity(0)
        self.__rear_right_motor.setVelocity(0)
        self.__front_left_motor.setPosition(0)
        self.__front_right_motor.setPosition(0)


        # --- Cámaras ---
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

        #Linea para debuggear
        self.__node.get_logger().info(f'Recibido V={v:.2f}, W={w:.2f}')

        #target_velocity = min(abs(v) / WHEEL_RADIUS, MAX_WHEEL_VELOCITY)
        #direction = 1 if v >= 0 else -1 
        #self.__rear_left_motor.setVelocity(target_velocity * direction)
        #self.__rear_right_motor.setVelocity(target_velocity * direction)

        #steering_angle = w * STEERING_K 
        #steering_angle = max(-MAX_STEERING_ANGLE, min(steering_angle, MAX_STEERING_ANGLE))
        #self.__front_left_motor.setPosition(steering_angle)
        #self.__front_right_motor.setPosition(steering_angle)

        target_velocity = v / WHEEL_RADIUS
        target_velocity = max(-MAX_WHEEL_VELOCITY, min(target_velocity, MAX_WHEEL_VELOCITY))

        #self.__rear_left_motor.setVelocity(target_velocity)
        #self.__rear_right_motor.setVelocity(target_velocity)

        self.__rear_left_motor.setVelocity(target_velocity)
        self.__rear_right_motor.setVelocity(target_velocity)

        steering_angle = w * STEERING_K
        steering_angle = max(-MAX_STEERING_ANGLE, min(steering_angle, MAX_STEERING_ANGLE))

        self.__front_left_motor.setPosition(steering_angle)
        self.__front_right_motor.setPosition(steering_angle)
        
