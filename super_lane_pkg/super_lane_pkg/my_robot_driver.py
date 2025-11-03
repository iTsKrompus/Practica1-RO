import rclpy
from geometry_msgs.msg import Twist

# Parámetros físicos del vehículo (ajustados al URDF)
HALF_DISTANCE_BETWEEN_WHEELS = 0.635  # Distancia lateral entre ruedas traseras / 2
WHEEL_RADIUS = 0.28675  # Radio de las ruedas traseras

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # Obtener motores traseros por nombre exacto del URDF
        self.__left_motor = self.__robot.getDevice('rear left wheel motor')
        self.__right_motor = self.__robot.getDevice('rear right wheel motor')

        # Configurar motores en modo velocidad
        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0.0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0.0)

        # Inicializar Twist objetivo
        self.__target_twist = Twist()

        # Crear nodo ROS 2 embebido
        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')

        # Suscribirse a /cmd_vel
        self.__node.create_subscription(
            Twist,
            '/cmd_vel',
            self.__cmd_vel_callback,
            1
        )

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        # Procesar mensajes ROS 2
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Extraer velocidades lineal y angular
        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        # Cinemática diferencial: convertir a velocidades de rueda
        left_velocity = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        right_velocity = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        # Aplicar velocidades a los motores
        self.__left_motor.setVelocity(left_velocity)
        self.__right_motor.setVelocity(right_velocity)
