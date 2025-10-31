from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Driver que conecta Webots con ROS 2
        Node(
            package='webots_ros2_driver',
            executable='driver',
            name='car_driver',
            output='screen',
            parameters=[{
                'robot_description': '/home/hugo/Documentos/RO/Práctica1/CitroenCZero.urdf',
                'camera_names': ['road_camera'],
            }]
        ),
        # Nodo de percepción
        Node(
            package='super_lane_pkg',
            executable='lane_detector',
            name='lane_detector',
            output='screen'
        ),
        # Nodo de control
        Node(
            package='super_lane_pkg',
            executable='lane_controller',
            name='lane_controller',
            output='screen'
        )
    ])
