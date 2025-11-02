import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('super_lane_pkg')

    # Ruta al URDF del coche
    robot_description_path = os.path.join(package_dir, 'resource', 'CitroenCZero.urdf')

    # Ruta al mundo de Webots
    world_path = os.path.join(package_dir, 'worlds', 'city_traffic.wbt')

    # Lanzador de Webots con el mundo
    webots = WebotsLauncher(
        world=world_path
    )

    # Controlador del coche
    car_driver = WebotsController(
        robot_name='car',  # Asegúrate de que el robot en Webots se llama "car"
        parameters=[
            {
                'robot_description': robot_description_path,
                'camera_names': ['road_camera'],
                'always_on_camera': True
            }
        ]
    )

    # Nodo de percepción
    lane_detector = Node(
        package='super_lane_pkg',
        executable='lane_detector',
        name='lane_detector',
        output='screen'
    )

    # Nodo de control
    lane_controller = Node(
        package='super_lane_pkg',
        executable='lane_controller',
        name='lane_controller',
        output='screen'
    )

    # Nodo de percepción de señales
    signal_det = Node(
        package='super_lane_pkg',
        executable='signal_det',
        name='signal_det',
        output='screen'
    )
    return LaunchDescription([
        webots,
        car_driver,
        lane_detector,
        lane_controller,
        signal_det,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])


