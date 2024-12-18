import os
import yaml
import launch
import argparse
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

from schemas import RobotShemas


def load_yaml(file_path):
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    package_dir = get_package_share_directory('iiwa_controller')
    robot_description_path = os.path.join(package_dir, 'config', 'LBRiiwa7R800.urdf')
    
    config_data = load_yaml(os.getenv("ROBOT_CONFIG"))
    robot_config = RobotShemas(**config_data["robot_settings"])

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'kuka_world.wbt')
    )
    
    lbr_driver = WebotsController(
        robot_name='LBRiiwa7R800',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        respawn=True
    )
    
    fri_driver = Node(
        package='iiwa_controller',
        executable='FRIcontroller',
        parameters=[
                    {"robot_ip": str(robot_config.robot_ip)},
                    {"robot_port": str(robot_config.robot_port)},
                ]
    )
    
    position_publisher = Node(
        package='iiwa_controller',
        executable='positionPublisher'
    )


    return LaunchDescription([
        # webots,
        # lbr_driver,
        fri_driver,
        # position_publisher,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=webots,
        #         on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        #     )
        # )
        
    ])