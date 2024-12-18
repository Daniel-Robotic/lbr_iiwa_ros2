import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	robot_description = ParameterValue(
            Command([
                  "xacro ",
                  os.path.join(get_package_share_directory("robot_control"), "urdf", "iiwa7.urdf.xacro")
			]),
			value_type=str
	)

	controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[os.path.join(get_package_share_directory("robot_control"), "config", "robot_controller.yaml")],
        output="screen"
    )

	robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

	joint_state_broadcaster_spawner = Node(
		package="controller_manager",
		executable="spawner",
		arguments=["joint_state_broadcaster", 
				"--controller-manager", 
				"/controller_manager"]
	)

	iiwa_controller_spawner = Node(
		package="controller_manager",
		executable="spawner",
		arguments=["forward_position_controller", 
				"--controller-manager", 
				"/controller_manager"]
	)

	return LaunchDescription([
        controller_manager,
		robot_state_publisher,
		joint_state_broadcaster_spawner,
		iiwa_controller_spawner
	])