import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
	pkg_descriptions = get_package_share_directory("robot_control")
	urdf_file = os.path.join(pkg_descriptions, "urdf", "iiwa7.urdf.xacro")
	rviz_config = os.path.join(pkg_descriptions, "config", "rviz_bases.rviz")

	model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg_descriptions, "urdf", "iiwa7.urdf.xacro"),
        description="Absolute path to the robot URDF file"
    )

	urdf_content = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))
	# with open(urdf_file, 'r') as file:
	# 	urdf_content = file.read()

	joint_state_publisher_node = Node(
		package='joint_state_publisher_gui',
		executable='joint_state_publisher_gui',
		name="JSP"
	)

	robot_state_publisher_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		name="RSP",
		parameters=[{"robot_description": urdf_content}]
	)

	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz_gui",
		arguments=["-d", rviz_config]
	)

	return LaunchDescription([
		model_arg,
		robot_state_publisher_node,
		joint_state_publisher_node,
		rviz_node
	])


if __name__ == "__main__":
	generate_launch_description()