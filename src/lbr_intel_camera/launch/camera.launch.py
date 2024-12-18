import os
import yaml
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
	

def generate_launch_description():
	package_name = "lbr_intel_camera"
	package_dir = get_package_share_directory(package_name)
	

	camera_stream = Node(
		package=package_name,
		executable="stream_camera",
		parameters=[
			{"config_path": os.getenv("ROBOT_CONFIG")}
		],
		arguments=["MyName"]

	)

	return LaunchDescription([
		camera_stream
	])