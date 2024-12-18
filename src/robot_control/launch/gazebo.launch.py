import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution, Command, FindExecutable
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable, SetEnvironmentVariable, IncludeLaunchDescription, SetLaunchConfiguration


def generate_launch_description():
	
	robot_name = "iiwa7"
	pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim') # Название пакета с gazebo
	pkg_descriptions = get_package_share_directory("robot_control") # Название пакета где располагается мой URDF файл
	gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py']) # Для вызова gazebo
	rviz_config = os.path.join(pkg_descriptions, "config", "rviz_bases.rviz")

	# Поиск URDF модели робота
	model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(pkg_descriptions, "urdf", "iiwa7.urdf.xacro"),
        description="Absolute path to the robot URDF file"
    )

	robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

	# Аргумент для выбора мира
	world_arg = DeclareLaunchArgument(
        name="world",
        default_value=os.path.join(pkg_descriptions, "sdf", "my_world"),
        description='World to load into Gazebo'
    )

	# Конфигурация для пути к файлу мира
	config = SetLaunchConfiguration(
        name="world_file",
        value=[LaunchConfiguration("world"), TextSubstitution(text=".sdf")]
    )

	# Менеджер контроллеров
	controller_manager_broadcaster = Node(
		package='controller_manager',
		executable='spawner',
		name='controller_manager_broadcaster',
		arguments=['joint_state_broadcaster']
	)

	controller_manager_robot = Node(
		package='controller_manager',
		executable='spawner',
		name='controller_manager_robot',
		arguments=['joint_trajectory_controller']
	)

	my_manager_controller = Node(
		package='controller_manager',
		executable='spawner',
		name='my_controller_manager',
		arguments=['my_hardware_controller']
	)

	# Запуск Gazebo
	start_gazebo = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            "gz_args": [PathJoinSubstitution([pkg_ros_gz_sim, 
											  'worlds', 
											  LaunchConfiguration("world_file")
											  ]),
						' -r'
						],
            'on_exit_shutdown': 'True'
        }.items()
    )
	
    # Запуск узла robot_state_publisher
	robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
            arguments=[]
		)

	# Спавн робота
	spawn_entity = Node(
		package='ros_gz_sim', 
		executable='create',
		parameters=[{
					'name': robot_name, # Change names
					'topic': '/robot_description',
					# 'file': os.path.join(pkg_descriptions, "urdf", "robot.sdf")
					}],
		output='screen',
		# respawn=True
	)

    # Запуск узла spawn_entity для загрузки робота в Gazebo
	rviz_node = Node(
		package="rviz2",
		executable="rviz2",
		name="rviz_gui",
		arguments=["-d", rviz_config]
    )
	
	# Запуск узла ros_gz_bridge
	bridge_params = os.path.join(pkg_descriptions, "config", "gz_bridge.yaml")
	ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
		arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )

	return LaunchDescription([
		SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg_descriptions),
		model_arg,
		world_arg,
		config,
		robot_state_publisher,
		controller_manager_broadcaster,
		controller_manager_robot,
		# my_manager_controller,
		start_gazebo,
		spawn_entity,
		# rviz_node,
		ros_gz_bridge,
		
	])