import array
import rclpy
import numpy as np

from typing import List
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from webots_ros2_driver.ros2_supervisor import Supervisor
from controller import Motor, PositionSensor

from iiwa_interfaces.srv import ChangeTrajectory


class LBRController():
	def init(self, webots_node, properties):
		
		robot_name = properties["robotName"]  # Параметр получаемы с URDF файла
		self.__THRESHOLD = float(properties["threshold"])  # Коэфициент проверки достижения конечной точки

		# Настройки ROS
		qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
										durability=DurabilityPolicy.VOLATILE,
										depth=1)

		rclpy.init(args=None)
		self.__node = rclpy.create_node(f'{robot_name}_node')

		# Топик получения точек перемещения робота
		self.__node.create_subscription(JointTrajectory,
										f'{robot_name}/cmd_positions',
										self.__cmd_positions_callback,
										qos_profile)

		self.__robot_position_pub = self.__node.create_publisher(JointTrajectoryPoint,
																"cmd_point",
																qos_profile=qos_profile)
  
		# Топик по публикации текущей позиции виртуального робота
		self.__current_positions = self.__node.create_publisher(msg_type=JointTrajectoryPoint,
																topic=f"{robot_name}/current_positions/virtual",
																qos_profile=qos_profile)

		# Сервис для остановки робота
		self.__node.create_service(srv_type=Trigger,
									srv_name=f"{robot_name}/stop_move",
									callback=self.__change_state_callback)
		
		# TODO: Сервис о перестроении траектории перемещения (ВОЗМОЖНО ИСЧЕЗНЕТ)
		self.__node.create_service(srv_type=ChangeTrajectory,
							 		srv_name=f"{robot_name}/change_trajectory",
									callback=self.__change_trajectory_callback)

		# Настройки Webots
		self.__base_joint_names = ["lbr_A1", "lbr_A2", "lbr_A3",
								   "lbr_A4", "lbr_A5", "lbr_A6",
								   "lbr_A7"]
		
		self.__robot: Supervisor = webots_node.robot  # Модель робота с Webots
		self.__axes: List[Motor] = [self.__robot.getDevice(i) for i in self.__base_joint_names] # Получение всех двигателей
		self.__axes_sensors: List[PositionSensor] = [self.__robot.getDevice(f"{i}_sensor") for i in self.__base_joint_names] # Получение всех датчиков позиции
		
		# Включение работы сенсоров
		for sensor in self.__axes_sensors:
			sensor.enable(10)

		# Перемещение робота в 0 позицию
		for joint in self.__axes:
			joint.setPosition(0)

		self.__points = []
		self.__joint_names = []
		self.__current_sensor_positions = [sensor.getValue() for sensor in self.__axes_sensors]
		
		self.__curent_point_index = 0 # Текущий индекс позиции
		self.__reached_curent_point = True # Флаг достижения одной точки
		self.__stop_movement = False # Флаг остановки выполнения перемещения

	def __change_trajectory_callback(self,
								  	request: ChangeTrajectory.Request,
									response: ChangeTrajectory.Response) -> ChangeTrajectory.Response:
		"""Обработчик для изминения траектории робота"""
		
		try:
			if not self.__points:
				raise ValueError("Отсутсвуют точки для перемещения")

			current_index = self.__curent_point_index
			new_index = max(0, current_index - request.step)
			points = self.__points[current_index:new_index:-1]

			self.__curent_point_index = 0
			self.__reached_curent_point = True
			self.__points = points
			
			response.position = array.array('f', points[-1].positions)
			response.joint_names = self.__joint_names

			response.status = True
			response.message = "Trajectory changed successfully"

		except Exception as e:
			response.message = str(e)
			response.status = False

		return response

	def __change_state_callback(self, 
								request: Trigger.Request, 
								response: Trigger.Response) -> Trigger.Response:
		"""Остановка движения робота движения робота"""

		try:
			self.__stop_movement = True
			self.__reset_positions()
			response.success = True
			response.message = "The robot's movement is stopped"
		except Exception as e:
			response.success = False
			response.message = str(e)

		return response

	def __cmd_positions_callback(self, msg: JointTrajectory):
		"""Установка позиций перемещения робота

		Args:
			msg (JointTrajectory): Траектория перемещения
		"""
		if msg:
			self.__stop_movement = False
			self.__reset_positions(points=msg.points, 
									joint=msg.joint_names, 
									point_index=0, 
									reached_curent_point=True)
			

	def step(self):
		rclpy.spin_once(self.__node, timeout_sec=0)

		# Срабатывает когда мы отправили роботу команду на остановку
		if self.__stop_movement:
			for i, axes in enumerate(self.__axes):
				axes.setPosition(self.__current_sensor_positions[i])

		if self.__points and self.__curent_point_index < len(self.__points) and self.__reached_curent_point:
			point = self.__points[self.__curent_point_index]

			for i, joint in enumerate(self.__joint_names):
				if joint in self.__base_joint_names:
					pos_id = self.__base_joint_names.index(joint)
					self.__axes[pos_id].setPosition(point.positions[i])

			self.__reached_curent_point = False
   
			pos_msg = JointTrajectoryPoint(positions=point.positions) 
			self.__robot_position_pub.publish(pos_msg)


		# Проверка достижения текущей точки
		if not self.__reached_curent_point:
			current_point_move = self.__points[self.__curent_point_index]

			# Преобразуем текущие позиции сенсоров и текущую точку перемещения в массивы NumPy
			current_sensor_positions = np.array([self.__axes_sensors[self.__base_joint_names.index(joint)].getValue() for joint in self.__joint_names])
			target_positions = np.array(current_point_move.positions)

			# Вычисляем евклидово расстояние между текущими позициями сенсоров и текущей точкой перемещения
			distance = np.linalg.norm(current_sensor_positions - target_positions)

			if distance < self.__THRESHOLD:
				# TODO: Здесь должен быть код передачи координат реальному роботу 
				self.__reached_curent_point = True
				self.__curent_point_index += 1

				
		# Публикация текущего положения вертуального робота
		sensor_msg = JointTrajectoryPoint()
		self.__current_sensor_positions = [sensor.getValue() for sensor in self.__axes_sensors]
		sensor_msg.positions = self.__current_sensor_positions
		self.__current_positions.publish(sensor_msg)

		# Сброс точек перемещения если робот достиг конечной точки
		if self.__points:
			dist = np.array(self.__current_sensor_positions) - np.array(self.__points[-1].positions)
			if  np.linalg.norm(dist) < 0.01 and self.__curent_point_index != 0:
				self.__node.get_logger().info("Moving along the trajectory is over")
				self.__reset_positions()
		

	def __reset_positions(self,
						  points:list=[],
						  joint:list=[],
						  point_index:int=0,
						  reached_curent_point:bool=True) -> None:
		"""Возвращение всех исходных переменных в первоначальное состояние

		Args:
			points (list, optional): Точки перемещения. Defaults to [].
			joint (list, optional): Углы которые необходимо переместить. Defaults to [].
			point_index (int, optional): Текущий индекс. Defaults to 0.
			reached_curent_point (bool, optional): Флаг достижения текущей точки. Defaults to True.
		"""

		self.__points = points
		self.__joint_names = joint
		self.__curent_point_index  = point_index
		self.__reached_curent_point = reached_curent_point