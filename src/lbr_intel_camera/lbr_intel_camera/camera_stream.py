import os
import cv2
import yaml
import rclpy
import shutil
import logging
import rclpy.clock
import numpy as np

from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_srvs.srv import Trigger
from lbr_intel_camera_interface.msg import HumanDetection
from lbr_intel_camera_interface.srv import IntelCameraInformation, ChangeProfile

from lbr_intel_camera.utils.CalibartionUtils import find_calibration_template
from lbr_intel_camera.utils.DepthCameraWrapper import DepthCameraWrapper, DepthCameraInformation
from lbr_intel_camera.utils import load_yaml

from lbr_intel_camera.schemas import CameraSchemas


class CameraStream(Node):
	# TODO: Когда добавиться распознование, сделать отдельный поток который будет славть растояние до каждой распознанной точки
	def __init__(self):

		super().__init__(f"{os.getenv('CAMERA_NAME')}_node")

		# TODO: Изменить на пустое поле
		self.declare_parameter("config_path", value="./config.yaml")
		self.__config = load_yaml(self.get_parameter("config_path").value)

		camera_config = CameraSchemas.CameraSchema(**self.__config["camera_settings"])

		width = camera_config.width
		height = camera_config.height
		fps = camera_config.fps

		self.model = model = YOLO("yolo11s-pose.engine")

		self.__camera_name = os.getenv('CAMERA_NAME')
		self.__flip_h = camera_config.flip_horizontally
		self.__flip_v = camera_config.flip_vertically
		self.__nn_state = camera_config.nn_state

		self.__calibration_view_mode = False

		# Настройка логирования
		logging.basicConfig(
			level=logging.INFO,
			format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
			handlers=[
				logging.FileHandler(f"{self.__camera_name}.log"),
				logging.StreamHandler()
			]
		)
		self._logger = logging.getLogger(self.get_name())

		# Настройки ROS
		qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
								 durability=DurabilityPolicy.VOLATILE,
								 depth=1)

		self.get_logger().info(f"Camera position `{self.__camera_name}` configuration...")

		# Настройка камеры
		self.__camera = DepthCameraWrapper(width=width,
										   height=height,
										   fps=fps)

		# TODO: Подумать как будет реализовано распознование ключевых точек
		# Стоит добавить в этот модуль

		# Объявление сервисов и публикаторов
		self.get_logger().info("Inicializing services and publishing structure...")

		# Публикатроы для отправки данных об изображении
		# TODO: Если сильно будет нагружаться система то убираем отправку картинок, и просто сохраняем в папку
		self.__camera_stream_rgb_publisher = self.create_publisher(msg_type=CompressedImage,
																	topic=f"camera/{self.__camera_name}/rgb/raw",
																	qos_profile=qos_profile)
		self.__camera_stream_nn_publisher = self.create_publisher(msg_type=HumanDetection,
																  topic=f"camera/{self.__camera_name}/human_pose",
																  qos_profile=qos_profile)

#   TODO: Полностью избавиться от отправки глубины (картинки), а только точки отправлять
		# self.__camera_stream_depth_publisher = self.create_publisher(msg_type=CompressedImage,
		# 													  		topic=f"camera/{self.__camera_name}/depth/raw",
		# 															qos_profile=qos_profile)
		
		self.__camera_stream_timer = self.create_timer(timer_period_sec=1/fps,
													   callback=self.__camera_stream_callback)
		

		# Инициализация сервисов

		# Получение информации с камеры
		self.__camera_information_service = self.create_service(srv_type=IntelCameraInformation,
														  		srv_name=f"camera/{self.__camera_name}/camera_information",
														  		callback=self.__get_information_callback)
		
		# Сервис смены профиля камеры
		self.__change_camera_profile_service = self.create_service(srv_type=ChangeProfile,
																	srv_name=f"camera/{self.__camera_name}/change_profile",
																 	callback=self.__change_camera_profile_callback)
		
		# Сервис смены предпросмотра калибровочного шаблона
		self.__change_camera_preview_calibration_service = self.create_service(srv_type=Trigger,
																			   srv_name=f"camera/{self.__camera_name}/change_preview_calibration",
																			   callback=self.__change_camera_preview_calibration_callback)

		# Сервим включения нейронной сети
		self.__change_neural_network_mode_service = self.create_service(srv_type=Trigger,
																		srv_name=f"camera/{self.__camera_name}/change_neural_network_mode",
																		callback=self.__change_neural_network_mode_callback)
		
		self.get_logger().info(f"Start camera {self.__camera_name} stream")


	def __camera_stream_callback(self):
		
		depth_image, color_image = self.__camera.get_aligned_images()

		if self.__flip_h:
			color_image = cv2.flip(color_image, 0)
			depth_image = cv2.flip(depth_image, 0)

		if self.__flip_v:
			color_image = cv2.flip(color_image, 1)
			depth_image = cv2.flip(depth_image, 1)

  
		if self.__nn_state and not self.__calibration_view_mode:
			results = self.model(color_image)

			# Обрабатываем результаты
			for result in results:
				boxes = result.boxes
				keypoints = result.keypoints

				# Проверяем, есть ли обнаруженные люди
				if len(boxes) > 0:
					# Берем только первого человека (самый уверенный)
					box = boxes[0]
					keypoints = keypoints[0]

					# Создаем сообщение
					msg = HumanDetection()

					# Заполняем bounding box
					msg.x = float(box.xywh[0][0].item())  # x центра
					msg.y = float(box.xywh[0][1].item())  # y центра
					msg.width = float(box.xywh[0][2].item())  # ширина
					msg.height = float(box.xywh[0][3].item())  # высота

					# Заполняем confidence
					msg.confidence = float(box.conf.item())

					# Заполняем keypoints
					msg.keypoints = []
					for kp in keypoints.xy[0]:
						msg.keypoints.extend([float(kp[0].item()), float(kp[1].item())])

					self.__camera_stream_nn_publisher.publish(msg)


		if self.__calibration_view_mode and not self.__nn_state:
			# TODO: size необходимо передавать
			status, color_image = find_calibration_template(frame=color_image, size=(6, 9))

		bridge = CvBridge()		

		self.__camera_stream_rgb_publisher.publish(bridge.cv2_to_compressed_imgmsg(color_image))
		# self.__camera_stream_depth_publisher.publish(bridge.cv2_to_compressed_imgmsg(depth_image, dst_format="png"))

	# Описание сервисов
	def __get_information_callback(self, 
								   request: IntelCameraInformation.Request, 
								   response: IntelCameraInformation.Response) -> IntelCameraInformation.Response:
		self.get_logger().info(f"Get camera {self.__camera_name} information")

		camera_info: DepthCameraInformation = self.__camera.get_camera_information()

		response.device_name = camera_info.device_name
		response.product_line = camera_info.product_line
		response.serial_number = camera_info.serial_number
		response.firmware_version = camera_info.firmware_version
		response.usb_type = camera_info.usb_type
		response.color_profile = camera_info.color_profile
		response.depth_profile = camera_info.depth_profile

		return response
	
	def __change_camera_profile_callback(self, 
									  	request: ChangeProfile.Request, 
									  	response: ChangeProfile.Response) -> ChangeProfile.Response:
		self.__camera_stream_timer.cancel()

		self.get_logger().info(f"Change camera {self.__camera_name} profile")

		status = self.__camera.change_camera_profile(width=request.width,
											   		 height=request.height,
													 fps=request.fps)
		
		self.__camera_stream_timer.reset()
		response.status

		if status:
			response.message = "The camera profile has been successfully changed"
			self.get_logger().info(response.message)

		else:
			response.message = "The camera profile has not been successfully changed"
			self.get_logger().error(response.message)

		return response

	# Смена типа предпросмотра:
	def __change_camera_preview_calibration_callback(self, 
												  	request: Trigger.Request, 
												  	response: Trigger.Response) -> Trigger.Response:
		
		response.message = "Unexpected error"
		response.success = False

		if not self.__calibration_view_mode and self.__nn_state:
			response.message = "It is not possible to enable a preview of the calibration template while the neural network is running"
			response.success = False


		elif not self.__calibration_view_mode:
			self.__calibration_view_mode = True
			response.message = "The calibration template preview has been successfully enabled"
			response.success = True

	
		else:
			self.__calibration_view_mode = False
			response.message = "The calibration template preview has been successfully disabled"
			response.success = True

		self.get_logger().info(response.message)

		return response

	# Включение нейронной сети:
	def __change_neural_network_mode_callback(self, 
										   	  request: Trigger.Request,
										   	  response: Trigger.Response) -> Trigger.Response:

		response.message = "Unexpected error"
		response.success = False

		if not self.__nn_state and self.__calibration_view_mode:
			response.message = "It is not possible to enable the neural network while the calibration template preview is enabled"
			response.success = False

		elif not self.__nn_state:
			self.__nn_state = True
			response.message = "The neural network has been successfully enabled"
			response.success = True

		else:
			self.__nn_state = False
			response.message = "The neural network has been successfully disabled"
			response.success = True

		return response

def main(args=None):
	# model = YOLO("yolo11s-pose.pt")
	# model.export(format="engine")

	rclpy.init(args=args)

	camera_node = CameraStream()
	rclpy.spin(camera_node)
	camera_node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()