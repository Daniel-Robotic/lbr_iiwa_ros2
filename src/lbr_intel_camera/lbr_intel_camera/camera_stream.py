import os
import cv2
import time
import rclpy
import torch
import logging
import rclpy.clock
import numpy as np

from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Header
from std_srvs.srv import Trigger
from sensor_msgs.msg import CompressedImage
from lbr_intel_camera_interface.msg import HumanDetection
from lbr_intel_camera_interface.srv import IntelCameraInformation, ChangeProfile

from lbr_intel_camera.utils import load_yaml
from lbr_intel_camera.utils.CalibartionUtils import find_calibration_template
from lbr_intel_camera.utils.DepthCameraWrapper import DepthCameraWrapper, DepthCameraInformation

from lbr_intel_camera.schemas import CameraSchemas


class CameraStream(Node):
    def __init__(self):

        super().__init__(f"{os.getenv('CAMERA_NAME')}_node")

        self.declare_parameter("config_path", value="./config.yaml")
        self.__config = load_yaml(self.get_parameter("config_path").value)

        camera_config = CameraSchemas.CameraSchema(**self.__config["camera_settings"])

        width = camera_config.width
        height = camera_config.height
        fps = camera_config.fps

        if not os.path.exists(f"./{camera_config.export_setting.nn_model_name}.engine"):
            
            try:
                self.get_logger().info(f"Loading model `{camera_config.export_setting.nn_model_name}`...")
                model = YOLO(f"{camera_config.export_setting.nn_model_name}.pt", verbose=False)
            
                self.get_logger().info(f"Convert model `{camera_config.export_setting.nn_model_name}` to TensoRT...")
                torch.cuda.empty_cache()
                model.export(format="engine",
                             device=camera_config.export_setting.device, 
                             half=camera_config.export_setting.convert_float16,
                             workspace=camera_config.export_setting.workspace,
                             verbose=False)
            
            except Exception as e:
                self.get_logger().error(f"Error loading model `{camera_config.export_setting.nn_model_name}`: {e}")
                exit(1)

        self.model = YOLO(f"./{camera_config.export_setting.nn_model_name}.engine", verbose=False)
        
        self.__camera_name = os.getenv('CAMERA_NAME')
        self.__flip_h = camera_config.flip_horizontally
        self.__flip_v = camera_config.flip_vertically
        self.__nn_state = camera_config.nn_state

        self.__calibration_view_mode = False
        self.__publish_image = False
        self.__publish_image_human_pose = True

        self.__zoom_level = camera_config.zoom_level

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
                                 depth=10)

        self.get_logger().info(f"Camera position `{self.__camera_name}` configuration...")

        # Настройка камеры
        self.__camera = DepthCameraWrapper(width=width,
                                           height=height,
                                           fps=fps)

        # Объявление сервисов и публикаторов
        self.get_logger().info("Inicializing services and publishing structure...")

        # Публикатроы для отправки данных об изображении
        # Отправку сообщения изображений можно отключить через специализированный сервис 
        self.__camera_stream_rgb_publisher = self.create_publisher(msg_type=CompressedImage,
                                                                    topic=f"camera/{self.__camera_name}/rgb/raw",
                                                                    qos_profile=1)
        self.__camera_stream_nn_publisher = self.create_publisher(msg_type=HumanDetection,
                                                                  topic=f"camera/{self.__camera_name}/human_pose",
                                                                  qos_profile=1)
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

        # Сервис включения нейронной сети
        self.__change_neural_network_mode_service = self.create_service(srv_type=Trigger,
                                                                        srv_name=f"camera/{self.__camera_name}/change_neural_network_mode",
                                                                        callback=self.__change_neural_network_mode_callback)
        
        self.__change_publish_image_service = self.create_service(srv_type=Trigger,
                                                                  srv_name=f"camera/{self.__camera_name}/change_publish_image",
                                                                  callback=self.__change_publish_image_callback)

        self.__change_publish_image_human_pose_service = self.create_service(srv_type=Trigger,
                                                                             srv_name=f"camera/{self.__camera_name}/change_publish_image_human_pose",
                                                                             callback=self.__change_publish_image_human_pose_callback)

        self.get_logger().info(f"Start camera {self.__camera_name} stream")

    def __kps_detection(self, frame: np.ndarray, depth_frame: np.ndarray) -> HumanDetection:
        start = time.time()
        results = self.model(frame, verbose=False)
        self.get_logger().debug(f"Inference time: {time.time() - start:.3f}s")
    
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

                # Заполняем keypoints и расстояния до них
                msg.keypoints = []
                msg.kps_distance = []  # Инициализируем список для расстояний
                
                for idx, kp in enumerate(keypoints.xy[0]):  # Используем enumerate для получения индекса
                    x = int(kp[0].item())  # Координата x ключевой точки
                    y = int(kp[1].item())  # Координата y ключевой точки

                    # Добавляем координаты ключевой точки
                    msg.keypoints.extend([float(x), float(y)])

                    # Получаем расстояние до ключевой точки из depth_frame
                    if 0 <= y < depth_frame.shape[0] and 0 <= x < depth_frame.shape[1]:
                        distance = depth_frame[y, x]  # Расстояние до точки
                        msg.kps_distance.append(float(distance))
                    else:
                        # Если координаты выходят за пределы кадра, добавляем NaN или 0
                        msg.kps_distance.append(float('nan'))  # или 0.0

                return msg
            else:
                return HumanDetection()

    def __camera_stream_callback(self):
        
        depth_image, color_image = self.__camera.get_aligned_images()

        if self.__flip_h:
            color_image = cv2.flip(color_image, 0)
            depth_image = cv2.flip(depth_image, 0)

        if self.__flip_v:
            color_image = cv2.flip(color_image, 1)
            depth_image = cv2.flip(depth_image, 1)

        if self.__zoom_level != 1.0:
            height, width = color_image.shape[:2]

            # Вычисляем новые размеры области обрезки
            new_width = int(width / self.__zoom_level)
            new_height = int(height / self.__zoom_level)

            # Вычисляем координаты области обрезки (центрируем)
            x1 = int((width - new_width) / 2)
            y1 = int((height - new_height) / 2)
            x2 = x1 + new_width
            y2 = y1 + new_height

            # Обрезаем изображение
            color_image = color_image[y1:y2, x1:x2]
            depth_image = depth_image[y1:y2, x1:x2]

            # Масштабируем обрезанное изображение до исходного размера
            color_image = cv2.resize(color_image, (width, height), interpolation=cv2.INTER_LINEAR)
            depth_image = cv2.resize(depth_image, (width, height), interpolation=cv2.INTER_LINEAR)
  
        if self.__nn_state and not self.__calibration_view_mode:
            msg = self.__kps_detection(color_image, depth_image)
            msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.__camera_name)

            if self.__publish_image_human_pose:
                msg.format = "jpg"
                msg.data.frombytes(np.array(cv2.imencode(f".jpg", color_image)[1]).tobytes())
                
            self.__camera_stream_nn_publisher.publish(msg)

        if self.__calibration_view_mode and not self.__nn_state:
            _, color_image = find_calibration_template(frame=color_image, size=self.__config["calibration_settings"]["pattern_size"])

        if self.__publish_image:
            self.__camera_stream_rgb_publisher.publish(CvBridge().cv2_to_compressed_imgmsg(color_image))

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
    
    # Смена профиля камеры
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

    # Сервис по переключению публикации стандартного сообщения публикации изображения
    def __change_publish_image_callback(self, request, response):
        self.__publish_image = not self.__publish_image
        response.success = True
        response.message = f"The image has been successfully changed on {self.__publish_image}"

        return response
    
    # Сервис по переключению публикации изображения в сообщении HumanPose
    def __change_publish_image_human_pose_callback(self, request, response):
        self.__publish_image_human_pose = not self.__publish_image_human_pose
        response.success = True
        response.message = f"The image has been successfully changed on {self.__publish_image_human_pose}"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraStream()
    
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()