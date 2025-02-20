import os
import cv2
import json
import rclpy
import logging
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from lbr_intel_camera.utils import load_yaml
from lbr_intel_camera.schemas import CameraSchemas

from std_srvs.srv import Trigger
from lbr_intel_camera_interface.msg import HumanDetection

class CameraHandler(Node):
    def __init__(self):
        super().__init__(f"{self.__class__.__name__}_node")
        
        self.declare_parameter("config_path", value="./config.yaml")
        self.__config = load_yaml(self.get_parameter("config_path").value)
        
        camera_config = CameraSchemas.CameraSchema(**self.__config["camera_settings"])

        self.__camera_list = self.__config["general_settings"]["camera_list"]
        self.__dataset_path = self.__config["general_settings"]["dataset_path"]
        self.__camera_width = camera_config.width
        self.__camera_height = camera_config.height
        self.__save_frames = True
        
        # Настройка логирования
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(f"{self.__class__.__name__}.log"),
                logging.StreamHandler()
            ]
        )
        self._logger = logging.getLogger(self.get_name())
        
        # Настройки ROS
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE,
                                 depth=1)
        
        
        self.__save_frames_service = self.create_service(Trigger,
                                                         f"{self.__class__.__name__}/save_frames", 
                                                         self.__save_frames_callback)
        
        for camera_name in self.__camera_list:
            self.create_subscription(msg_type=HumanDetection,
                                    topic=f"camera/{camera_name}/human_pose",
                                    callback=self.__nn_subscriber_calback,
                                    qos_profile=qos_profile)
        
        self.get_logger().info("Start camera handler...")
        
    
    def _create_directory(self, 
                          path: str = "./",
                          directory_name: str = "datasets") -> None:
        
        # Проверяем существование директории
        if not os.path.exists(os.path.join(path, directory_name)):
            # Если директория не существует, создаем её
            os.makedirs(os.path.join(path, directory_name))
            self.get_logger().info(f"Директория {os.path.join(path, directory_name)} успешно создана.")

    
    def _img_to_cv2(self, 
                    msg_data: np.ndarray) -> np.ndarray:
        
        # Преобразуем массив байтов в NumPy массив
        nparr = np.frombuffer(msg_data, np.uint8)
        # Декодируем изображение с использованием OpenCV
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        
        return image
    
    
    def _read_json(self, json_path: str) -> dict:
        try:
            with open(json_path, "r") as f:
                data = json.load(f)

        except:
            with open(json_path, "w") as f:
                data = {}
                json.dump(data, f)
            
        return data

    def _msg2json(self, json_path: str, data: dict) -> None:
            
        # Записываем обновленные данные обратно в файл
        with open(json_path, "w", encoding="utf-8") as file:
            json.dump(data, file, ensure_ascii=False, indent=4)
    
    
    def _convert_kps(self, kps: np.ndarray) -> list:
        kps = kps.tolist()
        
        if len(kps) % 2 != 0:
            self.get_logger().error("Длина массива keypoints должна быть четной.")
            raise ValueError("Длина массива keypoints должна быть четной.")
    
        kps = [int(kps[i:i+2]) for i in range(0, len(kps), 2)]
        return kps

    
    
    def __nn_subscriber_calback(self, msg: HumanDetection):
        
        if not self.__save_frames:
            return
            
        self._create_directory(path=self.__dataset_path,
                                directory_name="")
        
        self._create_directory(path=self.__dataset_path,
                                directory_name=msg.header.frame_id)

        frame = self._img_to_cv2(msg_data=msg.data)
        json_path = os.path.join(self.__dataset_path, "data.json")
        frame_path = os.path.join(self.__dataset_path, msg.header.frame_id, f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}.jpg")
        # Читаем существующие данные из JSON
        data = self._read_json(json_path=json_path)

        # Сохраняем изображение
        frame = self._img_to_cv2(msg_data=msg.data)
        cv2.imwrite(frame_path, frame)

        # Подготавливаем данные для записи
        bbox = {
            "x": msg.x,
            "y": msg.y,
            "width": msg.width,
            "height": msg.height,
        }

        frame_data = {
            "frame_path": frame_path,
            "bbox": bbox,
            "confidence": msg.confidence,
            "kps": self._convert_kps(msg.keypoints),
            "kps_distance": msg.kps_distance.tolist()
        }
        
        # Добавляем данные в словарь
        if msg.header.frame_id in data:
            data[msg.header.frame_id].append(frame_data)
        else:
            data[msg.header.frame_id] = [frame_data]

        # Записываем обновленные данные в JSON
        self._msg2json(json_path=json_path, data=data)
        


    def __save_frames_callback(self, 
                                request: Trigger.Request,
                                response: Trigger.Response):
        self.__save_frames = not self.__save_frames 
        response.success = True
        response.message = f"The data collection mode was {'on' if self.__save_frames else 'off'}"

        return response

        
        
        
def main(args=None):
    rclpy.init(args=args)
    camera_handler = CameraHandler()
    rclpy.spin(camera_handler)
    camera_handler.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()