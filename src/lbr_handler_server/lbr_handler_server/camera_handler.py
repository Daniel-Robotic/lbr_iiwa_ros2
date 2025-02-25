import os
import cv2
import json
import math
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
        
        self.__camera_list = self.__config["general_settings"]["camera_list"]
        self.__dataset_path = self.__config["general_settings"]["dataset_path"]
        self.__save_frames = False
        
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
                                 depth=10)
        
        self.create_service(Trigger,
                            f"{self.__class__.__name__}/save_frames", 
                            self.__save_frames_callback)
        
        for camera_name in self.__camera_list:
            self.create_subscription(
                msg_type=HumanDetection,
                topic=f"camera/{camera_name}/human_pose",
                callback=lambda msg, cam_name=camera_name: self.camera_callback(msg, cam_name),
                qos_profile=qos_profile
            )
        
        self.get_logger().info("Start camera handler...")
    
    def _create_directory(self, path: str, directory_name: str) -> None:
        full_path = os.path.join(path, directory_name)
        if not os.path.exists(full_path):
            os.makedirs(full_path)
            self.get_logger().info(f"Directory {full_path} created.")
    
    def _img_to_cv2(self, msg_data: np.ndarray) -> np.ndarray:
        nparr = np.frombuffer(msg_data, np.uint8)
        return cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    
    def _read_json(self, json_path: str) -> dict:
        try:
            with open(json_path, "r") as f:
                return json.load(f)
        except FileNotFoundError:
            return []
    
    def _write_json(self, json_path: str, data: list) -> None:
        with open(json_path, "w", encoding="utf-8") as file:
            json.dump(data, file, ensure_ascii=False, indent=4)
    
    def _convert_kps(self, kps: np.ndarray) -> list:
        kps = kps.tolist()
        if len(kps) % 2 != 0:
            self.get_logger().error("Keypoints array length must be even.")
            raise ValueError("Keypoints array length must be even.")
        return [list(map(self._safe_convert_to_int, kps[i:i+2])) for i in range(0, len(kps), 2)]
    
    def _safe_convert_to_int(self, value):
        return 0 if math.isnan(value) else int(value)
    
    def camera_callback(self, msg: HumanDetection, camera_name: str):
        if not self.__save_frames:
            return
        
        # Создаем директории, если они еще не существуют
        camera_dir = os.path.join(self.__dataset_path, camera_name)
        self._create_directory(path=self.__dataset_path, directory_name=camera_name)
        
        frame = self._img_to_cv2(msg_data=msg.data)
        timestamp = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec}"
        frame_path = os.path.join(camera_dir, f"{timestamp}.jpg")
        
        # Сохраняем изображение
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
            "kps_distance": list(map(self._safe_convert_to_int, msg.kps_distance.tolist()))
        }
        
        # Путь к JSON-файлу для данной камеры
        json_path = os.path.join(self.__dataset_path, f"{camera_name}.json")
        
        # Чтение существующих данных
        existing_data = self._read_json(json_path=json_path)
        
        # Добавляем новые данные
        existing_data.append(frame_data)
        
        # Записываем обновленные данные в JSON
        self._write_json(json_path=json_path, data=existing_data)
    
    def __save_frames_callback(self, request: Trigger.Request, response: Trigger.Response):
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