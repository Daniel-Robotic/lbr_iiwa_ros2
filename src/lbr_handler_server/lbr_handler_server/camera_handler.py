import os
import cv2
import json
import rclpy
import threading
import numpy as np

from queue import Queue
from threading import Lock
from rclpy.node import Node
from std_srvs.srv import Trigger
from lbr_intel_camera_interface.msg import HumanDetection
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Кастомные импорты
from lbr_intel_camera.utils import load_yaml

class CameraHandler(Node):
    def __init__(self):
        super().__init__(f"{self.__class__.__name__}_node")
        
        # Инициализация конфигурации
        self.declare_parameter("config_path", value="./config.yaml")
        self.__config = load_yaml(self.get_parameter("config_path").value)
        
        self.__camera_list = self.__config["general_settings"]["camera_list"]
        self.__dataset_path = self.__config["general_settings"]["dataset_path"]
        self.__save_frames = False
        self.__save_frames_lock = Lock()
        self.__sync_thread = None
        # Инициализация блокировок для JSON файлов
        self.__data_queues = {camera: Queue() for camera in self.__camera_list}
        self.__image_queues = {camera: Queue() for camera in self.__camera_list}
        self.__common_data = {camera: [] for camera in self.__camera_list}
        self.__json_lock = Lock()  # Общая блокировка для доступа к common_data
        
        # Создание директорий при инициализации
        for camera in self.__camera_list:
            camera_dir = os.path.join(self.__dataset_path, camera)
            self._create_directory(camera_dir)
        
        # Настройка ROS2 подписок и сервисов
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.create_service(
            Trigger,
            f"{self.__class__.__name__}/save_frames",
            self.__save_frames_callback
        )
        
        for camera_name in self.__camera_list:
            self.create_subscription(
                HumanDetection,
                f"camera/{camera_name}/human_pose",
                lambda msg, cam=camera_name: self.__camera_callback(msg, cam),
                qos_profile
            )
        self.start_sync_thread()
        self.get_logger().info("Camera handler initialized")

    def start_sync_thread(self):
        if self.__sync_thread is None or not self.__sync_thread.is_alive():
            self.__sync_thread = threading.Thread(target=self.__sync_and_save_data, daemon=True)
            self.__sync_thread.start()

    def _create_directory(self, path: str) -> None:
        if not os.path.exists(path):
            try:
                os.makedirs(path)
                self.get_logger().info(f"Created directory: {path}")
            except OSError as e:
                self.get_logger().error(f"Failed to create directory {path}: {e}")

    def __camera_callback(self, msg: HumanDetection, camera_name: str):
        # Проверка флага записи с блокировкой
        with self.__save_frames_lock:
            if not self.__save_frames:
                return
        try:
            timestamp = f"{msg.header.stamp.sec}_{msg.header.stamp.nanosec:09d}"
            frame_path = os.path.join(self.__dataset_path, camera_name, f"{timestamp}.jpg")
            
            frame_data = {
                "frame_path": frame_path,
                "bbox": {
                    "x": msg.x,
                    "y": msg.y,
                    "width": msg.width,
                    "height": msg.height
                },
                "confidence": msg.confidence,
                "kps": self.__convert_kps(msg.keypoints),
                "kps_distance": [self.__safe_convert_to_int(x) for x in msg.kps_distance]
            }
            
            img_data = {
                "frame_path": frame_path,
                "data": msg.data
            }
            
            self.__data_queues[camera_name].put(frame_data)
            self.__image_queues[camera_name].put(img_data)
        except Exception as e:
            self.get_logger().error(f"[{camera_name}] Image processing error: {e}")
            return

    def __sync_and_save_data(self):
        while True:
            data = {}
            images = {}
            all_data_ready = True
            
            for camera_name in self.__camera_list:
                if not self.__data_queues[camera_name].empty() and not self.__image_queues[camera_name].empty():
                    data[camera_name] = self.__data_queues[camera_name].get()
                    images[camera_name] = self.__image_queues[camera_name].get()
                else:
                    all_data_ready = False
                    break
            
            if all_data_ready:
                with self.__json_lock:
                    for camera_name, frame_data in data.items():
                        self.__common_data[camera_name].append(frame_data)
                        
                        image_data = images[camera_name]
                        frame = self.__img_to_cv2(image_data['data'])
                        cv2.imwrite(image_data['frame_path'], frame)
                    
                    common_json_path = os.path.join(self.__dataset_path, "data.json")
                    try:
                        with open(common_json_path, 'w') as f:
                            json.dump(self.__common_data, f, indent=4)
                    except Exception as e:
                        self.get_logger().error(f"Common JSON error: {e}")

    def __img_to_cv2(self, msg_data: bytes) -> np.ndarray:
        return cv2.imdecode(np.frombuffer(msg_data, np.uint8), cv2.IMREAD_COLOR)

    def __convert_kps(self, kps: np.ndarray) -> list:
        if len(kps) % 2 != 0:
            self.get_logger().error("Invalid keypoints format")
            return []
        return [[int(kps[i]), int(kps[i+1])] for i in range(0, len(kps), 2)]

    def __safe_convert_to_int(self, value: float) -> int:
        try:
            return int(value)
        except (ValueError, TypeError):
            return 0

    def __save_frames_callback(self, request, response):
        with self.__save_frames_lock:
            self.__save_frames = not self.__save_frames
            status = "ENABLED" if self.__save_frames else "DISABLED"
        
        response.success = True
        response.message = f"Frame saving {status}"
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    handler = CameraHandler()
    try:
        rclpy.spin(handler)
    except KeyboardInterrupt:
        handler.get_logger().info("Shutting down...")
    finally:
        handler.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()