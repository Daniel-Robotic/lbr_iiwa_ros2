# TODO: Переименовать файл в camera_server
# TODO: Добавить сервис по очистке папки

import os
import cv2
import yaml
import rclpy
import shutil
import logging
import rclpy.clock
import numpy as np

from rclpy.node import Node
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import CompressedImage

from lbr_intel_camera_interface.msg import CalibrationData
from lbr_intel_camera_interface.srv import MonoCalibration, StereoCalibration, ChoiceCamera

from lbr_intel_camera.utils import load_yaml
from lbr_intel_camera.utils.CalibartionUtils import mono_calibration, stereo_calibration
from lbr_intel_camera.schemas import CalibrationSchemas, GeneralSchemas



class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__(f"camera_server_node")

        # TODO: Изменить на пустое поле
        self.declare_parameter("config_path", value="/home/rnf/ros2_ws/config.yaml")
        self.__config = load_yaml(self.get_parameter("config_path").value)

        general_setting = GeneralSchemas.GeneralSchema(**self.__config["general_settings"])
        calib_config = CalibrationSchemas.CalibrationSchemas(**self.__config["calibration_settings"])

        self.__pattern_size = calib_config.pattern_size
        self.__convolution_size = calib_config.convolution_size
        self.__image_format_valid = calib_config.image_format_valid
        self.__pattern_frames_path = calib_config.save_config.pattern_frames_path
        self.__config_name = calib_config.save_config.config_name
        self.__overwrite_pattern_frames = calib_config.save_config.overwrite_pattern_frames
        self.__save_calibration_info = calib_config.save_config.save_calibration_info

        self.__camera_list = general_setting.camera_list

        # Настройка логирования
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler("camera_server.log"),
                logging.StreamHandler()
            ]
        )
        self._logger = logging.getLogger(self.get_name())

        # Создание папки для сохранения кадров
        if not self.__pattern_frames_path:
            self.__pattern_frames_path = os.path.join(os.getcwd(), f"camera_server_frames")
            self.get_logger().info(f"Pattern frames path: {self.__pattern_frames_path}")

        if self.__overwrite_pattern_frames and os.path.exists(self.__pattern_frames_path):
            shutil.rmtree(self.__pattern_frames_path)
            self.get_logger().warning(f"Directory: {self.__pattern_frames_path} - has been cleared")
        
        if not os.path.exists(self.__pattern_frames_path):
            os.makedirs(self.__pattern_frames_path)
            self.get_logger().info(f"Directory: {self.__pattern_frames_path} - has been created")

        # Настройки ROS
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE,
                                 depth=1)

        self.__subscriber = {}
        for camera_name in self.__camera_list:
            self.__subscriber[camera_name] = {
                "rgb": self.create_subscription(msg_type=CompressedImage,
                                                topic=f"camera/{camera_name}/rgb/raw",
                                                callback=lambda msg, camera_name=camera_name: self.__rgb_frame_callback(msg, camera_name),
                                                qos_profile=qos_profile),

                # "depth": self.create_subscription(msg_type=CompressedImage,
                #                                     topic=f"camera/{camera_name}/depth/raw",
                #                                   callback=lambda msg, camera_name=camera_name: self.__depth_frame_callback(msg, camera_name),
                #                                   qos_profile=qos_profile)
            }

        self.__frames = {camera_name: {} for camera_name in self.__camera_list}

        self.__get_camera_frame = self.create_service(srv_type=ChoiceCamera,
                                                      srv_name="calibration_server/get_camera_frame",
                                                      callback=self.__save_camera_frame_callback)

        self.__clear_camera_frame_folder = self.create_service(srv_type=ChoiceCamera,
                                                               srv_name="calibration_server/clear_frame_data",
                                                               callback=self.__clear_camera_frame_folder_callback)

        # TODO: Подумать над передаваемыми параметрами
        self.__mono_calibration_service = self.create_service(srv_type=MonoCalibration,
                                                              srv_name=f"calibration_server/calibration/mono_calibration",
                                                              callback=self.__mono_calibration_callback)
        
        self.__stereo_calibration_service = self.create_service(srv_type=StereoCalibration,
                                                                srv_name=f"calibration_server/calibration/stereo_calibration",
                                                                callback=self.__stereo_calibration_callback)

    def __check_camera_in_list(self, camera_name: str) -> tuple[bool, str]:
        status = True
        msg = ""
        
        if not camera_name in self.__camera_list:
            status = False
            msg = f"The camera {camera_name} does not exist"
        
        return status, msg
    
    def __check_exist_camera_calibration(self, camera_name):
        status, msg = self.__check_camera_in_list(camera_name)
        if not status:
            calibration_data = CalibrationData(status=status,
                                               msg=msg,
                                               camera_matrix=[0]*9,
                                               distortion_coefficients=[0]*5)
            raise IndexError(calibration_data.msg)
        
        return status, msg


    def __rgb_frame_callback(self, msg: CompressedImage, camera_name: str) -> None:
        color_image = CvBridge().compressed_imgmsg_to_cv2(msg, "bgr8")
        self.__frames[camera_name]["rgb"] = color_image

        
    def __depth_frame_callback(self, msg: CompressedImage, camera_name: str) -> None:
        depth_image = CvBridge().compressed_imgmsg_to_cv2(msg, "passthrough")
        self.__frames[camera_name]["depth"] = depth_image

    def __save_camera_frame_callback(self,
                                     request: ChoiceCamera.Request,
                                     response: ChoiceCamera.Response) -> ChoiceCamera.Response:

        try:
            response.status, response.msg = self.__check_camera_in_list(request.camera_name)
            if not response.status:
                raise IndexError(response.msg)

             # Создание папки для камеры, если она не существует
            camera_folder = os.path.join(self.__pattern_frames_path, request.camera_name)
            if not os.path.exists(camera_folder):
                os.makedirs(camera_folder)

            # Сохранение в файл:
            timestamp = rclpy.clock.Clock().now().to_msg().sec
            image_path = os.path.join(camera_folder, f"color_image_{timestamp}.png")
            cv2.imwrite(image_path, self.__frames[request.camera_name]["rgb"])

            response.status = True
            response.msg = f"Saved color image to {image_path}"

            self.get_logger().info(response.msg)

        except IndexError as e:
           self.get_logger().error(e)

        except Exception as e:
            response.status = False
            response.msg = f"Error: {e}"
            self.get_logger().error(response.msg)

        finally:
            return response

    def __clear_camera_frame_folder_callback(self,
                                             request: ChoiceCamera.Request,
                                             response: ChoiceCamera.Response) -> ChoiceCamera.Response:
        try:
            response.status, response.msg = self.__check_camera_in_list(request.camera_name)
            if not response.status:
                raise IndexError(response.msg)
            
            camera_folder = os.path.join(self.__pattern_frames_path, request.camera_name)

            # Проверка существования папки и её удаление
            if os.path.exists(camera_folder):
                shutil.rmtree(camera_folder)
                response.status = True
                response.msg = f"Deleted folder {camera_folder}"
                self.get_logger().info(response.msg)
            else:
                response.status = False
                response.msg = f"Folder {camera_folder} does not exist"
                self.get_logger().error(response.msg)

        except IndexError as e:
            self.get_logger().error(e)
        except Exception as e:
            response.status = False
            response.msg = f"Error: {e}"
            self.get_logger().error(response.msg)
        finally:
            return response
        


    def __mono_calibration_callback(self, 
                                    request: MonoCalibration.Request, 
                                    response: MonoCalibration.Response) -> MonoCalibration.Request:
        try:
            status, msg = self.__check_exist_camera_calibration(request.camera_name)
                
            camera_folder = os.path.join(self.__pattern_frames_path, request.camera_name)
            ret, mtx, dist, msg = mono_calibration(path=camera_folder,
                                                    images_format_valid=self.__image_format_valid,
                                                    pattern_size=self.__pattern_size,
                                                    conv_size=(self.__convolution_size, self.__convolution_size))
            
            if not ret:
                response.calibration_info = CalibrationData(status=False,
                                                            msg=msg,
                                                            camera_matrix=[0]*9,
                                                            distortion_coefficients=[0]*5)
                self.get_logger().error(msg)
                raise Exception(msg)
            
        
            response.calibration_info = CalibrationData(status=True,
                                                        msg=msg,
                                                        camera_matrix=mtx.flatten(),
                                                        distortion_coefficients=dist.flatten())
            self.get_logger().info(msg)

            if self.__save_calibration_info:
                calibration_data = {}

                if os.path.exists(f"{self.__config_name}.yaml"):
                    calibration_data = load_yaml(f"{self.__config_name}.yaml")

                calibration_data[request.camera_name] = {
                    "camera_matrix": mtx.tolist(),
                    "distortion_coefficients": dist.tolist()
                }

                with open(f"{self.__config_name}.yaml", "w", encoding="UTF-8") as file:
                    yaml.dump(calibration_data, file, default_flow_style=False)


        except IndexError as e:
            self.get_logger().error(e)
        except Exception as e:
            response.status = False
            response.msg = f"Error: {e}"
            self.get_logger().error(response.msg)
        finally:
            return response
        

    def __stereo_calibration_callback(self,
                                      request: StereoCalibration.Request, 
                                      response: StereoCalibration.Response) -> StereoCalibration.Response:
        
        try:
            _, msg_camera1 = self.__check_exist_camera_calibration(request.first_camera_name)
            _, msg_camera2 = self.__check_exist_camera_calibration(request.second_camera_name)

            camera1_folder = os.path.join(self.__pattern_frames_path, request.first_camera_name)
            camera2_folder = os.path.join(self.__pattern_frames_path, request.second_camera_name)

            calibration_data = load_yaml(f"{self.__config_name}.yaml")
            matrix1 = np.array(calibration_data[request.first_camera_name]["camera_matrix"])
            matrix2 = np.array(calibration_data[request.second_camera_name]["camera_matrix"])
            distortion1 = np.array(calibration_data[request.first_camera_name]["distortion_coefficients"])
            distortion2 = np.array(calibration_data[request.second_camera_name]["distortion_coefficients"])

            ret, msg, mtx1, dist1, mtx2, dist2, R, T = stereo_calibration(path_image_folde1=camera1_folder,
                                                                          path_image_folde2=camera2_folder,
                                                                          cam_matrix1=matrix1,
                                                                          cam_matrix2=matrix2,
                                                                          cam_dist1=distortion1,
                                                                          cam_dist2=distortion2,
                                                                          images_format_valid=self.__image_format_valid,
                                                                          pattern_size=self.__pattern_size,
                                                                          conv_size=(self.__convolution_size, self.__convolution_size))
            
            if not ret:
                response.first_camera_calibration_info = CalibrationData(status=False,
                                                                         msg=msg,
                                                                         camera_matrix=[0]*9,
                                                                         distortion_coefficients=[0]*5)
                
                response.second_camera_calibration_info = CalibrationData(status=False,
                                                                          msg=msg,
                                                                          camera_matrix=[0]*9,
                                                                          distortion_coefficients=[0]*5)
                response.rotation_matrix = [0]*9
                response.translation_vector = [0]*3

                self.get_logger().error(msg)
                raise Exception(msg)
        
            response.first_camera_calibration_info = CalibrationData(status=True,
                                                                     msg=msg,
                                                                     camera_matrix=mtx1.flatten(),
                                                                     distortion_coefficients=dist1.flatten())
            response.second_camera_calibration_info = CalibrationData(status=True,
                                                                     msg=msg,
                                                                     camera_matrix=mtx2.flatten(),
                                                                     distortion_coefficients=dist2.flatten())

            response.rotation_matrix = R.flatten()
            response.translation_vector = T.flatten()

            self.get_logger().info(msg)

            if self.__save_calibration_info:
                calibration_data = {}

                if os.path.exists(f"{self.__config_name}.yaml"):
                    calibration_data = load_yaml(f"{self.__config_name}.yaml")

                calibration_data[f"{request.first_camera_name}_{request.second_camera_name}"] = {
                    f"camera_matrix_{request.first_camera_name}": mtx1.tolist(),
                    f"distortion_coefficients_{request.first_camera_name}": dist1.tolist(),
                    f"camera_matrix_{request.second_camera_name}": mtx2.tolist(),
                    f"distortion_coefficients_{request.second_camera_name}": dist2.tolist(),
                    "rotation_matrix": R.tolist(),
                    "translation_vector": T.tolist()
                }

                with open(f"{self.__config_name}.yaml", "w", encoding="UTF-8") as file:
                    yaml.dump(calibration_data, file, default_flow_style=False)

        except IndexError as e:
            self.get_logger().error(e)
        except Exception as e:
            response.status = False
            response.msg = f"Error: {e}"
            self.get_logger().error(response.msg)
        finally:
            return response



def main(args=None):
    rclpy.init(args=args)

    calibration_node = CameraCalibrationNode()
    rclpy.spin(calibration_node)
    calibration_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
