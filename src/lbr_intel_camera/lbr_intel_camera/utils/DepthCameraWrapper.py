import json
import numpy as np
import pyrealsense2 as rs
from typing import List
from pydantic import BaseModel, ConfigDict


class DepthCameraInformation(BaseModel):
	device_name: str
	product_line: str
	serial_number: str
	firmware_version: str
	usb_type: str
	depth_profile: List[str]
	color_profile: List[str]

	model_config = ConfigDict(extra="forbid")

 
class DepthCameraProfiles(BaseModel):
	stream_type: str
	width: int
	height: int
	stream_format: str
	fps: int

	model_config = ConfigDict(extra="forbid")


class DepthCameraWrapper:

	"""
	Класс DepthCameraWrapper представляет обертку для работы с камерой глубины.

	Методы:
		- __init__(self, width: int = 640, height: int = 480, fps: int = 30): Инициализирует объект DepthCameraWrapper.
		- change_camera_profile(self, width, height, fps): Изменяет профиль камеры.
		- start_streaming_pipeline(self): Запускает поток камеры.
		- stop_streaming_pipeline(self): Останавливает поток камеры.
		- get_aligned_images(self): Возвращает выровненные изображения глубины и цвета.
		- get_pointcloud(self, depth_frame: rs.stream.depth, color_frame: rs.stream.depth): Возвращает облако точек.
		- get_camera_information(self): Возвращает информацию о камере.
		- __check_valid_profiles_specified_parameters(self, profiles: list, width: int, height: int, fps: int): Проверяет, являются ли указанные параметры допустимыми.
		- __get_profiles(self): Получает профили камеры.
		- __camera_sensor_profiles(self, sensor_id: int): Получает профили сенсора камеры.
   """

	def __init__(self, width:int=640, height:int=480, fps:int=30):

		self._pipeline = rs.pipeline()
		self._config = rs.config()
		self._rs_pointcloud = rs.pointcloud()
		
		pipeline_wrapper = rs.pipeline_wrapper(self._pipeline)
		pipeline_profile = self._config.resolve(pipeline_wrapper)
		self._device = pipeline_profile.get_device()

		align_to = rs.stream.color
		self._align = rs.align(align_to)
		
		found_rgb = False
		for s in self._device.sensors:
			if s.get_info(rs.camera_info.name) == 'RGB Camera':
				found_rgb = True
				break
			
		if not found_rgb:
			raise Exception(f"{str(self.__class__)} requires Depth camera with Color sensor")

		self._config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
		self._config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
		
		self._pipeline_started = False
	
	def __del__(self):
		self.stop_streaming_pipeline()
		self._device.hardware_reset()
		print("The camera has been rebooted")
		
	
	def change_camera_profile(self, width, height, fps):

		depth_stream_profiles, color_stream_profiles = self.__get_profiles()
		
		depth_status = self.__check_valid_profiles_specified_parameters(profiles=depth_stream_profiles,
																  		width=width, height=height, fps=fps)
		color_status = self.__check_valid_profiles_specified_parameters(profiles=color_stream_profiles,
																  		width=width, height=height, fps=fps)
		
		if depth_status and color_status:
			self.stop_streaming_pipeline()
   
			self._config = rs.config()
   
			self._config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
			self._config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
	
			self.start_streaming_pipeline()		
  
			return True

		return False

	
	def start_streaming_pipeline(self):
		if self._pipeline_started:
			return "The camera has already been started"

		self._pipeline.start(self._config)
		self._pipeline_started = True
		
		return "The camera turned on"


	def stop_streaming_pipeline(self):
		if not self._pipeline_started:
			return "The camera was already turned off"

		self._pipeline.stop()
		self._pipeline_started = False
		
		return "The camera is stopped"
	
	def get_aligned_images(self) -> List:
		if not self._pipeline_started:
			self.start_streaming_pipeline()
			
		frames = self._pipeline.wait_for_frames()
		align_frames = self._align.process(frames)
		
		depth_frame = align_frames.get_depth_frame()
		color_frame = align_frames.get_color_frame()
		
		if not depth_frame or not color_frame:
			raise('Invalid camera config')
		 
		depth_image = np.asanyarray(depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())
		
		return depth_image, color_image
	
	def get_pointcloud(self, depth_frame: rs.stream.depth,
							 color_frame: rs.stream.depth) -> List:
		
		points = self._rs_pointcloud.calculate(depth_frame)
		self._rs_pointcloud.map_to(color_frame)
		
		v, t = points.get_vertices(), points.get_texture_coordinates()
		verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
		texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv
		
		return verts, texcoords
	
	def get_camera_information(self):
		
		depth_stream_profiles, color_stream_profiles = self.__get_profiles()

		return DepthCameraInformation(
			device_name=str(self._device.get_info(rs.camera_info.name)),
			product_line=str(self._device.get_info(rs.camera_info.product_line)),
			serial_number=str(self._device.get_info(rs.camera_info.serial_number)),
			firmware_version=str(self._device.get_info(rs.camera_info.firmware_version)),
			usb_type=str(self._device.get_info(rs.camera_info.usb_type_descriptor)),
			depth_profile=depth_stream_profiles,
			color_profile=color_stream_profiles
		)
		
	def __check_valid_profiles_specified_parameters(self, profiles: list, width: int, height: int, fps: int) -> bool:
		status = False
	
		for profile in profiles:
			profile = json.loads(profile)

			if ((profile["width"] == width) and
				(profile["height"] == height) and
				(profile["fps"] == fps)):

				status = True
	
		return status

	 
  
	def __get_profiles(self):
		
		depth_modules = ["L500 Depth Sensor", "Stereo Module"]
		color_modules = ["RGB Camera"]
		
		depth_stream_profiles = []
		color_stream_profiles = []
		
		for i, s in enumerate(self._device.sensors):
			if s.get_info(rs.camera_info.name) in color_modules:
				color_stream_profiles = self.__camera_sensor_profiles(i)
			elif s.get_info(rs.camera_info.name) in depth_modules:
				depth_stream_profiles = self.__camera_sensor_profiles(i)
				
		return depth_stream_profiles, color_stream_profiles

	
	def __camera_sensor_profiles(self, sensor_id: int) -> List:
		stream_profiles = []
		profiles = self._device.sensors[sensor_id].get_stream_profiles()
		for profile in profiles:
			video_profile = rs.video_stream_profile(profile)
			if (video_profile.stream_type() == rs.stream.depth or 
				video_profile.stream_type() == rs.stream.color):
				
				video_format = video_profile.format() 
				
				if video_format == rs.format.bgr8 or video_format == rs.format.z16:
				
					type_p = video_profile.stream_type()
					width, height = video_profile.width(), video_profile.height()
					fps = video_profile.fps()
					format_p = video_format

					stream_profiles.append(DepthCameraProfiles(
						stream_type=f"{type_p}",
	  					width=width, 
		   				height=height, 
			   			stream_format=f"{format_p}", 
				  		fps=fps
					).model_dump_json())

		return stream_profiles
	
		
	
if __name__ == "__main__":
	camera = DepthCameraWrapper()
	print(camera.change_camera_profile(640, 480, 30))
	print(camera.change_camera_profile(640, 480, 100))
	
