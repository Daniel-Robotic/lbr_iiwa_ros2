from typing import List
from pydantic import BaseModel, ConfigDict

class SaveDataSchema(BaseModel):
	pattern_frames_path: str
	config_name: str
	overwrite_pattern_frames: bool
	save_calibration_info: bool

	model_config = ConfigDict(extra="forbid")


class GeneralSchema(BaseModel):
	robot_name: str
	camera_list: List[str]