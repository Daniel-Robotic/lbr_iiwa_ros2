from typing import List
from pydantic import BaseModel, ConfigDict
from .GeneralSchemas import SaveDataSchema

class CalibrationSchemas(BaseModel):
	pattern_size: List[int]
	convolution_size: int
	image_format_valid: List[str]
	
	save_config: SaveDataSchema

	model_config = ConfigDict(extra="forbid")
