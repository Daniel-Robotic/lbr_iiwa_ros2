from typing import List
from pydantic import BaseModel, Field, IPvAnyAddress, ConfigDict

class RobotShemas(BaseModel):
	robot_ip: IPvAnyAddress = Field(default="172.31.1.147")
	robot_port: int = Field(qe=0, default=30200)
	recive_move_step: int = Field(qe=0, default=10)
	virtual_threshold: float = Field(qe=0, le=1, default=30200)

	model_config = ConfigDict(extra="forbid")
 
 
class CameraShemas(BaseModel):
	camera_names: List[str] = Field(max_length=32)
	width: int = Field(qe=0)
	height: int = Field(qe=0)
	fps: int = Field(qe=0, le=90)
	
 
	model_config = ConfigDict(extra="forbid")