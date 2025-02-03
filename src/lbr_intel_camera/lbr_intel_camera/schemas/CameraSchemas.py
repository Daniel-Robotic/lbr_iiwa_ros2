from pydantic import BaseModel, Field, ConfigDict


class CameraSchema(BaseModel):
	width: int = Field(qe=0)
	height: int = Field(qe=0)
	fps: int = Field(qe=0)
	flip_horizontally: bool
	flip_vertically: bool
	nn_state: bool
	nn_model_name: str
	convert_float16: bool
	convert_int8: bool
	dynamic: bool

	model_config = ConfigDict(extra="forbid")