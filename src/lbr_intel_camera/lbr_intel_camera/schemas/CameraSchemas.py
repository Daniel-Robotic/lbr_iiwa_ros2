from pydantic import BaseModel, Field, ConfigDict


class ExportNNModelSchema(BaseModel):
    nn_model_name: str
    convert_float16: bool
    device: str
    workspace: float


class CameraSchema(BaseModel):
    width: int = Field(qe=0)
    height: int = Field(qe=0)
    fps: int = Field(qe=0)
    flip_horizontally: bool
    flip_vertically: bool
    nn_state: bool
    export_setting: ExportNNModelSchema
    zoom_level: float = Field(qe=0)
    crop_x_offset: float = Field(qe=0)
    crop_y_offset: float = Field(qe=0)
    

    model_config = ConfigDict(extra="forbid")