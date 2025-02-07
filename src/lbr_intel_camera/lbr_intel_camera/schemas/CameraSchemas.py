from pydantic import BaseModel, Field, ConfigDict


class ExportNNModelSchema(BaseModel):
    nn_model_name: str
    convert_float16: bool
    convert_int8: bool
    dynamic: bool
    device: str


class CameraSchema(BaseModel):
    width: int = Field(qe=0)
    height: int = Field(qe=0)
    fps: int = Field(qe=0)
    flip_horizontally: bool
    flip_vertically: bool
    nn_state: bool
    export_setting: ExportNNModelSchema
    

    model_config = ConfigDict(extra="forbid")