from typing import Callable
from dora import DoraStatus
import numpy as np
import pyarrow as pa
import pyrealsense2 as rs


pa.array([])

OUTPUT_WIDTH = 640
OUTPUT_HEIGHT = 480
# DEVICE_INDEX = os.environ.get('DEVICE_INDEX', '0')


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 15)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        self.config.enable_stream(rs.stream.infrared, 1)
        self.config.enable_stream(rs.stream.infrared, 2)
        self.pipeline.start(self.config)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

    def on_event(
        self,
        dora_event: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)
        return DoraStatus.CONTINUE

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):
        # 获取摄像头数据流
        frames = self.pipeline.wait_for_frames()
        infrared_frame = frames.get_infrared_frame(1)  # 获取左侧红外数据流

        # 将红外帧数据转换为numpy数组
        infra1_images = np.asanyarray(infrared_frame.get_data())

        infra1_images = ((infra1_images - np.min(infra1_images)) / (
                    np.max(infra1_images) - np.min(infra1_images)) * 255).astype(
                np.uint8)

        # 发布左红外数据流
        send_output(
                "image",
                pa.array(infra1_images.ravel().view(np.uint8)),
                dora_input["metadata"],
            )










        return DoraStatus.CONTINUE

    def drop_operator(self):
        self.pipeline.stop()
