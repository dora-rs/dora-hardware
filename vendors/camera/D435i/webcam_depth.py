from typing import Callable
from dora import DoraStatus
import os
import cv2
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

        # 获取深度流
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image = ((depth_image - np.min(depth_image)) / (np.max(depth_image) - np.min(depth_image)) * 255).astype(np.uint8)

        # 发布深度数据流
        send_output(
            "image",
            pa.array(depth_image.ravel().view(np.uint8)),
            dora_input["metadata"],
        )

        return DoraStatus.CONTINUE



    def drop_operator(self):
        self.pipeline.stop()
