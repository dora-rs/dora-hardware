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
        frames = self.pipeline.wait_for_frames()

        # 获得imu数据
        imu_frame = frames.first_or_default(rs.stream.accel)
        imu_data = imu_frame.as_motion_frame().get_motion_data()

        # 发布imu数据流
        send_output(
            "imu_data",
            pa.array(imu_data.ravel().view(np.uint8)),
            dora_input["metadata"],
        )


        return DoraStatus.CONTINUE
    print(len(pa.array(on_input.depth_images.ravel().view(np.uint8))))

    def drop_operator(self):
        self.pipeline.stop()
