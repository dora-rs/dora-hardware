from typing import Callable
from dora import DoraStatus
from DoraNmeaDriver_utils import DoraTwistStamped,DoraNavSatFix,DoraQuaternionStamped
import pickle

"""
输入：第一个字节表示消息类型： 1 表示NavSatFix消息类型
                          2 表示QuaternionStamped消息类型
                          3 表示TwistStamped消息类型
"""

class Operator:
    """
    反序列化后，输出NavSatFix、QuaternionStamped、TwistStamped消息内容
    """

    def __init__(self):
        self.dora_original_input=""
        self.receDoraQuaternionStamped = DoraQuaternionStamped()
        self.receDoraNavSatFix = DoraNavSatFix()
        self.receDoraTwistStamped = DoraTwistStamped()


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
        if "parsed_nmea_sentence" == dora_input["id"]:
            dora_original_input = dora_input["value"]
            dora_original_input_bytes = bytes(dora_original_input.to_pylist())

            #第1个字节作为消息类型的判定
            type_bytes = dora_original_input_bytes[:1]
            # 将整数部分转换回整数
            Sentence_type = int.from_bytes(type_bytes, byteorder='big')

            # 取从第2个字节开始的所有字节作为数据部分
            nmea_sentence_bytes = dora_original_input_bytes[1:]

            # NavSatFix消息类型
            if Sentence_type == 1 :
                self.receDoraNavSatFix = pickle.loads(nmea_sentence_bytes)
                print(self.receDoraNavSatFix)

            #QuaternionStamped消息类型
            if Sentence_type == 2:
                self.receDoraQuaternionStamped = pickle.loads(nmea_sentence_bytes)
                print(self.receDoraQuaternionStamped)
            #TwistStamped消息类型
            if Sentence_type == 3:
                self.receDoraTwistStamped = pickle.loads(nmea_sentence_bytes)
                print(self.receDoraTwistStamped)

            dora_original_input = "ddd".encode()
            send_output(
                "hh",
                dora_original_input,
                dora_input["metadata"],
            )
            return DoraStatus.CONTINUE

        else:
            return DoraStatus.CONTINUE