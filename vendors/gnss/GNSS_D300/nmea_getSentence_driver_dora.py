from typing import Callable
from dora import DoraStatus
import serial


serial_port = '/dev/ttyUSB1'
serial_baud = 115200


class Operator:
    """
    打开串口读数据，校验正确后，send_out下个nmea_publishSentence_driver_dora节点解析消息并发布
    """
    # 打开串口读取数据
    def __init__(self):
        try :
            self.GPS = serial.Serial(port = serial_port, baudrate = serial_baud, timeout = 1)
        except serial.SerialException as ex:
            print("Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))
        self.data = ""


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
        # 读取一行数据并去除首尾空白字符
        self.data = self.GPS.readline().strip()
        # 如果数据非空
        if self.data:
            try:
                if isinstance(self.data, bytes):
                    self.data = self.data.decode("utf-8")
                nmea_sentence = self.data
                # 校验NMEA句子
                split_sentence = nmea_sentence.split('*')
                if len(split_sentence) != 2:
                    # No checksum bytes were found... improperly formatted/incomplete NMEA data?
                    print(
                        "Received a sentence with an invalid checksum. " + "Sentence was: %s" % nmea_sentence)
                    return DoraStatus.CONTINUE
                transmitted_checksum = split_sentence[1].strip()

                # Remove the $ at the front
                data_to_checksum = split_sentence[0][1:]
                checksum = 0
                for c in data_to_checksum:
                    checksum ^= ord(c)

                if not ("%02X" % checksum) == transmitted_checksum.upper():
                    print(
                        "Received a sentence with an invalid checksum. " + "Sentence was: %s" % nmea_sentence)
                # 有效的NMEA数据则发送
                else:
                    nmea_sentence = nmea_sentence.encode()
                    #测试 print(nmea_sentence)
                    send_output(
                        "nmea_sentence",
                        nmea_sentence,
                        dora_input["metadata"],
                    )
                return DoraStatus.CONTINUE

            except ValueError as e:
                print(
                    "Value error, likely due to missing fields in the NMEA message. Error was: %s. " % e)
            return DoraStatus.CONTINUE



    def drop_operator(self):
         self. GPS.close()
