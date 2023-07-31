import time
import math
from transforms3d._gohlketransforms import quaternion_from_euler


class Timestamp:
    def __init__(self, sec=None, nanosec=None):
        self.sec = sec
        self.nanosec = nanosec
    @staticmethod
    def now():
        current_time = time.time()
        sec = int(current_time)
        nanosec = int((current_time - sec) * 1e9)
        return Timestamp(sec, nanosec)

class Header:
    def __init__(self):
        self.stamp = Timestamp()
        self.frame_id = ''


class NavSatStatus:
    __constants = {
        'STATUS_NO_FIX': -1,
        'STATUS_FIX': 0,
        'STATUS_SBAS_FIX': 1,
        'STATUS_GBAS_FIX': 2,
        'SERVICE_GPS': 1,
        'SERVICE_GLONASS': 2,
        'SERVICE_COMPASS': 4,
        'SERVICE_GALILEO': 8,
        
    }
    def STATUS_NO_FIX(self):
        """Message constant 'STATUS_NO_FIX'."""
        return self.__constants['STATUS_NO_FIX']

    def STATUS_FIX(self):
        """Message constant 'STATUS_FIX'."""
        return self.__constants['STATUS_FIX']

    def STATUS_SBAS_FIX(self):
        """Message constant 'STATUS_SBAS_FIX'."""
        return self.__constants['STATUS_SBAS_FIX']

    def STATUS_GBAS_FIX(self):
        """Message constant 'STATUS_GBAS_FIX'."""
        return self.__constants['STATUS_GBAS_FIX']

    def SERVICE_GPS(self):
        """Message constant 'SERVICE_GPS'."""
        return self.__constants['SERVICE_GPS']

    def SERVICE_GLONASS(self):
        """Message constant 'SERVICE_GLONASS'."""
        return self.__constants['SERVICE_GLONASS']

    def SERVICE_COMPASS(self):
        """Message constant 'SERVICE_COMPASS'."""
        return self.__constants['SERVICE_COMPASS']

    def SERVICE_GALILEO(self):
        """Message constant 'SERVICE_GALILEO'."""
        return self.__constants['SERVICE_GALILEO']

    def __init__(self):
        self.status = 0
        self.service = 0

class DoraNavSatFix :
    __constants = {
        'COVARIANCE_TYPE_UNKNOWN': 0,
        'COVARIANCE_TYPE_APPROXIMATED': 1,
        'COVARIANCE_TYPE_DIAGONAL_KNOWN': 2,
        'COVARIANCE_TYPE_KNOWN': 3,
    }
    def COVARIANCE_TYPE_UNKNOWN(self):
        """Message constant 'STATUS_NO_FIX'."""
        return self.__constants['COVARIANCE_TYPE_UNKNOWN']
    def COVARIANCE_TYPE_APPROXIMATED(self):
        """Message constant 'STATUS_NO_FIX'."""
        return self.__constants['COVARIANCE_TYPE_APPROXIMATED']
    def COVARIANCE_TYPE_DIAGONAL_KNOWN(self):
        """Message constant 'STATUS_NO_FIX'."""
        return self.__constants['COVARIANCE_TYPE_DIAGONAL_KNOWN']
    def COVARIANCE_TYPE_KNOWN(self):
        """Message constant 'STATUS_NO_FIX'."""
        return self.__constants['COVARIANCE_TYPE_KNOWN']
    

    def __init__(self):
        self.header=Header()
        self.status=NavSatStatus()
        self.latitude=0.0
        self.longitude=0.0
        self.altitude=0.0
        self.position_covariance = [0.0] * 9
        self.position_covariance_type = 0

    def __str__(self):
        return f"DoraNavSatFix: frame_id={self.header.frame_id}, sec={self.header.stamp.sec}, nanosec={self.header.stamp.nanosec}," \
               f"latitude={self.latitude},status={self.status.status},service={self.status.service},longitude={self.longitude}, altitude={self.altitude}" \
               f"position_covariance={self.position_covariance}, position_covariance_type={self.position_covariance_type}"


class Quaternion :
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0 


class DoraQuaternionStamped :
    def __init__(self):
        self.header=Header()
        self.quaternion=Quaternion()

    def __str__(self):
        return f"DoraQuaternionStamped: frame_id={self.header.frame_id}, sec={self.header.stamp.sec}, nanosec={self.header.stamp.nanosec}," \
               f"x={self.quaternion.x},y={self.quaternion.y},z={self.quaternion.z},w={self.quaternion.w}"


class DoraTwistStamped :
    def __init__(self):
        self.header=Header()
        self.x_linear = 0.0
        self.y_linear = 0.0

    def __str__(self):
        return f"DoraTwistStamped: frame_id={self.header.frame_id}, sec={self.header.stamp.sec}, nanosec={self.header.stamp.nanosec}," \
               f"x_linear={self.x_linear},y_linear={self.y_linear}"


 
class DoraNMEADriver :
   def __init__(self, frame_id="gps", tf_prefix="",use_RMC = False):
        self.frame_id = frame_id
        self.use_RMC = use_RMC
        self.prefix = tf_prefix
        self.current_fix = DoraNavSatFix()
        self.current_heading = DoraQuaternionStamped()
        self.current_vel = DoraTwistStamped()
        self.getParameter_NavSatStatus = NavSatStatus()
        self.getParameter_DoraNavSatFix = DoraNavSatFix()


        self.valid_fix = False
    
        # epe = estimated position error
        self.default_epe_quality0 =  1000000
        self.default_epe_quality1 =  4.0
        self.default_epe_quality2 =  0.1
        self.default_epe_quality4 =  0.02
        self.default_epe_quality5 =  4.0
        self.default_epe_quality9 =  3.0

        self.using_receiver_epe = False

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        """这个字典的格式是将GGA消息中的定位质量指示（fix type）作为键，
        每个条目包含一个元组，其中包括
        默认的估计位置误差、NavSatStatus值和NavSatFix协方差值。"""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                self.getParameter_NavSatStatus.STATUS_NO_FIX(),
                self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_UNKNOWN()
            ],
            # Invalid无效定位
            0: [
                self.default_epe_quality0,
                self.getParameter_NavSatStatus.STATUS_NO_FIX(),
                self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_UNKNOWN()
            ],
            # SPS单点定位
            1: [
                self.default_epe_quality1,
                self.getParameter_NavSatStatus.STATUS_FIX(),
                self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_APPROXIMATED()
            ],
            # DGPS差分定位
            2: [
                self.default_epe_quality2,
                self.getParameter_NavSatStatus.STATUS_SBAS_FIX(),
                self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_APPROXIMATED()
            ],
            # RTK Fix实时运动静态
            4: [
                self.default_epe_quality4,
                self.getParameter_NavSatStatus.STATUS_GBAS_FIX(),
                self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_APPROXIMATED()
            ],
            # RTK Float浮点解
            5: [
                self.default_epe_quality5,
                self.getParameter_NavSatStatus.STATUS_GBAS_FIX(),
                self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_APPROXIMATED()
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                self.getParameter_NavSatStatus.STATUS_GBAS_FIX(),
                self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_APPROXIMATED()
            ]
        }
 
   def get_frame_id(self):
        if len(self.prefix):
            return '%s/%s' % (self.prefix, self.frame_id)
        return self.frame_id

   def publish_parsed_sentence(self, parsed_sentence, frame_id, timestamp=None):
        if timestamp:
            current_time = timestamp
        else:
            current_time=Timestamp.now()



        if not self.use_RMC and 'GGA' in parsed_sentence:
            self.current_fix.header.stamp = current_time
            self.current_fix.header.frame_id = frame_id
            self.current_fix.position_covariance_type = self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_APPROXIMATED()

            data = parsed_sentence['GGA']
            fix_type = data['fix_type']
            if not (fix_type in self.gps_qualities):
                fix_type = -1
            gps_qual = self.gps_qualities[fix_type]
            default_epe = gps_qual[0]
            self.current_fix.status.status = gps_qual[1]
            self.current_fix.position_covariance_type = gps_qual[2]
            if self.current_fix.status.status > 0:
                self.valid_fix = True
            else:
                self.valid_fix = False

            self.current_fix.status.service = self.getParameter_NavSatStatus.SERVICE_GPS()
            latitude = data['latitude']
            if data['latitude_direction'] == 'S':
                latitude = -latitude
            self.current_fix.latitude = latitude

            longitude = data['longitude']
            if data['longitude_direction'] == 'W':
                longitude = -longitude
            self.current_fix.longitude = longitude

            # 高度是相对于椭球体的，因此需要进行平均海平面的调整
            altitude = data['altitude'] + data['mean_sea_level']
            self.current_fix.altitude = altitude

            # 如果没收到带有 epe的 GST 消息，就使用默认的 EPE 标准差
            if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                self.lon_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                self.lat_std_dev = default_epe
            if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                self.alt_std_dev = default_epe * 2

            hdop = data['hdop']
            self.current_fix.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
            self.current_fix.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
            self.current_fix.position_covariance[8] = (2 * hdop * self.alt_std_dev) ** 2  # FIXME
            #发布current_fix
            news_type = 1
            return news_type, self.current_fix

        elif not self.use_RMC and 'VTG' in parsed_sentence:
            data = parsed_sentence['VTG']
            # 只有在接收到有效的GGA定位修正时，才报告VTG数据
            if self.valid_fix:
                self.current_vel.header.stamp = current_time
                self.current_vel.header.frame_id = frame_id
                self.current_vel.x_linear = data['speed'] * math.sin(data['true_course'])
                self.current_vel.y_linear = data['speed'] * math.cos(data['true_course'])
                #发布current_vel
                news_type = 3
                return news_type, self.current_vel



        elif 'RMC' in parsed_sentence:
            data = parsed_sentence['RMC']
            # 只有在设置了use_RMC标志(参数初始化是false)时，才发布来自RMC的修复。
            if self.use_RMC:
                if data['fix_valid']:
                    self.current_fix.status.status = self.getParameter_NavSatStatus.STATUS_FIX
                else:
                    self.current_fix.status.status = self.getParameter_NavSatStatus.STATUS_NO_FIX

                self.current_fix.status.service = self.getParameter_NavSatStatus.SERVICE_GPS

                self.current_fix.header.stamp = current_time
                self.current_fix.header.frame_id = frame_id
                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                self.current_fix.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                self.current_fix.longitude = longitude

                self.current_fix.altitude = float('NaN')
                self.current_fix.position_covariance_type = \
                self.getParameter_DoraNavSatFix.COVARIANCE_TYPE_UNKNOWN()
                # 发布current_fix
                news_type = 1
                return news_type, self.current_fix


            if data['fix_valid']:
                self.current_vel.header.stamp = current_time
                self.current_vel.header.frame_id = frame_id
                self.current_vel.x_linear = data['speed'] * math.sin(data['true_course'])
                self.current_vel.y_linear = data['speed'] * math.cos(data['true_course'])
                # 发布current_vel
                news_type = 3
                return news_type, self.current_vel



        elif 'GST' in parsed_sentence:
            data = parsed_sentence['GST']
            # 如果可用，使用接收器提供的误差估计。
            self.using_receiver_epe = True
            self.lon_std_dev = data['lon_std_dev']
            self.lat_std_dev = data['lat_std_dev']
            self.alt_std_dev = data['alt_std_dev']



        elif 'HDT' in parsed_sentence:
            data = parsed_sentence['HDT']
            if data['heading']:
                 self.current_heading.header.stamp = current_time
                 self.current_heading.header.frame_id = frame_id
                 q = quaternion_from_euler(0, 0, math.radians(data['heading']))
                 self.current_heading.quaternion.x = q[1]
                 self.current_heading.quaternion.y = q[2]
                 self.current_heading.quaternion.z = q[3]
                 self.current_heading.quaternion.w = q[0]
                 # 发布current_heading
                 news_type = 2
                 return news_type, self.current_heading



        news_type = 0
        return news_type, news_type





 

   