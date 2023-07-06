float64 speed # 车速，单位：km/h
float64 angle # 车辆前轮转角，单位：deg

uint32 drive_mode_state # 驾驶模式: 0: 人工模式、1: iECU模式、2: 遥控模式、3: 半自动驾驶模式(只可控制转向)、4: 半自动驾驶模式(只可控驱动制动)
uint32 vehicle_gear # 车辆档位状态反馈，0: N挡、1: D挡、2: R挡、3: P挡
float64 vehicle_break # 实时制动压力，单位：MPa
uint32 wireacc_pedal_pos # 线控油门踏板百分比反馈