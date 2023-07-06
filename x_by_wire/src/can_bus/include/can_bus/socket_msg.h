/*
* @author: Zeyu Ren
* @date: 2023.6.12 
* @brief: CAN Msg Defination
* @ref: WLHGCAN0_500kbs_v5.dbc
*/

#ifndef CAN_BUS_SOCKET_MSG_H_
#define CAN_BUS_SOCKET_MSG_H_

#define MAX_TIRE_ANGLE_DEG 30
#define MAX_BRAKE_CMD 100
#define MAX_ACC_CMD 100

/*
* Msg Name: iECU_Control_Flag
* Msg ID: 0x501
* Msg Send Type: Cycle(20 ms)
* iECU -> VCU
*/
struct iECU_Control_Flag
{
    iECU_Control_Flag()
        : iECU_Control_Request_Flag(0x0)
    {}
    unsigned char iECU_Control_Request_Flag; // 0表示无效，1表示有效
};
typedef iECU_Control_Flag iECU_Control_Flag;

/*
* Msg Name: iECU_Control_Steering
* Msg ID: 0x502
* Msg Send Type: Cycle(20 ms)
* Comment: iECU转向请求
* iECU -> VCU
*/
struct iECU_Control_Steering
{
    iECU_Control_Steering()
        : iECU_Steering_Valid(0x0)
        , iECU_Tire_Speed_Cmd(0x06)
        , iECU_Tire_Angle_Cmd(0x12c)
    {}
    
    void set_Tire_Angle_Cmd_Dbl(const double& Tire_Angle_Deg_Cmd_Dbl)
    {
        double Tire_Angle_Deg_Cmd_Dbl_Lim = (Tire_Angle_Deg_Cmd_Dbl > MAX_TIRE_ANGLE_DEG) ? MAX_TIRE_ANGLE_DEG : Tire_Angle_Deg_Cmd_Dbl;
        Tire_Angle_Deg_Cmd_Dbl_Lim = (Tire_Angle_Deg_Cmd_Dbl_Lim < -MAX_TIRE_ANGLE_DEG) ? -MAX_TIRE_ANGLE_DEG : Tire_Angle_Deg_Cmd_Dbl_Lim;
        
        iECU_Tire_Angle_Cmd = static_cast<unsigned short>((Tire_Angle_Deg_Cmd_Dbl_Lim + 30.0) / 0.1); 
    }

    unsigned char iECU_Steering_Valid; // 0转向无效，1转向有效
    unsigned char iECU_Tire_Speed_Cmd; // 轮端角速度，deg/s，取值范围[0, 20]，建议取值5~7 
    unsigned short iECU_Tire_Angle_Cmd; // -30到30度前轮转角，deg
};
typedef iECU_Control_Steering iECU_Control_Steering;

/*
* Msg Name: iECU_Control_Brake
* Msg ID: 0x503
* Msg Send Type: Cycle(20 ms)
* Comment: iECU制动请求
* iECU -> VCU
*/
struct iECU_Control_Brake
{
    iECU_Control_Brake()
        : iECU_DBS_Valid(0x0)
        , iECU_BrakePressure_Cmd(0x0)
    {}

    void set_Brake_Cmd_Dbl(const unsigned int& Brake_Cmd_Dbl)
    {
        unsigned int Brake_Cmd_Dbl_Lim = (Brake_Cmd_Dbl > MAX_BRAKE_CMD) ? MAX_BRAKE_CMD : Brake_Cmd_Dbl;
        
        iECU_BrakePressure_Cmd = static_cast<unsigned char>(Brake_Cmd_Dbl_Lim);
    }

    unsigned char iECU_DBS_Valid; // 0制动无效，1制动有效
    unsigned char iECU_BrakePressure_Cmd; // 0~100%压力请求，对应0~8Mpa
};
typedef struct iECU_Control_Brake iECU_Control_Brake;

/*
* Msg Name: iECU_Control_Accelerate
* Msg ID: 0x504
* Msg Send Type: Cycle(20 ms)
* Comment: iECU驱动请求
* iECU -> VCU
*/
struct iECU_Control_Accelerate
{
    iECU_Control_Accelerate()
        : iECU_Accelerate_Valid(0x0)
        , iECU_Gear_Request(0x0)
        , iECU_Acc_Cmd(0x0)
    {}

    void set_Acc_Cmd_Dbl(const unsigned int& Acc_Cmd_Dbl)
    {
        unsigned int Acc_Cmd_Dbl_Lim = (Acc_Cmd_Dbl > MAX_ACC_CMD) ? MAX_ACC_CMD : Acc_Cmd_Dbl;
        
        iECU_Acc_Cmd = static_cast<unsigned short>(100 * Acc_Cmd_Dbl_Lim);
    }

    unsigned char iECU_Accelerate_Valid; // 0驱动无效，1驱动有效
    unsigned char iECU_Gear_Request; // 0空档，1前进挡，2后退挡
    unsigned short iECU_Acc_Cmd; // 油门开度请求
};
typedef struct  iECU_Control_Accelerate iECU_Control_Accelerate;

/*
* Msg Name: iECU_Control_Harware
* Msg ID: 0x506
* Msg Send Type: Cycle(100 ms)
* Comment: iECU灯光请求
* iECU -> VCU
*/
struct iECU_Control_Harware
{
    iECU_Control_Harware()
        : iECU_Light(0x0)
        , iECU_Turn_Light(0x0)
        , iECU_Lock(0x0)
    {}
    unsigned char iECU_Light; // 0灯光关，1小灯，2近光灯，3远光灯
    unsigned char iECU_Turn_Light; // 0关，1左转灯亮，2右转灯亮
    unsigned char iECU_Lock; // 1解锁车锁
};
typedef iECU_Control_Harware iECU_Control_Harware;

//---------------------------------------------------------------------------------------------------------------
/*
* Msg Name: VCU_Vehicle_Diagnosis
* Msg ID: 0x301
* Msg Send Type: Event
*/
struct VCU_Vehicle_Diagnosis
{
    VCU_Vehicle_Diagnosis()
        : Eme_Stop_State(0x0)
        , Steering_State(0x0)
        , DBS_State(0x0)
        , Remote_State(0x0)
        , iECU_State(0x0)
    {}

    unsigned char Eme_Stop_State; // 0: 急停按钮未按下，1: 急停按钮按下
    unsigned char Steering_State; // 0: 无故障，1: 信号丢失
    unsigned char DBS_State; // 0: 无故障，1: 信号丢失
    unsigned char Remote_State; // 0: 无故障，1: 信号丢失
    unsigned char iECU_State; // 0: 无故障，1: 信号丢失
};
typedef struct VCU_Vehicle_Diagnosis VCU_Vehicle_Diagnosis;

/*
* Msg Name: VCU_Vehicle_Status
* Msg ID: 0x304
* Msg Send Type: Cycle(50 ms)
*/
struct VCU_Vehicle_Status
{
    VCU_Vehicle_Status()
        : Vehicle_Speed(0x320)
        , Vehicle_Break(0x0)
        , Tire_Angle(0x190)
        , Vehicle_Gear(0x2)
        , Drive_Mode_State(0x0)
        , Light_State(0x0)
        , Turn_Light_State(0x0)
    {}

    // 单位：km/h
    double get_Vehicle_Speed_Dbl() const
    {
        double Vehicle_Speed_Dbl = 0.1 * static_cast<double>(Vehicle_Speed) - 80.0;
        return Vehicle_Speed_Dbl;
    }

    // 单位：MPa
    double get_Vehicle_Break_Dbl() const
    {
        double Vehicle_Break_Dbl = 0.1 * static_cast<double>(Vehicle_Break);
        return Vehicle_Break_Dbl;
    }

    // 单位：°
    double get_Tire_Angle_Dbl() const
    {
        double Tire_Angle_Dbl = 0.1 * static_cast<double>(Tire_Angle) - 40.0;
        return Tire_Angle_Dbl;
    }

    unsigned short Vehicle_Speed; // 车辆实时车速
    unsigned short Vehicle_Break; // 车辆实时制动力(MPa) 0~8 MPa
    unsigned short Tire_Angle; // 车辆实时转角角度
    unsigned char Vehicle_Gear; // 车辆挡位状态反馈，1: D挡，2: N挡，3: R挡
    unsigned char Drive_Mode_State; // 0: 人工模式，1: iECU模式，2: 遥控模式，3: 半自动驾驶模式(只可控转向)，4: 半自动驾驶模式(只可控驱动制动)
    unsigned char Light_State; // 0: 关，1: 小灯亮，2: 近光灯亮，3: 远光灯亮
    unsigned char Turn_Light_State; // 0: 关，1: 左转灯亮，2: 右转灯亮
};
typedef struct VCU_Vehicle_Status VCU_Vehicle_Status;

/*
* Msg Name: Wheel_speed
* Msg ID: 0x305
* Msg Send Type: Cycle(50 ms)
* Comment: 四轮转速反馈
* Chassis -> VCU
*/
struct Wheel_Speed
{
    Wheel_Speed()
        : FL_Speed(0x0)
        , FR_Speed(0x0)
        , RL_Speed(0x0)
        , RR_Speed(0x0)
    {}

    unsigned short FL_Speed; // FL轮转速反馈
    unsigned short FR_Speed; // FR轮转速反馈
    unsigned short RL_Speed; // RL轮转速反馈
    unsigned short RR_Speed; // RR轮转速反馈
};
typedef struct Wheel_Speed Wheel_Speed;

#endif