#include "../include/can_bus/can_bus.h"

CAN_BUS::CAN_BUS()
    : Node("CAN_bus_node")
    , sub_Flag(false)
    , Throttle_Percentage_Cmd(0)
    , Brake_Percentage_Cmd(0)
    , Tire_Angle_Deg_Cmd(0.0)
    , cnt(0)
{
    psocket_CAN = std::make_unique<Socket_CAN>(0);
    psocket_CAN->Init();

    control_Command_Sub = this->create_subscription<msg_interfaces::msg::RawControlCommand>(
        "control_command", 1, std::bind(&CAN_BUS::control_Command_Callback, this, _1));
            
    vehicle_Status_Pub = this->create_publisher<msg_interfaces::msg::VehicleStatus>("vehicle_status", 1);

    m_Timer = this->create_wall_timer(10ms, std::bind(&CAN_BUS::timer_Callback, this));
}

CAN_BUS::~CAN_BUS() {
    psocket_CAN->Stop();
}

void CAN_BUS::control_Command_Callback(const msg_interfaces::msg::RawControlCommand::SharedPtr msg)
{
    Throttle_Percentage_Cmd = msg->throttle;
    Brake_Percentage_Cmd = msg->brake;
    Tire_Angle_Deg_Cmd = msg->front_steer;
    sub_Flag = true;
    // RCLCPP_INFO(get_logger(), "Control Command Callback is exec...");
} 

void CAN_BUS::timer_Callback() {
    // RCLCPP_INFO(get_logger(), "Timer callback is exec...");

    cnt = cnt % 10;
    
    if (cnt % 5 == 0) {
	receive_CAN_data();
    }

    if (cnt % 5 == 0) {
        msg_interfaces::msg::VehicleStatus vehicle_Status_Msg;
        vehicle_Status_Msg.speed = vcu_Vehicle_Status.get_Vehicle_Speed_Dbl();
        vehicle_Status_Msg.angle = vcu_Vehicle_Status.get_Tire_Angle_Dbl();
        vehicle_Status_Msg.drive_mode_state = static_cast<unsigned int>(vcu_Vehicle_Status.Drive_Mode_State);
        vehicle_Status_Msg.vehicle_gear = static_cast<unsigned int>(vcu_Vehicle_Status.Vehicle_Gear);
        vehicle_Status_Msg.vehicle_break = vcu_Vehicle_Status.get_Vehicle_Break_Dbl();
        // vehicle_Status_Msg.wireacc_pedal_pos = vehicle_Status_2.get_WireAcc_PedalPos();        
        vehicle_Status_Pub->publish(vehicle_Status_Msg);          
    }
      
    if (cnt == 0) {
        struct can_frame CAN_Transmit_Data[5];
	memset(CAN_Transmit_Data, 0, sizeof(CAN_Transmit_Data));
        
        CAN_Transmit_Data[0].can_id = 0x501 & CAN_SFF_MASK;
        CAN_Transmit_Data[0].can_dlc = 0x08;
        if (sub_Flag) {
            iecu_Control_Flag.iECU_Control_Request_Flag = 0x1;
        } else {
            iecu_Control_Flag.iECU_Control_Request_Flag = 0x0;
        }
        CAN_Transmit_Data[0].data[0] = iecu_Control_Flag.iECU_Control_Request_Flag;

        CAN_Transmit_Data[1].can_id = 0x502 & CAN_SFF_MASK;
        CAN_Transmit_Data[1].can_dlc = 0x08;
        if (sub_Flag) {
            iecu_Control_Steering.iECU_Steering_Valid = 0x1;
        } else {
            iecu_Control_Steering.iECU_Steering_Valid = 0x0;
        }
        CAN_Transmit_Data[1].data[0] = iecu_Control_Steering.iECU_Steering_Valid;
        CAN_Transmit_Data[1].data[1] = iecu_Control_Steering.iECU_Tire_Speed_Cmd;
        iecu_Control_Steering.set_Tire_Angle_Cmd_Dbl(Tire_Angle_Deg_Cmd);
        CAN_Transmit_Data[1].data[4] = static_cast<unsigned char>(iecu_Control_Steering.iECU_Tire_Angle_Cmd & 0x00FF);
        CAN_Transmit_Data[1].data[5] = static_cast<unsigned char>((iecu_Control_Steering.iECU_Tire_Angle_Cmd & 0xFF00) >> 8);

        CAN_Transmit_Data[2].can_id = 0x503 & CAN_SFF_MASK;
        CAN_Transmit_Data[2].can_dlc = 0x08;
        if (sub_Flag) {
            iecu_Control_Brake.iECU_DBS_Valid = 0x1;    
        } else {
            iecu_Control_Brake.iECU_DBS_Valid = 0x0;
        }
        CAN_Transmit_Data[2].data[0] = iecu_Control_Brake.iECU_DBS_Valid;
        iecu_Control_Brake.set_Brake_Cmd_Dbl(Brake_Percentage_Cmd);
        CAN_Transmit_Data[2].data[1] = iecu_Control_Brake.iECU_BrakePressure_Cmd;

        CAN_Transmit_Data[3].can_id = 0x504 & CAN_SFF_MASK;
        CAN_Transmit_Data[3].can_dlc = 0x8;
        if (sub_Flag) {
            iecu_Control_Accelerate.iECU_Accelerate_Valid = 0x1;
            iecu_Control_Accelerate.iECU_Gear_Request = 0x1;
        } else {
            iecu_Control_Accelerate.iECU_Accelerate_Valid = 0x0;
            iecu_Control_Accelerate.iECU_Gear_Request = 0x0;
        }
        CAN_Transmit_Data[3].data[0] = iecu_Control_Accelerate.iECU_Accelerate_Valid;
        CAN_Transmit_Data[3].data[3] = iecu_Control_Accelerate.iECU_Gear_Request;
        iecu_Control_Accelerate.set_Acc_Cmd_Dbl(Throttle_Percentage_Cmd);
        CAN_Transmit_Data[3].data[5] = static_cast<unsigned char>(iecu_Control_Accelerate.iECU_Acc_Cmd & 0xFF);
        CAN_Transmit_Data[3].data[6] = static_cast<unsigned char>((iecu_Control_Accelerate.iECU_Acc_Cmd & 0xFF00) >> 8);

        CAN_Transmit_Data[4].can_id = 0x506 & CAN_SFF_MASK;
        CAN_Transmit_Data[4].can_dlc = 0x8;
        if (Tire_Angle_Deg_Cmd > 2.0) {
            iecu_Control_Harware.iECU_Turn_Light = 0x1;
        } else if (Tire_Angle_Deg_Cmd < -2.0) {
            iecu_Control_Harware.iECU_Turn_Light = 0x2;
        } else {
            iecu_Control_Harware.iECU_Turn_Light = 0x0;
        }
        CAN_Transmit_Data[4].data[0] = static_cast<unsigned char>((iecu_Control_Harware.iECU_Light & 0x7) +  
            ((iecu_Control_Harware.iECU_Turn_Light & 0xF) << 3) + ((iecu_Control_Harware.iECU_Lock & 0x1) << 7));

        psocket_CAN->Tramsmit(CAN_Transmit_Data, 5);                                
    } else if (cnt % 2 == 0) {
        struct can_frame CAN_Transmit_Data[4];
        memset(CAN_Transmit_Data, 0, sizeof(CAN_Transmit_Data));

        CAN_Transmit_Data[0].can_id = 0x501 & CAN_SFF_MASK;
        CAN_Transmit_Data[0].can_dlc = 0x08;
        if (sub_Flag) {
            iecu_Control_Flag.iECU_Control_Request_Flag = 0x1;
        } else {
            iecu_Control_Flag.iECU_Control_Request_Flag = 0x0;
        }
        CAN_Transmit_Data[0].data[0] = iecu_Control_Flag.iECU_Control_Request_Flag;

        CAN_Transmit_Data[1].can_id = 0x502 & CAN_SFF_MASK;
        CAN_Transmit_Data[1].can_dlc = 0x08;
        if (sub_Flag) {
            iecu_Control_Steering.iECU_Steering_Valid = 0x1;
        } else {
            iecu_Control_Steering.iECU_Steering_Valid = 0x0;
        }
        CAN_Transmit_Data[1].data[0] = iecu_Control_Steering.iECU_Steering_Valid;
        CAN_Transmit_Data[1].data[1] = iecu_Control_Steering.iECU_Tire_Speed_Cmd;
        iecu_Control_Steering.set_Tire_Angle_Cmd_Dbl(Tire_Angle_Deg_Cmd);
        CAN_Transmit_Data[1].data[4] = static_cast<unsigned char>(iecu_Control_Steering.iECU_Tire_Angle_Cmd & 0x00FF);
        CAN_Transmit_Data[1].data[5] = static_cast<unsigned char>((iecu_Control_Steering.iECU_Tire_Angle_Cmd & 0xFF00) >> 8);

        CAN_Transmit_Data[2].can_id = 0x503 & CAN_SFF_MASK;
        CAN_Transmit_Data[2].can_dlc = 0x08;
        if (sub_Flag) {
            iecu_Control_Brake.iECU_DBS_Valid = 0x1;    
        } else {
            iecu_Control_Brake.iECU_DBS_Valid = 0x0;
        }
        CAN_Transmit_Data[2].data[0] = iecu_Control_Brake.iECU_DBS_Valid;
        iecu_Control_Brake.set_Brake_Cmd_Dbl(Brake_Percentage_Cmd);
        CAN_Transmit_Data[2].data[1] = iecu_Control_Brake.iECU_BrakePressure_Cmd;

        CAN_Transmit_Data[3].can_id = 0x504 & CAN_SFF_MASK;
        CAN_Transmit_Data[3].can_dlc = 0x8;
        if (sub_Flag) {
            iecu_Control_Accelerate.iECU_Accelerate_Valid = 0x1;
            iecu_Control_Accelerate.iECU_Gear_Request = 0x1;
        } else {
            iecu_Control_Accelerate.iECU_Accelerate_Valid = 0x0;
            iecu_Control_Accelerate.iECU_Gear_Request = 0x0;
        }
        CAN_Transmit_Data[3].data[0] = iecu_Control_Accelerate.iECU_Accelerate_Valid;
        CAN_Transmit_Data[3].data[3] = iecu_Control_Accelerate.iECU_Gear_Request;
        iecu_Control_Accelerate.set_Acc_Cmd_Dbl(Throttle_Percentage_Cmd);
        CAN_Transmit_Data[3].data[5] = static_cast<unsigned char>(iecu_Control_Accelerate.iECU_Acc_Cmd & 0xFF);
        CAN_Transmit_Data[3].data[6] = static_cast<unsigned char>((iecu_Control_Accelerate.iECU_Acc_Cmd & 0xFF00) >> 8);

        psocket_CAN->Tramsmit(CAN_Transmit_Data, 4); 
    }
    cnt += 1;
}

void CAN_BUS::receive_CAN_data() {
    struct can_frame CAN_Frames[3];
    memset(CAN_Frames, 0, sizeof(CAN_Frames));

    unsigned int receive_num = static_cast<unsigned int>(psocket_CAN->Receive(CAN_Frames, 3));
    RCLCPP_INFO(get_logger(), "receive_num:%u", receive_num);
    // std::cout << receive_num << std::endl;
   
    if (receive_num > 0) {
        for (unsigned int idx = 0; idx < receive_num; idx++) {
	    std::cout << (CAN_Frames[idx].can_id & CAN_SFF_MASK) << "  ";
            deal_Received_CAN_Data(CAN_Frames[idx]);
        }
    }
    std::cout << std::endl;
}

void CAN_BUS::deal_Received_CAN_Data(struct can_frame& CANData)
{
    unsigned int CAN_Data_ID = (CANData.can_id & CAN_SFF_MASK);

    switch (CAN_Data_ID)
    {
    case 0x301:
        deal_VCU_Vehicle_Diagnosis(CANData);
        break;
    case 0x304:
        deal_VCU_Vehicle_Status(CANData);
        break;
    case 0x305:
        deal_Wheel_Speed(CANData);
    default:
        break;
    }
}

void CAN_BUS::deal_VCU_Vehicle_Diagnosis(struct can_frame& CANData)
{
    vcu_Vehicle_Diagnosis.Eme_Stop_State = CANData.data[0];
    vcu_Vehicle_Diagnosis.Steering_State = CANData.data[1];
    vcu_Vehicle_Diagnosis.DBS_State = CANData.data[2];
    vcu_Vehicle_Diagnosis.Remote_State = CANData.data[3];
    vcu_Vehicle_Diagnosis.iECU_State = CANData.data[4];
}

void CAN_BUS::deal_VCU_Vehicle_Status(struct can_frame& CANData)
{
    vcu_Vehicle_Status.Vehicle_Speed = static_cast<unsigned short>((CANData.data[1] << 8) + CANData.data[0]);
    vcu_Vehicle_Status.Vehicle_Break = static_cast<unsigned short>((CANData.data[3] << 8) + CANData.data[2]);
    vcu_Vehicle_Status.Tire_Angle = static_cast<unsigned short>((CANData.data[5] << 8) + CANData.data[4]);
    vcu_Vehicle_Status.Vehicle_Gear = CANData.data[6];
    vcu_Vehicle_Status.Drive_Mode_State = (CANData.data[7] & 0x07); 
    vcu_Vehicle_Status.Light_State = ((CANData.data[7] & 0x18) >> 3);
    vcu_Vehicle_Status.Turn_Light_State = ((CANData.data[7] & 0xe0) >> 5);
}

void CAN_BUS::deal_Wheel_Speed(struct can_frame& CANData)
{
    wheel_Speed.FL_Speed = static_cast<unsigned short>((CANData.data[1]<<8) + CANData.data[0]);
    wheel_Speed.FR_Speed = static_cast<unsigned short>((CANData.data[3]<<8) + CANData.data[2]);
    wheel_Speed.RL_Speed = static_cast<unsigned short>((CANData.data[5]<<8) + CANData.data[4]);
    wheel_Speed.RR_Speed = static_cast<unsigned short>((CANData.data[7]<<8) + CANData.data[6]);
}
