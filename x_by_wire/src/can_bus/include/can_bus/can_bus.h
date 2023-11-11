/*
* @author: Zeyu Ren
* @date: 2023.6.13 
* @brief: Carsmos线下比赛，CAN_BUS核心模块
*/

#ifndef CAN_BUS_CAN_BUS_H_
#define CAN_BUS_CAN_BUS_H_

#include "socket_msg.h"
#include "socket_can.h"
#include <chrono>
#include <memory>
#include "msg_interfaces/msg/vehicle_status.hpp"
#include "msg_interfaces/msg/raw_control_command.hpp"
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

class CAN_BUS : public rclcpp::Node
{
public:
    CAN_BUS();
    ~CAN_BUS();
protected:
    void control_Command_Callback(const msg_interfaces::msg::RawControlCommand::SharedPtr msg);
    void timer_Callback();

    void receive_CAN_data();
    void deal_Received_CAN_Data(struct can_frame& CANData);
    void deal_VCU_Vehicle_Diagnosis(struct can_frame& CANData);
    void deal_VCU_Vehicle_Status(struct can_frame& CANData);
    void deal_Wheel_Speed(struct can_frame& CANData);
private:
    rclcpp::Subscription<msg_interfaces::msg::RawControlCommand>::SharedPtr control_Command_Sub;
    rclcpp::Publisher<msg_interfaces::msg::VehicleStatus>::SharedPtr vehicle_Status_Pub;  
    rclcpp::TimerBase::SharedPtr m_Timer;

    // 接收端
    bool sub_Flag;
    unsigned int Throttle_Percentage_Cmd; // [0, 100]
    unsigned int Brake_Percentage_Cmd; // [0, 100]
    double Tire_Angle_Deg_Cmd; // [-30, 30]

    // for transmit
    unsigned int cnt; // 计数器，每五次发送一次线控灯光指令
    iECU_Control_Flag iecu_Control_Flag; // 0x501
    iECU_Control_Steering iecu_Control_Steering; // 0x502
    iECU_Control_Brake iecu_Control_Brake; // 0x503
    iECU_Control_Accelerate iecu_Control_Accelerate; // 0x504
    iECU_Control_Harware iecu_Control_Harware; // 0x506

    // for receive
    VCU_Vehicle_Diagnosis vcu_Vehicle_Diagnosis; // 0x301
    VCU_Vehicle_Status vcu_Vehicle_Status; // 0x304
    Wheel_Speed wheel_Speed; // 0x305

    std::unique_ptr<Socket_CAN> psocket_CAN;
};

#endif