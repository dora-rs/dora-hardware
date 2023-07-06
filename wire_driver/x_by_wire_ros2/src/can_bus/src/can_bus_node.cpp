#include "../include/can_bus/can_bus.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CAN_BUS>());
    rclcpp::shutdown();

    return 0;
}
