There are two files. Msg interfaces is the msg defination packages, and can bus is can bus module package which uses the msg interfaces. What's more, the can bus modules is relates to the vehicles, and this package is for wuling hongguang Mine EV.

There are a few steps in use stage:
1. colcon build --packages-select msg_interfaces
2. colcon build --packages-select can_bus
3. sudo su
4. ip link set can0 type can bitrate 500000 triple-sampling off
5. ifconfig can0 up(或ip link set can0 up)
6. ros2 run can_bus can_bus
7. ros2 topic pub -r 20 /control_command msg_interfaces/msg/RawControlCommand "{throttle: 0, brake: 0, front_steer: 0.0}"
