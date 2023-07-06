#include "../include/can_bus/socket_can.h"

Socket_CAN::Socket_CAN(unsigned int id)
    : device_idx(id)
{}

Socket_CAN::~Socket_CAN() 
{}

signed int Socket_CAN::Tramsmit(struct can_frame* p_send, unsigned int len) {
    for (unsigned int idx = 0; idx < len; idx++) {
        signed int nbytes = static_cast<signed int>(write(s, &p_send[idx], sizeof(p_send[idx])));
        if (nbytes == -1) {
            std::cout << "Send message failed" << std::endl;
            return -1;
        }
    }
    return static_cast<signed int>(len);
}

signed int Socket_CAN::Receive(struct can_frame* p_receive, unsigned int len) {
    for (unsigned int idx = 0; idx < len; idx++) {
        signed int nbytes = static_cast<signed int>(read(s, &p_receive[idx], sizeof(p_receive[idx])));

        if (nbytes == -1) {
            std::cout << "Receive message failed" << std::endl;
            return -1;
        } else if (nbytes == 0) {
            return static_cast<signed int>(idx);
        }
    }
    return static_cast<signed int>(len);
}

bool Socket_CAN::Stop() {
    if (close(s) == -1) {
        std::cout << "Close socket can failed" << std::endl;
        return false;
    } else {
        std::cout << "Close socket can success" << std::endl;
        return true;
    }
}

bool Socket_CAN::Init()
{
    // 创建套接字
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1) {
        std::cout << "Error in open socket" << std::endl;
        return false;
    }

    // 指定CAN设备
    std::string can_name("can" + std::to_string(device_idx));
    strcpy(ifr.ifr_name, can_name.data());
    if ((ioctl(s, SIOCGIFINDEX, &ifr)) == -1) {
        std::cout << "Error in assign CAN device" << std::endl;
        return false;
    }

    // 将套接字与CAN设备进行绑定
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
        std::cout << "Error in sock bind" << std::endl;
        return false;
    }

    // 定义接收规则，只接收id等于0x301、0x304和0x305的报文
    rfilter[0].can_id = 0x301;
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id = 0x304;
    rfilter[1].can_mask = CAN_SFF_MASK;
    rfilter[2].can_id = 0x305;
    rfilter[2].can_mask = CAN_SFF_MASK;
    // 设置过滤规则
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) == -1) {
        std::cout << "Error in set socket FD's option" << std::endl;
        return false;
    };

    return true; 
}
