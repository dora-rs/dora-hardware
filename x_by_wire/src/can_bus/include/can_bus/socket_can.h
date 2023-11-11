/*
* @author: Zeyu Ren
* @date: 2023.6.12 
* @brief: Socket CAN Driver
*/

#ifndef CAN_BUS_SOCKET_CAN_H_
#define CAN_BUS_SOCKET_CAN_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <iostream>
#include <sys/time.h>

class Socket_CAN
{
public:
    Socket_CAN(unsigned int id = 0);
    ~Socket_CAN();
    signed int Tramsmit(struct can_frame* p_send, unsigned int len);
    signed int Receive(struct can_frame* p_receive, unsigned int len);
    bool Stop();
    bool Init();
private:
    unsigned int device_idx;
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_filter rfilter[3];
};

#endif