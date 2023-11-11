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

int main()
{
	int s, nbytes;
	struct sockaddr_can addr;
    	struct ifreq ifr;

	struct can_frame send_frame[10];
	memset(send_frame, 0, sizeof(send_frame));

	s = socket(PF_CAN, SOCK_RAW, CAN_RAW); // 创建套接字
    	strcpy(ifr.ifr_name, "can1");
    	ioctl(s, SIOCGIFINDEX, &ifr); // 指定can设备

    	addr.can_family = AF_CAN;
    	addr.can_ifindex = ifr.ifr_ifindex;
    	bind(s, (struct sockaddr*)&addr, sizeof(addr)); // 将套接字与can设备进行绑定

	setsockopt(s,SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	while (true) {
		// 发送报文
		send_frame[0].can_id = 0x301 & CAN_SFF_MASK;
		send_frame[0].can_dlc = 0x08;
		send_frame[0].data[0] = 1;

                send_frame[1].can_id = 0x302 & CAN_SFF_MASK;
                send_frame[1].can_dlc = 0x08;
                send_frame[1].data[0] = 2;

		send_frame[2].can_id = 0x303 & CAN_SFF_MASK;
                send_frame[2].can_dlc = 0x08;
                send_frame[2].data[0] = 3;

                send_frame[3].can_id = 0x304 & CAN_SFF_MASK;
                send_frame[3].can_dlc = 0x08;
                send_frame[3].data[0] = 4;

                send_frame[4].can_id = 0x305 & CAN_SFF_MASK;
                send_frame[4].can_dlc = 0x08;
                send_frame[4].data[0] = 5;

		send_frame[5].can_id = 0x306 & CAN_SFF_MASK;
                send_frame[5].can_dlc = 0x08;
                send_frame[5].data[0] = 6;

                send_frame[6].can_id = 0x307 & CAN_SFF_MASK;
                send_frame[6].can_dlc = 0x08;
                send_frame[6].data[0] = 7;

                send_frame[7].can_id = 0x308 & CAN_SFF_MASK;
                send_frame[7].can_dlc = 0x08;
                send_frame[7].data[0] = 8;

                send_frame[8].can_id = 0x309 & CAN_SFF_MASK;
                send_frame[8].can_dlc = 0x08;
                send_frame[8].data[0] = 9;

              	send_frame[9].can_id = 0x30A & CAN_SFF_MASK;
                send_frame[9].can_dlc = 0x08;
                send_frame[9].data[0] = 10;

		for (unsigned int idx = 0; idx < 10; idx++) {
			nbytes = write(s, &send_frame[idx], sizeof(send_frame[idx]));
		}

		usleep(50000);

	}

	return 0;
}
