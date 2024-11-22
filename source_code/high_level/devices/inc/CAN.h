#ifndef CAN_H
#define CAN_H

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <cstring>
#include <unistd.h>
#include <iostream>

#define ENABLE_CAN_DEBUG (0)

class CAN
{
public:
    CAN(const char *interface);
    ~CAN();
    void send(const can_frame &frame);
    void receive(can_frame &frame);
private:
    int can_socket = -1;
    struct ifreq ifr = {};
    struct sockaddr_can addr = {};
};

#endif