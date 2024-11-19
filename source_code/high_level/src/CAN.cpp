#include "CAN.h"

CAN::CAN(const char *interface)
{
    // Open a socket
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket < 0)
    {
        std::cerr << "Error while opening socket: " << strerror(errno) << std::endl;
        return;
    }

    // Bind the socket to the CAN interface
    std::strcpy(ifr.ifr_name, interface);
    ioctl(can_socket, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        std::cerr << "Error in socket bind: " << strerror(errno) << std::endl;
        return;
    }

    std::cout << "CAN interface opened successfully" << std::endl;
}

CAN::~CAN()
{
    // Close the socket
    close(can_socket);
}

void CAN::send(const can_frame &frame)
{
    // Send the CAN message
    int nbytes = write(can_socket, &frame, sizeof(frame));
    if (nbytes != sizeof(frame))
    {
        std::cerr << "Error sending CAN message: " << strerror(errno) << std::endl;
    }
    else
    {
        std::cout << "Sent CAN ID: " << std::hex << frame.can_id << std::endl;
        std::cout << "Data: " << std::endl;
        for (int i = 0; i < frame.can_dlc; i++)
        {
            std::cout << std::hex << (int)frame.data[i] << " ";
        }
        std::cout << std::endl;
    }
}

void CAN::receive(can_frame &frame)
{
    // Receive the CAN message
    int nbytes = read(can_socket, &frame, sizeof(frame));
    if (nbytes < 0)
    {
        std::cerr << "Error receiving CAN message: " << strerror(errno) << std::endl;
    }
    else if (nbytes > 0)
    {
        std::cout << "Received CAN ID: " << std::hex << frame.can_id << std::endl;
        std::cout << "Data: " << std::endl;
        for (int i = 0; i < frame.can_dlc; i++)
        {
            std::cout << std::hex << (int)frame.data[i] << " ";
        }
        std::cout << std::endl;
    }
}