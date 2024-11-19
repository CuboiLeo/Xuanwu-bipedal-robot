#include "CAN.h"
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>

int main()
{
    CAN can("can0");
    can_frame send_frame;
    send_frame.can_id = 0x123;
    send_frame.can_dlc = 8;
    for (int i = 0; i < send_frame.can_dlc; i++)
    {
        send_frame.data[i] = i;
    }

    can_frame receive_frame;
    receive_frame.can_dlc = 8;

    while (true)
    {
        //can.send(send_frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        can.receive(receive_frame);
    }
    return 0;
}