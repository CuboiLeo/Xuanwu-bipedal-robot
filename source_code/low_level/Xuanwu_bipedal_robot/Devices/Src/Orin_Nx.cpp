#include "Orin_NX.h"

Orin::Orin()
{
    send_data[0] = 0x01;
    send_data[1] = 0x02;
    send_data[2] = 0x03;
    send_data[3] = 0x04;
    send_data[4] = 0x05;
    send_data[5] = 0x06;
    send_data[6] = 0x07;
    send_data[7] = 0x08;
}

void Orin::sendData(void)
{
    fdcanx_send_data(&hfdcan3, 0x66, send_data, 8);
}

void Orin::receiveData(void)
{
    uint16_t rec_id;
    fdcanx_receive(&hfdcan3, &rec_id, receive_data);
}