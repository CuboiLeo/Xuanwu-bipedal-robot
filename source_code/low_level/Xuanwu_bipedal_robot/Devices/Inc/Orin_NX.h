#ifndef ORIN_NX_H
#define ORIN_NX_H

#include "CAN_BSP.h"
#include "stdint.h"

class Orin
{
public:
    Orin();
    void sendData(void);
    void receiveData(void);

private:
    uint8_t send_data[8];
    uint8_t receive_data[8];
};

#endif
