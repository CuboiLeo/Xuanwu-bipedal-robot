#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include "STM32_protocol.h"
#include "motor.h"
#include "IMU.h"
#include "command.h"

int main()
{
    Motor motor;
    IMU imu;
    Command command;
    STM32 stm32;

    CAN can("can0");
    while (true)
    {
        stm32.sendData(can, motor);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        stm32.receiveData(can, motor, imu, command);
    }
    return 0;
}