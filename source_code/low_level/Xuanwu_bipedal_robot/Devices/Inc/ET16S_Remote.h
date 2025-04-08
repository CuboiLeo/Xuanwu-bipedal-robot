#ifndef ET16S_Remote_H
#define ET16S_Remote_H

#define REMOTE_UART (huart5)

#include "usart.h"
#include <stdint.h>

class Remote
{
public:
    Remote();
    static constexpr float CHANNEL_MAX_VALUE = 671.0f;
    enum switch_state
    {
        SWITCH_DOWN = -1,
        SWITCH_MIDDLE = 0,
        SWITCH_UP = 1
    };

    int16_t getLeftStickX(void) const { return left_stick.x; };
    int16_t getLeftStickY(void) const { return left_stick.y; };
    int16_t getRightStickX(void) const { return right_stick.x; };
    int16_t getRightStickY(void) const { return right_stick.y; };
    int8_t getSwitchSA(void) const { return switch_sa; };
    int8_t getSwitchSB(void) const { return switch_sb; };
    // int8_t getSwitchSC(void)const { return switch_sc; } ;
    int8_t getSwitchSD(void) const { return switch_sd; };
    int8_t getSwitchSE(void) const { return switch_se; };
    int8_t getSwitchSF(void) const { return switch_sf; };
    int8_t getSwitchSG(void) const { return switch_sg; };
    int8_t getSwitchSH(void) const { return switch_sh; };
    int16_t getLeftDial(void) const { return left_dial; };
    int16_t getRightDial(void) const { return right_dial; };

    static constexpr uint8_t REMOTE_BUFFER_SIZE = 25;
    static constexpr uint8_t REMOTE_CHANNEL_NUM = 16;
    void Init(UART_HandleTypeDef *huart);
    void processBuffer(void);
    void checkWatchdog(UART_HandleTypeDef *huart);

    friend void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

private:
    struct stick_position
    {
        int16_t x;
        int16_t y;
    };
    stick_position left_stick;
    stick_position right_stick;
    int8_t switch_sa;
    int8_t switch_sb;
    // uint8_t switch_sc; broken :(
    int8_t switch_sd;
    int8_t switch_se;
    int8_t switch_sf;
    int8_t switch_sg;
    int8_t switch_sh;
    int16_t left_dial;
    int16_t right_dial;
    int16_t channel_val[REMOTE_CHANNEL_NUM];
    uint8_t remote_buffer[REMOTE_BUFFER_SIZE];
    uint8_t watchdog;
};

#endif
