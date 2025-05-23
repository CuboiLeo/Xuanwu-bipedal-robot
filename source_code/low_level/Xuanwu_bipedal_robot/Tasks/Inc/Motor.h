#ifndef MOTOR_H
#define MOTOR_H

#include "DM4310driver.h"
#include "Robot.h"
#include "Delay.h"

#define FDCAN1_PowerUp(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, x)
#define FDCAN2_PowerUp(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, x)

class Motor
{
public:
    static constexpr uint8_t NUM_MOTORS = 10;
    static constexpr uint8_t ALL_JOINTS_ZEROED_FLAG = 1;

    Motor();
    void resetJoints(void);
    void sendAll(void);
    void disableAll(void);
    void enableAll(void);
    uint8_t returnZeroPos(uint8_t received_data_flag);
    void createVirtualBoundary(void);
    float getPos(const int motor) const { return motor_info[motor].para.pos; };
    float getVel(const int motor) const { return motor_info[motor].para.vel; };
    float getTor(const int motor) const { return motor_info[motor].para.tor; };
    float getRefPos(const int motor) const { return motor_info[motor].ctrl.pos_set; };
		float getRefTor(const int motor) const { return motor_info[motor].ctrl.tor_set; };
    void setPos(const int motor, const float pos) { motor_info[motor].ctrl.pos_set = pos; };
    void setVel(const int motor, const float vel) { motor_info[motor].ctrl.vel_set = vel; };
    void setTor(const int motor, const float tor) { motor_info[motor].ctrl.tor_set = tor; };
    uint8_t getSoftStartFlag(void) const { return soft_start_flag; };
    void setSoftStartFlag(const uint8_t &flag) { soft_start_flag = flag; };

    friend void fdcan1_rx_callback(void);
    friend void fdcan2_rx_callback(void);

private:
    motor_t motor_info[NUM_MOTORS];
		uint8_t all_joints_zeroed_flag;
    uint8_t soft_start_flag;

    float kps[NUM_MOTORS] = {10.0f, 80.0f, 80.0f, 80.0f, 10.0f,
                              10.0f, 80.0f, 80.0f, 80.0f, 10.0f};
    float kds[NUM_MOTORS] = {0.2f, 0.6f, 0.6f, 0.6f, 0.2f,
                              0.2f, 0.6f, 0.6f, 0.6f, 0.2f};
};

#endif
