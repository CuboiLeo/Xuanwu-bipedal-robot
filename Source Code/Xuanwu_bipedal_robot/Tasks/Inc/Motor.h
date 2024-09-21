#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "DM4310driver.h"
#include "Robot.h"

#define	FDCAN1_PowerUp(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, x)
#define	FDCAN2_PowerUp(x) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, x)

class Motor
{
public:
    static constexpr uint8_t NUM_MOTORS = 8;
    static constexpr uint8_t ALL_JOINTS_ZEROED_FLAG = 1;

    Motor();
    void setAllJointsPos(Robot* robot);
    void resetJoints(void);
    void sendAll(void);
    void disableAll(void);
    void enableAll(void);
    uint8_t returnZeroPos(void);
    void createVirtualBoundary(void);
    float getPos(int motor) const { return motor_info[motor].para.pos; };
    uint8_t getSoftStartFlag(void) const { return soft_start_flag; };
    void setSoftStartFlag(const uint8_t& flag) { soft_start_flag = flag; };

    friend void fdcan1_rx_callback(void);
    friend void fdcan2_rx_callback(void);

private:
    motor_t motor_info[NUM_MOTORS];
    uint8_t soft_start_flag;
};
	
extern void Motor_Ctrl_Task(void const * argument);

#ifdef __cplusplus
}
#endif

#endif
