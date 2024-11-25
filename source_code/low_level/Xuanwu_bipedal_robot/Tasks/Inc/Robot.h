#ifndef ROBOT_H
#define ROBOT_H

#include "stdint.h"
#include "User_Math.h"
#include "Robot_Types.h"

#define ROBOT_MAX_VEL (1.0f)
#define ROBOT_MAX_ANG_VEL (0.5f)

static constexpr float ROBOT_TASK_PERIOD = 0.002f; // Robot task period in seconds
class Robot
{
public:
    Robot();

    // Public interface for interacting with robot velocity
    Direction_Vector getRefRobotVel() const { return ref_robot_vel; };
    void setRefRobotVel(const Direction_Vector &velocity) { ref_robot_vel = velocity; };

    // Public interface for interacting with robot angular velocity
    Direction_Vector getRefRobotAngVel() const { return ref_robot_ang_vel; };
    void setRefRobotAngVel(const Direction_Vector &velocity) { ref_robot_ang_vel = velocity; };

    // Public interface for interacting with battery voltage
    void initBatteryADC() const;
    float getBatteryVoltage();

private:
    Direction_Vector ref_robot_vel; // Reference robot velocity
    Direction_Vector ref_robot_ang_vel; // Reference robot angular velocity

    float battery_voltage;       // Battery voltage in volts
    uint16_t battery_adc_val[2]; // ADC values for battery voltage
};

#endif
