#ifndef ROBOT_H
#define ROBOT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
#include "User_Math.h"
#include "Robot_Types.h"
    // Number of joints per leg, end effector counts
    static constexpr int NUM_JOINTS = 5;

    // Denavit-Hartenberg parameters for the robot
    static constexpr DH_Parameter DH_Left_Leg[NUM_JOINTS] = {
        // a      alpha   d       theta
        {-0.135f, 0, 0, 0},       // Base - Hip Yaw
        {-0.095f, -PI / 2, 0, 0}, // Hip Yaw - Hip Roll
        {-0.09f, -PI / 2, 0, 0},  // Hip Roll - Hip Pitch
        {-0.18f, 0, 0, 0},        // Hip Pitch - Knee Pitch
        {-0.38f, 0, 0, 0}         // Knee Pitch - Foot
    };
    static constexpr DH_Parameter DH_Right_Leg[NUM_JOINTS] = {
        // a      alpha   d       theta
        {0.135f, 0, 0, 0},       // Base - Hip Yaw
        {0.095f, PI / 2, 0, 0},  // Hip Yaw - Hip Roll
        {-0.09f, -PI / 2, 0, 0}, // Hip Roll - Hip Pitch
        {-0.18f, 0, 0, 0},       // Hip Pitch - Knee Pitch
        {-0.38f, 0, 0, 0}        // Knee Pitch - Foot
    };
    // Joint angle offsets for the robot
    static constexpr float LEFT_LEG_HIP_ROLL_OFFSET = PI / 2;
    static constexpr float RIGHT_LEG_HIP_ROLL_OFFSET = -PI / 2;

    // Identifier of left and right legs for forward kinematics
    static constexpr uint8_t LEFT_LEG = 1;
    static constexpr uint8_t RIGHT_LEG = 2;
    class Robot
    {
    public:
        Robot();
        // Public interface for interacting with joint angles
        Joint_Angle getActJointAnglesLeft() const { return act_left_leg_angles; };
        Joint_Angle getActJointAnglesRight() const { return act_right_leg_angles; };
        void setActJointAnglesLeft(const Joint_Angle &angles) { act_left_leg_angles = angles; };
        void setActJointAnglesRight(const Joint_Angle &angles) { act_right_leg_angles = angles; };
        Joint_Angle getRefJointAnglesLeft() const { return ref_left_leg_angles; };
        Joint_Angle getRefJointAnglesRight() const { return ref_right_leg_angles; };
        void setRefJointAnglesLeft(const Joint_Angle &angles) { ref_left_leg_angles = angles; };
        void setRefJointAnglesRight(const Joint_Angle &angles) { ref_right_leg_angles = angles; };

        // Public interface for interacting with foot position
        Foot_Position getRefFootPosLeft() const { return ref_left_foot_pos; };
        Foot_Position getRefFootPosRight() const { return ref_right_foot_pos; };
        void setRefFootPosLeft(const Foot_Position &position) { ref_left_foot_pos = position; };
        void setRefFootPosRight(const Foot_Position &position) { ref_right_foot_pos = position; };
        Foot_Position getActFootPosLeft() const { return act_left_foot_pos; };
        Foot_Position getActFootPosRight() const { return act_right_foot_pos; };
        void setActFootPosLeft(const Foot_Position &position) { act_left_foot_pos = position; };
        void setActFootPosRight(const Foot_Position &position) { act_right_foot_pos = position; };

        // Public interface for interacting with battery voltage
        void initBatteryADC() const;
        float getBatteryVoltage();

    private:
        // Internal state of the robot
        Joint_Angle act_left_leg_angles;  // Actual left leg joint angles
        Joint_Angle act_right_leg_angles; // Actual right leg joint angles
        Joint_Angle ref_left_leg_angles;  // Reference left leg joint angles through inverse kinematics
        Joint_Angle ref_right_leg_angles; // Reference right leg joint angles through inverse kinematics

        Foot_Position ref_left_foot_pos;  // Reference left foot position
        Foot_Position ref_right_foot_pos; // Reference right foot position
        Foot_Position act_left_foot_pos;  // Actual left foot position through forward kinematics
        Foot_Position act_right_foot_pos; // Actual right foot position through forward kinematics

        float battery_voltage;       // Battery voltage in volts
        uint16_t battery_adc_val[2]; // ADC values for battery voltage
    };

#ifdef __cplusplus
}
#endif

#endif
