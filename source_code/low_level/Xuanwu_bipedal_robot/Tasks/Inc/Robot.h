#ifndef ROBOT_H
#define ROBOT_H

#include "stdint.h"
#include "User_Math.h"
#include "Robot_Types.h"

#define ROBOT_MAX_VEL (1.0f)
#define ROBOT_MAX_ANG_VEL (0.5f)

static constexpr float ROBOT_TASK_PERIOD = 0.002f; // Robot task period in seconds

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
    Direction_Vector getRefFootPosLeft() const { return ref_left_foot_pos; };
    Direction_Vector getRefFootPosRight() const { return ref_right_foot_pos; };
    void setRefFootPosLeft(const Direction_Vector &position) { ref_left_foot_pos = position; };
    void setRefFootPosRight(const Direction_Vector &position) { ref_right_foot_pos = position; };
    Direction_Vector getActFootPosLeft() const { return act_left_foot_pos; };
    Direction_Vector getActFootPosRight() const { return act_right_foot_pos; };
    void setActFootPosLeft(const Direction_Vector &position) { act_left_foot_pos = position; };
    void setActFootPosRight(const Direction_Vector &position) { act_right_foot_pos = position; };

    // Public interface for interacting with CoM position
    Direction_Vector getActCoMPos() const { return act_CoM_pos; };
    void setActCoMPos(const Direction_Vector &position) { act_CoM_pos = position; };
    Direction_Vector getRefCoMPos() const { return ref_CoM_pos; };
    void setRefCoMPos(const Direction_Vector &position) { ref_CoM_pos = position; };

    // Public interface for interacting with ZMP position
    Direction_Vector getActZMPPos() const { return act_ZMP_pos; };
    void setActZMPPos(const Direction_Vector &position) { act_ZMP_pos = position; };
    Direction_Vector getRefZMPPos() const { return ref_ZMP_pos; };
    void setRefZMPPos(const Direction_Vector &position) { ref_ZMP_pos = position; };

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
    // Internal state of the robot
    Joint_Angle act_left_leg_angles;  // Actual left leg joint angles
    Joint_Angle act_right_leg_angles; // Actual right leg joint angles
    Joint_Angle ref_left_leg_angles;  // Reference left leg joint angles through inverse kinematics
    Joint_Angle ref_right_leg_angles; // Reference right leg joint angles through inverse kinematics

    Direction_Vector ref_left_foot_pos;  // Reference left foot position
    Direction_Vector ref_right_foot_pos; // Reference right foot position
    Direction_Vector act_left_foot_pos;  // Actual left foot position through forward kinematics
    Direction_Vector act_right_foot_pos; // Actual right foot position through forward kinematics

    Direction_Vector ref_CoM_pos; // Reference Center of Mass position
    Direction_Vector act_CoM_pos; // Center of Mass position
    Direction_Vector ref_ZMP_pos; // Reference Zero Moment Point position
    Direction_Vector act_ZMP_pos; // Zero Moment Point position

    Direction_Vector ref_robot_vel; // Reference robot velocity
    Direction_Vector ref_robot_ang_vel; // Reference robot angular velocity

    float battery_voltage;       // Battery voltage in volts
    uint16_t battery_adc_val[2]; // ADC values for battery voltage
};

#endif
