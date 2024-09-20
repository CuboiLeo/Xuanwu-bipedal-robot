#include "usart.h"
#include "cmsis_os.h"
#include "Robot.h"
#include "ET16S_Remote.h"
#include "Kinematics.h"
#include "DM4310ctrl.h"

Robot robot;
// FreeRTOS Task for controlling the robot
void Robot_Task(void const* argument) {
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(1);

    Foot_Position init_left_foot = {-0.095f, 0.0f, -0.559999f};
    robot.setRefFootPosLeft(init_left_foot);

    Remote_Init(&REMOTE_UART);  // Initialize the remote

    for (;;) {
        // Assign joint angles based on motor feedback
        Joint_Angle left_angles = {
            g_Motor[Left_Hip_Yaw].para.pos,
            g_Motor[Left_Hip_Roll].para.pos,
            g_Motor[Left_Hip_Pitch].para.pos,
            g_Motor[Left_Knee_Pitch].para.pos
        };
        Joint_Angle right_angles = {
            g_Motor[Right_Hip_Yaw].para.pos,
            g_Motor[Right_Hip_Roll].para.pos,
            g_Motor[Right_Hip_Pitch].para.pos,
            g_Motor[Right_Knee_Pitch].para.pos
        };

        // Update the robot's joint angles
        robot.setActJointAnglesLeft(left_angles);
        robot.setActJointAnglesRight(right_angles);

        kinematics.computeForwardKinematics(&robot);
        kinematics.computeInverseKinematics(&robot);

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}

// Constructor
Robot::Robot() {
    // Initialize robot state (optional)
    act_left_leg_angles = {0.0f, 0.0f, 0.0f, 0.0f};
    act_right_leg_angles = {0.0f, 0.0f, 0.0f, 0.0f};
    ref_left_foot_pos = {-0.095f, 0.0f, -0.559999f };
    ref_right_foot_pos = {0.0f, 0.0f, 0.0f};
    soft_start_flag = 0;
}

// Public methods for interacting with joint angles and foot position

Joint_Angle Robot::getActJointAnglesLeft() const {
    return act_left_leg_angles;
}

Joint_Angle Robot::getActJointAnglesRight() const {
    return act_right_leg_angles;
}

void Robot::setActJointAnglesLeft(const Joint_Angle& angles) {
    act_left_leg_angles = angles;
}

void Robot::setActJointAnglesRight(const Joint_Angle& angles) {
    act_right_leg_angles = angles;
}

Joint_Angle Robot::getRefJointAnglesLeft() const {
    return ref_left_leg_angles;
}

Joint_Angle Robot::getRefJointAnglesRight() const {
    return ref_right_leg_angles;
}

void Robot::setRefJointAnglesLeft(const Joint_Angle& angles) {
    ref_left_leg_angles = angles;
}

void Robot::setRefJointAnglesRight(const Joint_Angle& angles) {
    ref_right_leg_angles = angles;
}

Foot_Position Robot::getRefFootPosLeft() const {
    return ref_left_foot_pos;
}

Foot_Position Robot::getRefFootPosRight() const {
    return ref_right_foot_pos;
}

void Robot::setRefFootPosLeft(const Foot_Position& position) {
    ref_left_foot_pos = position;
}

void Robot::setRefFootPosRight(const Foot_Position& position) {
    ref_right_foot_pos = position;
}

Foot_Position Robot::getActFootPosLeft() const {
    return act_left_foot_pos;
}

Foot_Position Robot::getActFootPosRight() const {
    return act_right_foot_pos;
}

void Robot::setActFootPosLeft(const Foot_Position& position) {
    act_left_foot_pos = position;
}

void Robot::setActFootPosRight(const Foot_Position& position) {
    act_right_foot_pos = position;
}

uint8_t Robot::getSoftStartFlag() const {
    return soft_start_flag;
}

void Robot::setSoftStartFlag(uint8_t flag) {
    soft_start_flag = flag;
}
