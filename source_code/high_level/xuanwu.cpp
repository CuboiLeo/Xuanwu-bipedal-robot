#include <iostream>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "STM32_protocol.h"
#include "motor.h"
#include "IMU.h"
#include "command.h"
#include "robot.h"
#include "kinematics.h"

// Shared data between threads
struct Shared_Data
{
    Motor motor;
    IMU imu;
    Command command;

    bool new_data = false;
};

Shared_Data shared_data;                // Global shared data
std::mutex shared_data_mutex;           // Mutex for shared data
std::condition_variable shared_data_cv; // Condition variable for shared data
STM32 stm32;     // STM32 object for CAN communication
CAN can("can0"); // CAN object for communication

// Thread function declarations
void CAN_receive_thread();
void CAN_send_thread();
void compute_thread();

int main()
{
    std::thread CAN_receive(CAN_receive_thread);
    std::thread compute(compute_thread);
    std::thread CAN_send(CAN_send_thread);

    CAN_receive.join();
    compute.join();
    CAN_send.join();

    return 0;
}

void compute_thread()
{
    Robot robot;           // Robot object for robot state
    Kinematics kinematics; // Kinematics object for kinematics computation
    float time = 0;
    while (true)
    {
        std::unique_lock<std::mutex> lock(shared_data_mutex); // Lock the shared data
        shared_data_cv.wait(lock, []
                            { return shared_data.new_data; }); // Wait for new data
        shared_data.new_data = false;

        // Compute the robot's state
        Joint_Angles left_act_angle = {shared_data.motor.getActPos(Left_Hip_Yaw), shared_data.motor.getActPos(Left_Hip_Roll), shared_data.motor.getActPos(Left_Hip_Pitch), shared_data.motor.getActPos(Left_Knee_Pitch)};
        Joint_Angles right_act_angle = {shared_data.motor.getActPos(Right_Hip_Yaw), shared_data.motor.getActPos(Right_Hip_Roll), shared_data.motor.getActPos(Right_Hip_Pitch), shared_data.motor.getActPos(Right_Knee_Pitch)};
        robot.setLegActAngles(left_act_angle, right_act_angle);

        // Compute the forward kinematics
        Direction_Vector left_act_pos = kinematics.computeForwardKinematics(robot.getLegActAngles(LEFT_LEG_ID), DH_Left_Leg);
        Direction_Vector right_act_pos = kinematics.computeForwardKinematics(robot.getLegActAngles(RIGHT_LEG_ID), DH_Right_Leg);
        robot.setFootActPos(left_act_pos, right_act_pos);

        // Compute the inverse kinematics
        time += 0.0001;
        Joint_Angles left_ref_angle = {1*sin(2*PI*time),2*sin(2*PI*time),3,4};
        Joint_Angles right_ref_angle = {5,6,7,8};
        //Joint_Angles left_ref_angle = kinematics.computeInverseKinematics(robot.getFootActPos(LEFT_LEG_ID), robot.getFootRefPos(LEFT_LEG_ID),robot.getLegActAngles(LEFT_LEG_ID), DH_Left_Leg);
        //Joint_Angles right_ref_angle = kinematics.computeInverseKinematics(robot.getFootActPos(RIGHT_LEG_ID), robot.getFootRefPos(RIGHT_LEG_ID),robot.getLegActAngles(RIGHT_LEG_ID), DH_Right_Leg);
        robot.setLegRefAngles(left_ref_angle, right_ref_angle);

        // Set the motor data
        robot.setMotorData(shared_data.motor);

        lock.unlock(); // Unlock the shared data
    }
}

void CAN_receive_thread()
{
    while (true)
    {
        stm32.receiveData(can); // Receive data from the STM32
        {
            std::lock_guard<std::mutex> lock(shared_data_mutex);                       // Lock the shared data
            stm32.decodeData(shared_data.motor, shared_data.imu, shared_data.command); // Decode the received data
            shared_data.new_data = true;                                               // Set the new data flag
        }
        shared_data_cv.notify_one(); // Notify the compute thread that new data is available
    }
}

void CAN_send_thread()
{
    while (true)
    {
        {
            std::lock_guard<std::mutex> lock(shared_data_mutex); // Lock the shared data
            stm32.encodeData(shared_data.motor);                 // Encode the data to send
        }
        stm32.sendData(can); // Send the data to the STM32
    }
}