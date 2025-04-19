#include <iostream>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <sched.h>
#include <pthread.h>
#include "STM32_protocol.h"
#include "motor.h"
#include "IMU.h"
#include "command.h"
#include "robot.h"
#include "kinematics.h"
#include "dynamics.h"
#include "controls.h"
#include "walking_patterns.h"
#include "estimations.h"
#include "data_exporter.h"
#include "RL_inference.h"

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
STM32 stm32;                            // STM32 object for CAN communication
CAN can("can0");                        // CAN object for communication

// Thread function declarations
void CAN_receive_thread();
void CAN_send_thread();
void compute_thread();
void set_thread_realtime(std::thread &thread, int priority);
void pinThreadToCore(std::thread &thr, int core_id);

int main()
{
    std::thread CAN_receive(CAN_receive_thread);
    std::thread compute(compute_thread);
    std::thread CAN_send(CAN_send_thread);

    set_thread_realtime(CAN_receive, 99);
    set_thread_realtime(compute, 99);
    set_thread_realtime(CAN_send, 99);

    pinThreadToCore(CAN_receive, 1);
    pinThreadToCore(compute, 2);
    pinThreadToCore(CAN_send, 3);

    CAN_receive.join();
    compute.join();
    CAN_send.join();

    return 0;
}

void compute_thread()
{
    Robot robot;                                                                    // Robot object for robot state
    Kinematics kinematics;                                                          // Kinematics object for kinematics computation
    Dynamics dynamics;                                                              // Dynamics object for dynamics computation
    Controls controls;                                                              // Controls object for control computation
    Estimations estimations;                                                        // Estimation object for estimation computation
    Walking_Patterns walking_patterns;                                              // Walking patterns object for walking pattern computation
    RL_Inference rl_inference("../../RL/trained_policy/flat_ground_walking_v1.pt"); // RL inference object for RL policy inference
    auto start = std::chrono::high_resolution_clock::now();
    auto next_cycle = std::chrono::steady_clock::now();

    while (true)
    {
        next_cycle += std::chrono::milliseconds(10); // 100 Hz

        std::unique_lock<std::mutex> lock(shared_data_mutex); // Lock the shared data
        shared_data_cv.wait(lock, []
                            { return shared_data.new_data; }); // Wait for new data
        shared_data.new_data = false;

        // Update the IMU data
        shared_data.imu.processData();

        // Set the leg act angles
        Joint_Angles left_act_angle = {shared_data.motor.getActPos(Left_Hip_Yaw), shared_data.motor.getActPos(Left_Hip_Roll), shared_data.motor.getActPos(Left_Hip_Pitch), shared_data.motor.getActPos(Left_Knee_Pitch), shared_data.motor.getActPos(Left_Ankle_Pitch)};
        Joint_Angles right_act_angle = {shared_data.motor.getActPos(Right_Hip_Yaw), shared_data.motor.getActPos(Right_Hip_Roll), shared_data.motor.getActPos(Right_Hip_Pitch), shared_data.motor.getActPos(Right_Knee_Pitch), shared_data.motor.getActPos(Right_Ankle_Pitch)};
        robot.setLegActAngles(left_act_angle, right_act_angle);

        // Set the leg act velocities
        Joint_Velocities left_act_vel = {shared_data.motor.getActVel(Left_Hip_Yaw), shared_data.motor.getActVel(Left_Hip_Roll), shared_data.motor.getActVel(Left_Hip_Pitch), shared_data.motor.getActVel(Left_Knee_Pitch), shared_data.motor.getActVel(Left_Ankle_Pitch)};
        Joint_Velocities right_act_vel = {shared_data.motor.getActVel(Right_Hip_Yaw), shared_data.motor.getActVel(Right_Hip_Roll), shared_data.motor.getActVel(Right_Hip_Pitch), shared_data.motor.getActVel(Right_Knee_Pitch), shared_data.motor.getActVel(Right_Ankle_Pitch)};
        robot.setLegActVel(left_act_vel, right_act_vel);

        // Set the leg act torques
        // Joint_Torques left_act_torque = {shared_data.motor.getActTor(Left_Hip_Yaw), shared_data.motor.getActTor(Left_Hip_Roll), shared_data.motor.getActTor(Left_Hip_Pitch), shared_data.motor.getActTor(Left_Knee_Pitch), shared_data.motor.getActTor(Left_Ankle_Pitch)};
        // Joint_Torques right_act_torque = {shared_data.motor.getActTor(Right_Hip_Yaw), shared_data.motor.getActTor(Right_Hip_Roll), shared_data.motor.getActTor(Right_Hip_Pitch), shared_data.motor.getActTor(Right_Knee_Pitch), shared_data.motor.getActTor(Right_Ankle_Pitch)};
        // robot.setLegActTorque(left_act_torque, right_act_torque);

        // Compute the center of mass position
        // Position CoM_pos = kinematics.computeCoMPos({robot.getLegActAngles(LEFT_LEG_ID), robot.getLegActAngles(RIGHT_LEG_ID)}, shared_data.imu.getRotationMatrix());
        // robot.setCoMActPos(CoM_pos);

        // Compute the center of mass acceleration
        // Acceleration CoM_accel = dynamics.computeCoMAccel(robot.getCoMActPos(), shared_data.imu.getAccel(), shared_data.imu.getGyro(), shared_data.imu.getGyroDot());
        // robot.setCoMActAccel(CoM_accel);

        // Compute the zero moment point position
        // Position ZMP_pos = dynamics.computeZMPPos(robot.getCoMActPos(), robot.getCoMActAccel());
        // robot.setZMPActPos(ZMP_pos);
        // std::cout << "CoM Position: " << CoM_pos.x << " " << CoM_pos.y << " " << CoM_pos.z << std::endl;
        // std::cout << "ZMP Position: " << ZMP_pos.x << " " << ZMP_pos.y << std::endl;

        // Estimate the center of mass states
        // estimations.estimateCoMStates(shared_data.imu.getVel(), shared_data.imu.getAccel());
        // Velocity estimated_CoM_vel = estimations.getEstimatedCoMVel();

        // Compute the foot forward kinematics
        // Pose left_act_pose = trans_to_pose(kinematics.computeFootFK(robot.getLegActAngles(LEFT_LEG_ID), LEFT_LEG_ID));
        // Pose right_act_pose = trans_to_pose(kinematics.computeFootFK(robot.getLegActAngles(RIGHT_LEG_ID), RIGHT_LEG_ID));
        // robot.setFootActPose(left_act_pose, right_act_pose);
        // std::cout << "Left Foot Pose: " << left_act_pose.position.x << " " << left_act_pose.position.y << " " << left_act_pose.position.z << " " << left_act_pose.orientation.roll << " " << left_act_pose.orientation.pitch << " " << left_act_pose.orientation.yaw << std::endl;
        // std::cout << "Right Foot Pose: " << right_act_pose.position.x << " " << right_act_pose.position.y << " " << right_act_pose.position.z << " " << right_act_pose.orientation.roll << " " << right_act_pose.orientation.pitch << " " << right_act_pose.orientation.yaw << std::endl;

        // Set the foot reference pose with walking pattern generator
        // Pose_Two_Foots foot_ref_poses = walking_patterns.gaitPlanner(robot.getCoMActPos(), estimations.getEstimatedCoMVel(), {robot.getFootActPose(LEFT_LEG_ID), robot.getFootActPose(RIGHT_LEG_ID)}, shared_data.imu.getEuler().roll);
        // robot.setFootRefPose(foot_ref_poses.left, foot_ref_poses.right);

        // Pose left_ref_pose = {{-L1, -0.001, -0.55}, {0, 0, 0}};
        // Pose right_ref_pose = {{L1, -0.001, -0.55}, {0, 0, 0}};
        // robot.setFootRefPose(left_ref_pose, right_ref_pose);

        // Compute the inverse kinematics
        // Joint_Angles left_ref_angle = kinematics.computeFootIK(robot.getFootActPose(LEFT_LEG_ID), robot.getFootRefPose(LEFT_LEG_ID), robot.getLegActAngles(LEFT_LEG_ID), LEFT_LEG_ID);
        // Joint_Angles right_ref_angle = kinematics.computeFootIK(robot.getFootActPose(RIGHT_LEG_ID), robot.getFootRefPose(RIGHT_LEG_ID), robot.getLegActAngles(LEFT_LEG_ID), RIGHT_LEG_ID);
        // robot.setLegRefAngles(left_ref_angle, right_ref_angle);
        // std::cout << "Left Angles:  " << left_ref_angle.hip_yaw << " | " << left_ref_angle.hip_roll << " | " << left_ref_angle.hip_pitch << " | " << left_ref_angle.knee_pitch << " | " << left_ref_angle.ankle_pitch << std::endl;
        // std::cout << "Right Angles: " << right_ref_angle.hip_yaw << " | " << right_ref_angle.hip_roll << " | " << right_ref_angle.hip_pitch << " | " << right_ref_angle.knee_pitch << " | " << right_ref_angle.ankle_pitch << std::endl;

        // Set the leg reference velocities
        // robot.setLegRefVel({0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});

        // Set the leg reference torques
        // double gait_phase = walking_patterns.getGaitPhase();
        // int step_counter = walking_patterns.getStepCounter();
        // Joint_Torques left_ref_torque;
        // Joint_Torques right_ref_torque;
        // if (gait_phase < walking_patterns.Tdbl)
        // {
        //     left_ref_torque = dynamics.computeGRFTorques({0, 0, MT * GRAVITY / 2, 0, MT * GRAVITY / 2 * L1, 0}, robot.getLegActAngles(LEFT_LEG_ID), LEFT_LEG_ID);
        //     right_ref_torque = dynamics.computeGRFTorques({0, 0, MT * GRAVITY / 2, 0, -MT * GRAVITY / 2 * L1, 0}, robot.getLegActAngles(RIGHT_LEG_ID), RIGHT_LEG_ID);
        // }
        // else
        // {
        //     if (step_counter % 2 == 0)
        //     {
        //         left_ref_torque = {0, 0, 0, 0, 0};
        //         right_ref_torque = dynamics.computeGRFTorques({0, 0, MT * GRAVITY, 0, -MT * GRAVITY * L1, 0}, robot.getLegActAngles(RIGHT_LEG_ID), RIGHT_LEG_ID);
        //     }
        //     else
        //     {
        //         left_ref_torque = dynamics.computeGRFTorques({0, 0, MT * GRAVITY, 0, MT * GRAVITY * L1, 0}, robot.getLegActAngles(LEFT_LEG_ID), LEFT_LEG_ID);
        //         right_ref_torque = {0, 0, 0, 0, 0};
        //     }
        // }
        // robot.setLegRefTorque(left_ref_torque, right_ref_torque);
        // robot.setLegRefTorque({0, 0, 0, 0, 0}, {0, 0, 0, 0, 0});
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        // std::cout << "Current time: " << elapsed.count() << " seconds" << std::endl;

        std::vector<float> cmd = {(float)shared_data.command.getLinearVel().x, (float)shared_data.command.getLinearVel().y, (float)shared_data.command.getAngularVel().z};
        std::vector<float> q = {(float)left_act_angle.hip_yaw, (float)left_act_angle.hip_roll, (float)left_act_angle.hip_pitch, (float)left_act_angle.knee_pitch, (float)left_act_angle.ankle_pitch, (float)right_act_angle.hip_yaw, (float)right_act_angle.hip_roll, (float)right_act_angle.hip_pitch, (float)right_act_angle.knee_pitch, (float)right_act_angle.ankle_pitch};
        std::vector<float> dq = {(float)left_act_vel.hip_yaw, (float)left_act_vel.hip_roll, (float)left_act_vel.hip_pitch, (float)left_act_vel.knee_pitch, (float)left_act_vel.ankle_pitch, (float)right_act_vel.hip_yaw, (float)right_act_vel.hip_roll, (float)right_act_vel.hip_pitch, (float)right_act_vel.knee_pitch, (float)right_act_vel.ankle_pitch};
        Eigen::Vector3d gyro = shared_data.imu.getGyro();
        std::vector<float> omega = {(float)gyro(0), (float)gyro(1), (float)gyro(2)};
        std::vector<float> eul_ang = {(float)shared_data.imu.getEuler().roll, (float)shared_data.imu.getEuler().pitch, (float)shared_data.imu.getEuler().yaw};

        std::vector<float> target_q = rl_inference.infer(elapsed.count(), cmd, q, dq, omega, eul_ang);
        Joint_Angles left_ref_angle = {target_q[0], target_q[1], target_q[2], target_q[3], target_q[4]};
        Joint_Angles right_ref_angle = {target_q[5], target_q[6], target_q[7], target_q[8], target_q[9]};
        // float scale = 0.0f;
        // Joint_Angles left_ref_angle = {0.0 * sin(2 * PI * elapsed.count() / 0.64), 0.2 * sin(2 * PI * elapsed.count() / 0.64), scale * sin(2 * PI * elapsed.count() / 0.64) + 0.152f, -2.0f*scale* sin(2 * PI * elapsed.count() / 0.64) - 0.36f, scale * sin(2 * PI * elapsed.count() / 0.64) + 0.208f};
        // Joint_Angles right_ref_angle = {0, 0, 0, 0, 0};
        robot.setLegRefAngles(left_ref_angle, right_ref_angle);
        // std::cout << "Left Angles: " << left_ref_angle.hip_yaw << " | " << left_ref_angle.hip_roll << " | " << left_ref_angle.hip_pitch << " | " << left_ref_angle.knee_pitch << " | " << left_ref_angle.ankle_pitch << std::endl;
        // std::cout << "Right Angle: " << right_ref_angle.hip_yaw << " | " << right_ref_angle.hip_roll << " | " << right_ref_angle.hip_pitch << " | " << right_ref_angle.knee_pitch << " | " << right_ref_angle.ankle_pitch << std::endl;

        // Set the motor data
        robot.setMotorData(shared_data.motor);

        // Test the foot wrench computation (checked)
        // Wrench right_foot_wrench = dynamics.computeFootWrench(robot.getLegActTorque(RIGHT_LEG_ID), robot.getLegActAngles(RIGHT_LEG_ID), RIGHT_LEG_ID);

        // logDataToCSV(elapsed.count(), left_ref_angle.hip_yaw, left_ref_angle.hip_roll, left_ref_angle.hip_pitch, left_ref_angle.knee_pitch, left_ref_angle.ankle_pitch);
        // logDataToCSV(elapsed.count(),omega[0],omega[1],omega[2]);
        lock.unlock(); // Unlock the shared data
        std::this_thread::sleep_until(next_cycle);
    }
}

void CAN_receive_thread()
{
    auto start = std::chrono::high_resolution_clock::now();
    while (true)
    {
        stm32.receiveData(can); // Receive data from the STM32
        {
            std::lock_guard<std::mutex> lock(shared_data_mutex); // Lock the shared data
#ifdef USE_LITE_PACKAGE
            stm32.decodeDataLite(shared_data.motor, shared_data.imu, shared_data.command); // Decode the received data
#else
            stm32.decodeData(shared_data.motor, shared_data.imu, shared_data.command); // Decode the received data
#endif
            shared_data.new_data = true; // Set the new data flag
            shared_data_cv.notify_one(); // Notify the compute thread that new data is available
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        // std::cout << "Current time: " << elapsed.count() << " seconds" << std::endl;
    }
}

void CAN_send_thread()
{
    auto start = std::chrono::high_resolution_clock::now();
    auto next_cycle = std::chrono::steady_clock::now();
    while (true)
    {
        next_cycle += std::chrono::milliseconds(1);
        {
            std::lock_guard<std::mutex> lock(shared_data_mutex); // Lock the shared data
#ifdef USE_LITE_PACKAGE
            stm32.encodeDataLite(shared_data.motor); // Encode the data to send
#else
            stm32.encodeData(shared_data.motor); // Encode the data to send
#endif
        }
        stm32.sendData(can); // Send the data to the STM32
        std::this_thread::sleep_until(next_cycle);

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        // std::cout << "Current time: " << elapsed.count() << " seconds" << std::endl;
    }
}

void set_thread_realtime(std::thread &thread, int priority)
{
    // Convert std::thread to pthread
    pthread_t native_handle = thread.native_handle();

    // Prepare the scheduling params
    sched_param sch_params;
    sch_params.sched_priority = priority;

    // Attempt to set the policy & priority
    int policy = SCHED_FIFO;
    int ret = pthread_setschedparam(native_handle, policy, &sch_params);
    if (ret != 0)
    {
        std::cerr << "Failed to set RT priority for thread. Errno: " << ret << std::endl;
    }
}

void pinThreadToCore(std::thread &thr, int core_id)
{
    // Prepare a CPU set
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    // Convert std::thread to pthread
    pthread_t native_handle = thr.native_handle();

    // Call pthread_setaffinity_np
    int rc = pthread_setaffinity_np(native_handle, sizeof(cpu_set_t), &cpuset);
    if (rc != 0)
    {
        std::cerr << "Error calling pthread_setaffinity_np: "
                  << strerror(rc) << std::endl;
    }
    else
    {
        std::cout << "Pinned thread to core " << core_id << std::endl;
    }
}