#ifndef ESTIMATIONS_H
#define ESTIMATIONS_H

#include <stdint.h>
#include "robot_types.h"
#include "robot_configs.h"
#include "user_math.h"
#include <chrono>
#include <algorithm>

class Estimations
{
    public:
    Estimations();
    void estimateCoMStates(const Velocity measured_CoM_vel, const Acceleration measured_CoM_accel);
    Position getEstimatedCoMPos(void) const { return estimated_CoM_pos; }
    Velocity getEstimatedCoMVel(void) const { return estimated_CoM_vel; }


    private:
    // Estimated states
    Position estimated_CoM_pos;
    Velocity estimated_CoM_vel;

    double getTimeElapsed(void);
    std::chrono::high_resolution_clock::time_point last_time;    // Time point of the last estimation update
    std::chrono::high_resolution_clock::time_point current_time; // Time point of the current estimation update
    double delta_t;                                              // Time elapsed since the last estimation update

    // Kalman filter parameters
    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
    Eigen::Matrix<double, 4, 2> B = Eigen::Matrix<double, 4, 2>::Zero();
    Eigen::Matrix4d C;
    Eigen::Vector4d x = Eigen::Vector4d::Zero();
    Eigen::Vector2d u = Eigen::Vector2d::Zero();
    Eigen::Vector4d y = Eigen::Vector4d::Zero();
    Eigen::Vector4d x_hat = Eigen::Vector4d::Zero();
    Eigen::Matrix4d P = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d K = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d Q;
    Eigen::Matrix4d R;
};

#endif // ESTIMATIONS_H
