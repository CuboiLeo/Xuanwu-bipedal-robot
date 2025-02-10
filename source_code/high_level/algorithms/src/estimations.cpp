#include "estimations.h"

Estimations::Estimations()
{
    // Initialize the time points
    last_time = std::chrono::high_resolution_clock::now();
    current_time = std::chrono::high_resolution_clock::now();

    // Initialize the Kalman filter parameters
    C << 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    Q << 0.01, 0, 0, 0,
        0, 0.01, 0, 0,
        0, 0, 0.01, 0,
        0, 0, 0, 0.01;
    R << 0.01, 0, 0, 0.01,
        0, 0.01, 0, 0,
        0, 0, 0.01, 0,
        0, 0, 0, 0.01;
}

double Estimations::getTimeElapsed()
{
    // Get the time elapsed since the last IMU update
    current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> time_elapsed = current_time - last_time;

    last_time = current_time;

    // Convert the time elapsed to seconds
    return time_elapsed.count();
}

void Estimations::estimateCoMStates(const Velocity measured_CoM_vel, const Acceleration measured_CoM_accel)
{
    // Get the time elapsed since the last estimation update
    delta_t = getTimeElapsed();
    delta_t = std::min(delta_t, 0.01); // Ensure that delta_t is no bigger than a threashold when program is paused

    // Kalman filter parameters
    A << 1, 0, delta_t, 0,
        0, 1, 0, delta_t,
        0, 0, 1, 0,
        0, 0, 0, 1;
    B << 0.5 * delta_t * delta_t, 0,
        0, 0.5 * delta_t * delta_t,
        delta_t, 0,
        0, delta_t;
    u << measured_CoM_accel.x, measured_CoM_accel.y;
    y << 0, 0, measured_CoM_vel.x, measured_CoM_vel.y;
    x_hat = A * x + B * u;
    P = A * P * A.transpose() + R;
    K = P * C.transpose() * (C * P * C.transpose() + Q).inverse();
    x = x_hat + K * (y - C * x_hat);
    P = (Eigen::Matrix4d::Identity() - K * C) * P;

    // Set the estimated states
    estimated_CoM_pos = {x(0), x(1), 0};
    estimated_CoM_vel = {x(2), x(3), 0};
    return ;
}