#ifndef WALKING_PATTERNS_H
#define WALKING_PATTERNS_H

#include <stdint.h>
#include "robot_types.h"
#include "robot_configs.h"
#include "user_math.h"
#include <math.h>
#include <chrono>

class Walking_Patterns
{
public:
    Walking_Patterns() { start_time = std::chrono::high_resolution_clock::now(); };
    Pose_Two_Foots gaitPlanner(const Position &act_CoM_pos, const Velocity &act_CoM_vel, const Pose_Two_Foots &foot_poses, const double &roll_angle);
    double getGaitPhase() { return gait_phase; };
    double getStepCounter() { return step_counter; };
    static constexpr double Tsup = 0.8;              // Single support phase time
    static constexpr double Tdbl = 0.2;              // Double support phase time

private:
    Position computeLIPM(const Position &act_CoM_pos, const Velocity &act_CoM_vel, const Position &stance_foot_pos);
    Position generateFootTrajectory(const Position &initial_pos, const Position &final_pos, const double &phase_percentage);

    Position next_step;
    Position current_pos;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    std::chrono::duration<double> duration;

    double foot_lift = 0.03;                                 // Foot lift height (m)
    int step_counter = 0;                                    // Odd means right foot next, even means left foot next
    double gait_phase = 0;                                   // Step phase
    Position left_retract_pos = {-0.1, -0.01, -0.51};     // Left foot retract position
    Position right_retract_pos = {0.1, -0.01, -0.51};     // Right foot retract position
    Position left_extend_pos = {-0.135, -0.01, -0.54};      // Left foot extend position
    Position right_extend_pos = {0.135, -0.01, -0.54};      // Right foot extend position
    Position left_swing_pos = {-0.135, 0.06, -0.54};         // Left foot swing position
    Position right_swing_pos = {0.135, 0.06, -0.54};         // Right foot swing position

    std::vector<double> sx = {0.2, 0.2, 0.2, 0.2, 0.2};      // nominal step offset in x (m)
    std::vector<double> sy = {0.27, 0.27, 0.27, 0.27, 0.27}; // nominal step offset in y (m)
    double x_local = 0;
    double y_local = 0;
    double xdot_local = 0;
    double ydot_local = 0;
    double x_free = 0;
    double y_free = 0;
    double xdot_free = 0;
    double ydot_free = 0;
    double px_star = 0;
    double py_star = 0;

    static constexpr double zc = 0.4;                // CoM height
    static constexpr double Tc = sqrt(zc / GRAVITY); // Time constant
    static constexpr double C = cosh(Tsup / Tc);
    static constexpr double S = sinh(Tsup / Tc);
    static constexpr double a = 10;                                              // Weighting factor of the evaluation function
    static constexpr double b = 1;                                               // Weighting factor of the evaluation function
    static constexpr double D = a * (C - 1) * (C - 1) + b * (S / Tc) * (S / Tc); // Evaluation function
};

#endif