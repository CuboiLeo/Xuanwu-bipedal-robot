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
    Walking_Patterns(){start_time = std::chrono::high_resolution_clock::now();};
    Position gaitPlanner(const Position &act_CoM_pos, const Velocity &act_CoM_vel, const Position &stance_foot_pos);
    double getStepCounter() { return step_counter; };

private:
    Position computeLIPM(const Position &act_CoM_pos, const Velocity &act_CoM_vel, const Position &stance_foot_pos);
    Position generateFootTrajectory(const Position &next_step, const double &time);

    Position next_step;
    Position current_pos;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    std::chrono::duration<double> duration;

    double foot_lift = 0.04;                                 // Foot lift height
    int step_counter = 0;                                 // Odd means right foot next, even means left foot next
    std::vector<double> sx = {0.2, 0.2, 0.2, 0.2, 0.2};          // nominal step offset in x (m)
    std::vector<double> sy = {0.28, 0.28, 0.28, 0.28, 0.28}; // nominal step offset in y (m)
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
    static constexpr double Tsup = 0.8;              // Single support phase time
    static constexpr double C = cosh(Tsup / Tc);
    static constexpr double S = sinh(Tsup / Tc);
    static constexpr double a = 10;                                              // Weighting factor of the evaluation function
    static constexpr double b = 1;                                               // Weighting factor of the evaluation function
    static constexpr double D = a * (C - 1) * (C - 1) + b * (S / Tc) * (S / Tc); // Evaluation function
};

#endif