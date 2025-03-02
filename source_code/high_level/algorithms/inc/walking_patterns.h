#ifndef WALKING_PATTERNS_H
#define WALKING_PATTERNS_H

#include <stdint.h>
#include "robot_types.h"
#include "robot_configs.h"
#include "user_math.h"
#include <math.h>

class Walking_Patterns
{
public:
    Position computeLIPM(const Position &act_CoM_pos, const Velocity &act_CoM_vel);

private:
    double step_counter = 0; // Odd means left foot next, even means right foot next
    Eigen::VectorXd sx {{0, 0.3, 0.3, 0.3, 0}};
    Eigen::VectorXd sy {{0.2, 0.2, 0.2, 0.2, 0.2}};

    static constexpr double zc = 0.4;                // CoM height
    static constexpr double Tc = sqrt(zc / GRAVITY); // Time constant
    static constexpr double Tsup = 0.8;              // Support phase time
    static constexpr double C = cosh(Tsup / Tc);
    static constexpr double S = sinh(Tsup / Tc);
    static constexpr double a = 10; // Weighting factor of the evaluation function
    static constexpr double b = 1;  // Weighting factor of the evaluation function
    static constexpr double D = a * (C - 1) * (C - 1) + b * (S / Tc) * (S / Tc); // Evaluation function
};

#endif