#include "walking_patterns.h"

Position Walking_Patterns::computeLIPM(const Position &act_CoM_pos, const Velocity &act_CoM_vel, const Position &stance_foot_pos)
{
    x_local = act_CoM_pos.x - stance_foot_pos.x;
    y_local = act_CoM_pos.y - stance_foot_pos.y;
    xdot_local = act_CoM_vel.x;
    ydot_local = act_CoM_vel.y;

    x_free = C * x_local + Tc * S * xdot_local;
    y_free = C * y_local + Tc * S * ydot_local;
    xdot_free = S / Tc * x_local + C * xdot_local;
    ydot_free = S / Tc * y_local + C * ydot_local;

    double px = -sx(step_counter) * (-1) ^ step_counter;
    double py = sy(step_counter);

    double x_bar = sx(step_counter + 1) / 2 * (-1) ^ step_counter;
    double y_bar = sy(step_counter + 1) / 2 ;
    double vx_bar = (C - 1) / (Tc * S) * x_bar;
    double vy_bar = (C + 1) / (Tc * S) * y_bar;

    double xd = px + x_bar;
    double yd = py + y_bar;
    double xd_dot = vx_bar;
    double yd_dot = vy_bar;

    px_star = -a * (C - 1) / D * (xd - C * x_local - Tc * S * xdot_local) - b * (S / Tc) / D * (xd_dot - (S / Tc) * x_local - C * xdot_local);
    py_star = -a * (C - 1) / D * (yd - C * y_local - Tc * S * ydot_local) - b * (S / Tc) / D * (yd_dot - (S / Tc) * y_local - C * ydot_local);

    return {px_star, py_star, 0};
}