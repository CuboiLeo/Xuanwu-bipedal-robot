#include "walking_patterns.h"

Pose_Two_Foots Walking_Patterns::gaitPlanner(const Position &act_CoM_pos, const Velocity &act_CoM_vel, const Pose_Two_Foots &foot_poses, const double &roll_angle)
{
    duration = std::chrono::high_resolution_clock::now() - start_time;
    gait_phase = duration.count();
    if (gait_phase >= Tsup + Tdbl)
    {
        step_counter++;
        next_step = step_counter % 2 == 0 ? computeLIPM(act_CoM_pos, act_CoM_vel, foot_poses.right.position) : computeLIPM(act_CoM_pos, act_CoM_vel, foot_poses.left.position);
        start_time = std::chrono::high_resolution_clock::now();
        gait_phase = 0;
    }
    std::cout << "Gait Phase: " << gait_phase << std::endl;
    std::cout << "Next Step: " << next_step.x << " | " << next_step.y << std::endl;

    Pose left_ref_pose;
    Pose right_ref_pose;

    if (step_counter > sx.size())
        step_counter = 0;

    if (step_counter % 2 == 0)
    {
        if (gait_phase < Tdbl)
        {
            left_ref_pose = {generateFootTrajectory(left_retracted_pose.position, left_extended_pose.position, gait_phase / Tdbl),left_extended_pose.orientation};
            right_ref_pose = {generateFootTrajectory(right_extended_pose.position, right_retracted_pose.position, gait_phase / Tdbl),right_retracted_pose.orientation};
        }
        else
        {
            left_ref_pose = left_extended_pose;// generateFootTrajectory(left_extend_pose.position, next_step, (gait_phase - Tdbl) / Tsup);
            left_ref_pose.position.z = -0.55 + foot_lift * sin(M_PI * (gait_phase - Tdbl) / Tsup);
            right_ref_pose = right_retracted_pose;
        }
    }
    else
    {
        if (gait_phase < Tdbl)
        {
            left_ref_pose = {generateFootTrajectory(left_extended_pose.position, left_retracted_pose.position, gait_phase / Tdbl),left_retracted_pose.orientation};
            right_ref_pose = {generateFootTrajectory(right_retracted_pose.position, right_extended_pose.position, gait_phase / Tdbl),right_extended_pose.orientation};
        }
        else
        {
            right_ref_pose = right_extended_pose;//generateFootTrajectory(right_extend_pose.position, next_step, (gait_phase - Tdbl) / Tsup);
            right_ref_pose.position.z = -0.55 + foot_lift * sin(M_PI * (gait_phase - Tdbl) / Tsup);
            left_ref_pose = left_retracted_pose;
        }6g
    }

    return {left_ref_pose, right_ref_pose};
}

Position Walking_Patterns::generateFootTrajectory(const Position &initial_pos, const Position &final_pos, const double &phase_percentage)
{
    // Cubic Bezier Curve
    current_pos.x = initial_pos.x + (final_pos.x - initial_pos.x) * (3*pow(phase_percentage, 2) - 2*pow(phase_percentage, 3));
    current_pos.y = initial_pos.y + (final_pos.y - initial_pos.y) * (3*pow(phase_percentage, 2) - 2*pow(phase_percentage, 3));
    current_pos.z = initial_pos.z + (final_pos.z - initial_pos.z) * (3*pow(phase_percentage, 2) - 2*pow(phase_percentage, 3));

    return current_pos;
}

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

    double px = -sx[step_counter] * pow((-1), step_counter);
    double py = sy[step_counter];

    double x_bar = sx[step_counter + 1] / 2 * pow((-1), step_counter);
    double y_bar = sy[step_counter + 1] / 2;
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