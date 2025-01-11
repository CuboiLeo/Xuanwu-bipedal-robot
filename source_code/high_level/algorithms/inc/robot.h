#ifndef ROBOT_H
#define ROBOT_H

#include "robot_types.h"
#include "robot_configs.h"
#include "motor.h"

class Robot
{
public:
    // Setter and getter functions for the leg angles
    void setLegActAngles(const Joint_Angles &left, const Joint_Angles &right) { leg_angles.left.act = left; leg_angles.right.act = right; }
    Joint_Angles getLegActAngles(const uint8_t leg) const { return leg == LEFT_LEG_ID ? leg_angles.left.act : leg_angles.right.act; }
    void setLegRefAngles(const Joint_Angles &left, const Joint_Angles &right) { leg_angles.left.ref = left; leg_angles.right.ref = right; }
    Joint_Angles getLegRefAngles(const uint8_t leg) const { return leg == LEFT_LEG_ID ? leg_angles.left.ref : leg_angles.right.ref; }
    
    // Setter and getter functions for the foot positions
    void setFootActPose(const Pose &left, const Pose &right) { foot_poses.left.act = left; foot_poses.right.act = right; }
    Pose getFootActPose(const uint8_t leg) const { return leg == LEFT_LEG_ID ? foot_poses.left.act : foot_poses.right.act; }
    void setFootRefPose(const Pose &left, const Pose &right) { foot_poses.left.ref = left; foot_poses.right.ref = right; }
    Pose getFootRefPose(const uint8_t leg) const { return leg == LEFT_LEG_ID ? foot_poses.left.ref : foot_poses.right.ref; }
    
    // Setter and getter functions for the CoM positions (in the world frame)
    void setCoMActPos(const Position &CoM_pos) { this->CoM_pos.act = CoM_pos; }
    Position getCoMActPos() const { return CoM_pos.act; }
    void setCoMRefPos(const Position &CoM_pos) { this->CoM_pos.ref = CoM_pos; }
    Position getCoMRefPos() const { return CoM_pos.ref; }

    // Setter and getter functions for the ZMP positions (in the world frame)
    void setZMPActPos(const Position &ZMP_pos) { this->ZMP_pos.act = ZMP_pos; }
    Position getZMPActPos() const { return ZMP_pos.act; }
    void setZMPRefPos(const Position &ZMP_pos) { this->ZMP_pos.ref = ZMP_pos; }
    Position getZMPRefPos() const { return ZMP_pos.ref; }

    // Setter and getter functions for the CoM accelerations (in the world frame)
    void setCoMActAccel(const Position &CoM_accel) { this->CoM_accel.act = CoM_accel; }
    Position getCoMActAccel() const { return CoM_accel.act; }
    void setCoMRefAccel(const Position &CoM_accel) { this->CoM_accel.ref = CoM_accel; }
    Position getCoMRefAccel() const { return CoM_accel.ref; }

    // Set the motor data
    void setMotorData(Motor &motor);

private:
    Leg_Data leg_angles = {};
    Foot_Data foot_poses = {};
    Position_Data CoM_pos = {};
    Position_Data CoM_accel = {};
    Position_Data ZMP_pos = {};
};

#endif