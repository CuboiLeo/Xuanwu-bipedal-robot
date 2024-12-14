#ifndef ROBOT_H
#define ROBOT_H

#include "robot_types.h"
#include "robot_configs.h"
#include "motor.h"

class Robot
{
public:
    Robot();
    // Setter and getter functions for the leg angles
    void setLegActAngles(const Joint_Angles &left, const Joint_Angles &right) { leg_angles.left.act = left; leg_angles.right.act = right; }
    Joint_Angles getLegActAngles(const uint8_t leg) const { return leg == LEFT_LEG_ID ? leg_angles.left.act : leg_angles.right.act; }
    void setLegRefAngles(const Joint_Angles &left, const Joint_Angles &right) { leg_angles.left.ref = left; leg_angles.right.ref = right; }
    Joint_Angles getLegRefAngles(const uint8_t leg) const { return leg == LEFT_LEG_ID ? leg_angles.left.ref : leg_angles.right.ref; }
    
    // Setter and getter functions for the foot positions
    void setFootActPos(const Direction_Vector &left, const Direction_Vector &right) { foot_pos.left.act = left; foot_pos.right.act = right; }
    Direction_Vector getFootActPos(const uint8_t leg) const { return leg == LEFT_LEG_ID ? foot_pos.left.act : foot_pos.right.act; }
    void setFootRefPos(const Direction_Vector &left, const Direction_Vector &right) { foot_pos.left.ref = left; foot_pos.right.ref = right; }
    Direction_Vector getFootRefPos(const uint8_t leg) const { return leg == LEFT_LEG_ID ? foot_pos.left.ref : foot_pos.right.ref; }
    
    // Setter and getter functions for the CoM positions (in the world frame)
    void setCoMActPos(const Direction_Vector &CoM_pos) { this->CoM_pos.act = CoM_pos; }
    Direction_Vector getCoMActPos() const { return CoM_pos.act; }
    void setCoMRefPos(const Direction_Vector &CoM_pos) { this->CoM_pos.ref = CoM_pos; }
    Direction_Vector getCoMRefPos() const { return CoM_pos.ref; }

    // Setter and getter functions for the ZMP positions (in the world frame)
    void setZMPActPos(const Direction_Vector &ZMP_pos) { this->ZMP_pos.act = ZMP_pos; }
    Direction_Vector getZMPActPos() const { return ZMP_pos.act; }
    void setZMPRefPos(const Direction_Vector &ZMP_pos) { this->ZMP_pos.ref = ZMP_pos; }
    Direction_Vector getZMPRefPos() const { return ZMP_pos.ref; }

    // Setter and getter functions for the CoM accelerations (in the world frame)
    void setCoMActAccel(const Direction_Vector &CoM_accel) { this->CoM_accel.act = CoM_accel; }
    Direction_Vector getCoMActAccel() const { return CoM_accel.act; }
    void setCoMRefAccel(const Direction_Vector &CoM_accel) { this->CoM_accel.ref = CoM_accel; }
    Direction_Vector getCoMRefAccel() const { return CoM_accel.ref; }

    // Set the motor data
    void setMotorData(Motor &motor);

private:
    Leg_Data leg_angles;
    Foot_Data foot_pos;
    Direction_Data CoM_pos;
    Direction_Data CoM_accel;
    Direction_Data ZMP_pos;
};

#endif