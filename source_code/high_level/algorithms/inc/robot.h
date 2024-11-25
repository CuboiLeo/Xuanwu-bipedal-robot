#ifndef ROBOT_H
#define ROBOT_H

#include "robot_types.h"
#include "robot_configs.h"
#include "motor.h"

class Robot
{
public:
    // Setter and getter methods for robot data
    void setLegActAngles(const Joint_Angles &left, const Joint_Angles &right) { leg_angles.left.act = left; leg_angles.right.act = right; }
    void setFootActPos(const Direction_Vector &left, const Direction_Vector &right) { foot_pos.left.act = left; foot_pos.right.act = right; }
    void setCoMActPos(const Direction_Vector &CoM) { CoM_pos.act = CoM; }
    void setZMPActPos(const Direction_Vector &ZMP) { ZMP_pos.act = ZMP; }

    Joint_Angles getLegActAngles(const uint8_t leg) const { return leg == LEFT_LEG_ID ? leg_angles.left.act : leg_angles.right.act; }
    Direction_Vector getFootActPos(const uint8_t leg) const { return leg == LEFT_LEG_ID ? foot_pos.left.act : foot_pos.right.act; }
    Direction_Vector getCoMActPos() const { return CoM_pos.act; }
    Direction_Vector getZMPActPos() const { return ZMP_pos.act; }

    void setLegRefAngles(const Joint_Angles &left, const Joint_Angles &right) { leg_angles.left.ref = left; leg_angles.right.ref = right; }
    void setFootRefPos(const Direction_Vector &left, const Direction_Vector &right) { foot_pos.left.ref = left; foot_pos.right.ref = right; }
    void setCoMRefPos(const Direction_Vector &CoM) { CoM_pos.ref = CoM; }
    void setZMPRefPos(const Direction_Vector &ZMP) { ZMP_pos.ref = ZMP; }

    Joint_Angles getLegRefAngles(const uint8_t leg) const { return leg == LEFT_LEG_ID ? leg_angles.left.ref : leg_angles.right.ref; }
    Direction_Vector getFootRefPos(const uint8_t leg) const { return leg == LEFT_LEG_ID ? foot_pos.left.ref : foot_pos.right.ref; }
    Direction_Vector getCoMRefPos() const { return CoM_pos.ref; }
    Direction_Vector getZMPRefPos() const { return ZMP_pos.ref; }

    void setMotorData(Motor &motor);

private:
    Leg_Data leg_angles = {};
    Foot_Data foot_pos = {};
    Direction_Data CoM_pos = {};
    Direction_Data ZMP_pos = {};
};

#endif