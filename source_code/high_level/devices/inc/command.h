#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>
#include "robot_types.h"

class Command
{
    public:
    Direction_Vector getLinearVel() const { return linear_vel; };
    Direction_Vector getAngularVel() const { return angular_vel; };

    void setLinearVel(const Direction_Vector linear_vel) { this->linear_vel = linear_vel; };
    void setAngularVel(const Direction_Vector angular_vel) { this->angular_vel = angular_vel; };

    private:
    Direction_Vector linear_vel;
    Direction_Vector angular_vel;
};

#endif