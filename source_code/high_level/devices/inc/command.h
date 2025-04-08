#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>
#include "robot_types.h"

class Command
{
    public:
    Velocity getLinearVel() const { return linear_vel; };
    Velocity getAngularVel() const { return angular_vel; };

    void setLinearVel(const Velocity linear_vel) { this->linear_vel = linear_vel; };
    void setAngularVel(const Velocity angular_vel) { this->angular_vel = angular_vel; };

    private:
    Velocity linear_vel;
    Velocity angular_vel;
};

#endif