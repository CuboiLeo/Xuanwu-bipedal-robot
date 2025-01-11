#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>
#include "robot_types.h"

class Command
{
    public:
    Position getLinearVel() const { return linear_vel; };
    Position getAngularVel() const { return angular_vel; };

    void setLinearVel(const Position linear_vel) { this->linear_vel = linear_vel; };
    void setAngularVel(const Position angular_vel) { this->angular_vel = angular_vel; };

    private:
    Position linear_vel;
    Position angular_vel;
};

#endif