#ifndef COMMAND_H
#define COMMAND_H

#include <stdint.h>
#include "robot_types.h"

class Command
{
    public:
    direction_vector getLinearVel() const { return linear_vel; };
    direction_vector getAngularVel() const { return angular_vel; };

    void setLinearVel(const direction_vector linear_vel) { this->linear_vel = linear_vel; };
    void setAngularVel(const direction_vector angular_vel) { this->angular_vel = angular_vel; };

    private:
    direction_vector linear_vel;
    direction_vector angular_vel;
};

#endif