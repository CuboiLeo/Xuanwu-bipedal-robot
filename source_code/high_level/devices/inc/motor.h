#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

enum motor_name {
    Left_Hip_Yaw,
    Left_Hip_Roll,
    Left_Hip_Pitch,
    Left_Knee_Pitch,
    Right_Hip_Yaw,
    Right_Hip_Roll,
    Right_Hip_Pitch,
    Right_Knee_Pitch,
    num_motor   // This is the number of motors
};

struct motor_data {
    float pos;
    float vel;
    float tor;
};

class Motor
{
public:
    float getPos(const uint8_t motor_id) const { return motor[motor_id].pos; };
    float getVel(const uint8_t motor_id) const { return motor[motor_id].vel; };
    float getTor(const uint8_t motor_id) const { return motor[motor_id].tor; };

    void setPos(const uint8_t motor_id, const float pos) { motor[motor_id].pos = pos; };
    void setVel(const uint8_t motor_id, const float vel) { motor[motor_id].vel = vel; };
    void setTor(const uint8_t motor_id, const float tor) { motor[motor_id].tor = tor; };

private:
    motor_data motor[num_motor];
};

#endif