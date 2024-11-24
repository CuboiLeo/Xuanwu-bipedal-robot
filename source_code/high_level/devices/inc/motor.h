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

struct motor_params {
    float pos;
    float vel;
    float tor;
};

struct motor_data {
    motor_params ref;
    motor_params act;
};

class Motor
{
public:
    // Setter and getter methods for motor data
    float getRefPos(const uint8_t motor_id) const { return motor[motor_id].ref.pos; };
    float getRefVel(const uint8_t motor_id) const { return motor[motor_id].ref.vel; };
    float getRefTor(const uint8_t motor_id) const { return motor[motor_id].ref.tor; };
    float getActPos(const uint8_t motor_id) const { return motor[motor_id].act.pos; };
    float getActVel(const uint8_t motor_id) const { return motor[motor_id].act.vel; };
    float getActTor(const uint8_t motor_id) const { return motor[motor_id].act.tor; };

    void setRefPos(const uint8_t motor_id, const float pos) { motor[motor_id].ref.pos = pos; };
    void setRefVel(const uint8_t motor_id, const float vel) { motor[motor_id].ref.vel = vel; };
    void setRefTor(const uint8_t motor_id, const float tor) { motor[motor_id].ref.tor = tor; };
    void setActPos(const uint8_t motor_id, const float pos) { motor[motor_id].act.pos = pos; };
    void setActVel(const uint8_t motor_id, const float vel) { motor[motor_id].act.vel = vel; };
    void setActTor(const uint8_t motor_id, const float tor) { motor[motor_id].act.tor = tor; };

private:
    motor_data motor[num_motor];
};

#endif