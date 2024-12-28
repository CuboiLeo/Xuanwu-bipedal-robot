#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

enum Motor_Name {
    Left_Hip_Yaw,
    Left_Hip_Roll,
    Left_Hip_Pitch,
    Left_Knee_Pitch,
    Left_Foot_Pitch,
    Right_Hip_Yaw,
    Right_Hip_Roll,
    Right_Hip_Pitch,
    Right_Knee_Pitch,
    Right_Foot_Pitch,
    num_motor   // This is the number of motors
};

struct Motor_Params {
    float pos;
    float vel;
    float tor;
};

struct Motor_Data {
    Motor_Params ref;
    Motor_Params act;
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
    Motor_Data motor[num_motor] = {};
};

#endif