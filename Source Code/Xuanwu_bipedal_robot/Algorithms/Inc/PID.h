#ifndef PID_H
#define PID_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif
class PID {
public:
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, float kf = 0.0f, float i_out_max = 0.0f, float output_max = 0.0f);

    float compute(float ref_val, float meas_val);

    // Setters for PID parameters
    void setKp(float kp) { this->kp = kp; }
    void setKi(float ki) { this->ki = ki; }
    void setKd(float kd) { this->kd = kd; }
    void setKf(float kf) { this->kf = kf; }
    void setIOutMax(float max) { this->i_out_max = max; }
    void setOutputMax(float max) { this->output_max = max; }

private:
    float ref_val;
    float meas_val;
    float error;
    float prev_error;

    float kp;
    float ki;
    float kd;
    float kf;

    float p_out;
    float i_out;
    float d_out;
    float f_out;

    float i_out_max;
    float output_max;
    float output;
};

#ifdef __cplusplus
}
#endif

#endif
