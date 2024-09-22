#include "PID.h"

PID::PID(float kp, float ki, float kd, float kf, float i_out_max, float output_max)
{
	this->kp = kp;
	this->ki = ki;
	this->kd = kd;
	this->kf = kf;
	this->i_out_max = i_out_max;
	this->output_max = output_max;
	ref_val = 0.0f;
	meas_val = 0.0f;
	error = 0.0f;
	prev_error = 0.0f;
	p_out = 0.0f;
	i_out = 0.0f;
	d_out = 0.0f;
	f_out = 0.0f;
	output = 0.0f;
}

float PID::compute(float ref_val, float meas_val) {
    this->ref_val = ref_val;
    this->meas_val = meas_val;

    prev_error = error;
    error = ref_val - meas_val;

    p_out = kp * error;
    i_out += ki * error;
    d_out = kd * (error - prev_error);
    f_out = kf * ref_val;

    i_out = (i_out > i_out_max) ? i_out_max : i_out;
	i_out = (i_out < -i_out_max) ? -i_out_max : i_out;

    output = p_out + i_out + d_out + f_out;

	output = (output > output_max) ? output_max : output;
	output = (output < -output_max) ? -output_max : output;

    return output;
}