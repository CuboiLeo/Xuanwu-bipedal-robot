#include "PID.h"

float PID(PID_t *PID, float ref_val, float meas_val)
{
	PID->ref_val = ref_val;
	PID->meas_val = meas_val;

	PID->prev_error = PID->error;
	PID->error = PID->ref_val - PID->meas_val;
	
	PID->p_out = PID->kp * PID->error;
	PID->i_out += PID->ki * PID->error;
	PID->d_out = PID->kd * (PID->error - PID->prev_error);
	PID->f_out = PID->kf * PID->ref_val;
	
	PID->i_out = PID->i_out > PID->i_out_max ? PID->i_out_max : PID->i_out;
	PID->i_out = PID->i_out < -PID->i_out_max ? -PID->i_out_max : PID->i_out;
	
	PID->output = PID->p_out + PID->i_out + PID->d_out + PID->f_out;

	PID->output = PID->output > PID->output_max ? PID->output_max : PID->output;
	PID->output = PID->output < -PID->output_max ? -PID->output_max : PID->output;
	
	return PID->output;
}
