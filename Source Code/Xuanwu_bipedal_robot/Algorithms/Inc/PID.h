#ifndef PID_H
#define PID_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif
typedef struct PID
{
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
}PID_t;

float PID(PID_t *PID, float ref_val, float meas_val);

#ifdef __cplusplus
}
#endif

#endif
