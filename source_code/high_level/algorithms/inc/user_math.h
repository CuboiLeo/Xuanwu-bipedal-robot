#ifndef USER_MATH_H
#define USER_MATH_H

#include "math.h"

#define PI (3.14159265358979323846f)
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)
#define GRAVITY (9.81f)

int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

#endif