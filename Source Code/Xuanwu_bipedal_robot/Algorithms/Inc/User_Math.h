#ifndef USER_MATH_H
#define USER_MATH_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "math.h"

#define PI (3.14159265358979323846f)
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)
#define GRAVITY (9.81f)

    extern float WRAP2_2PI(float x);

#ifdef __cplusplus
}
#endif

#endif
