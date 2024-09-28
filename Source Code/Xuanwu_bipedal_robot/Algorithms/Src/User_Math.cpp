#include "User_Math.h"
#include <cmath>

float WRAP2_2PI(float x)
{
	return std::fmod(x + PI, 2 * PI) - PI;
}