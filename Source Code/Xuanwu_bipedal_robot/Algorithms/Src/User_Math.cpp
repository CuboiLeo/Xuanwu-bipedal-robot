#include "User_Math.h"
#include <cmath>

float WRAP2_2PI(float x)
{
	x = std::fmod(x + PI, 2 * PI);
	if (x < 0) 
        x += 2 * PI;  
	return x - PI;
}