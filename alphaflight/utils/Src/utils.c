#include "utils.h"
#include <math.h>

uint32_t UTILS_MAX_I(uint32_t var1, uint32_t var2){
    return var1 > var2 ? var1 : var2;
}

uint32_t UTILS_MIN_I(uint32_t var1, uint32_t var2){
    return var1 < var2 ? var1 : var2;
}

float UTILS_MAX_F(float var1, float var2){
    return var1 > var2 ? var1 : var2;
}

float UTILS_MIN_F(float var1, float var2){
    return var1 < var2 ? var1 : var2;
}

void UTILS_QUATERNION_PRODUCT(const float* q1,const float* q2, float* q3){		// calculates q1*q2, saves value in q3
	float q_new[4];
	q_new[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
	q_new[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
	q_new[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
	q_new[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];

	q3[0] = q_new[0]; q3[1] = q_new[1]; q3[2] = q_new[2]; q3[3] = q_new[3];
}

float UTILS_RADIANS(float degrees){
	return degrees * ((float)M_PI / 180.0f);
}

float UTILS_DEGREES(float radians){
	return radians * (180.0f / (float)M_PI);
}
