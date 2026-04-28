#include <stdint.h>

#ifndef UTILS_H_
#define UTILS_H_

uint32_t UTILS_MAX_UI(uint32_t var1, uint32_t var2); // returns the highest number of the two
uint32_t UTILS_MIN_UI(uint32_t var1, uint32_t var2); // returns the lowest number of the two
int32_t UTILS_MAX_I(int32_t var1, int32_t var2); // returns the highest number of the two
int32_t UTILS_MIN_I(int32_t var1, int32_t var2); // returns the lowest number of the two
float UTILS_MAX_F(float var1, float var2);  // returns the highest number of the two
float UTILS_MIN_F(float var1, float var2);  // returns the lowest number of the two
void UTILS_QUATERNION_PRODUCT(const float* q1,const float* q2, float* q3);   // stores the dot product of two quaternions to a third
float UTILS_RADIANS(float degrees); // converts degrees in radians
float UTILS_DEGREES(float radians); // converts radians in degrees

#endif