#include <stdint.h>
#include "math_types.h"

#ifndef UTILS_H_
#define UTILS_H_

uint32_t UTILS_MAX_UI(uint32_t var1, uint32_t var2); // returns the highest number of the two
uint32_t UTILS_MIN_UI(uint32_t var1, uint32_t var2); // returns the lowest number of the two
uint32_t UTILS_MIN_MAX_UI(uint32_t var, uint32_t min, uint32_t max);    // returns value clamped between max and min

int32_t UTILS_MAX_I(int32_t var1, int32_t var2); // returns the highest number of the two
int32_t UTILS_MIN_I(int32_t var1, int32_t var2); // returns the lowest number of the two
int32_t UTILS_MIN_MAX_I(int32_t var, int32_t min, int32_t max); // returns value clamped between max and min

float UTILS_MAX_F(float var1, float var2);  // returns the highest number of the two
float UTILS_MIN_F(float var1, float var2);  // returns the lowest number of the two
float UTILS_MIN_MAX_F(float var, float min, float max); // returns value clamped between max and min

QUAT_T UTILS_QUATERNION_PRODUCT(QUAT_T q1,QUAT_T q2);   // stores the dot product of two quaternions to a third
QUAT_T UTILS_QUATERNION_NORMALIZE(QUAT_T q);
QUAT_T UTILS_QUATERNION_SCALE(QUAT_T q, float scalar);
QUAT_T UTILS_QUATERNION_ADD(QUAT_T q1, QUAT_T q2);
QUAT_T UTILS_QUATERNION_CONJUGATE(QUAT_T q);

VECT_3D_T UTILS_VECT_CROSS_PRODUCT(VECT_3D_T a, VECT_3D_T b);
VECT_3D_T UTILS_VECT_NORMALIZE(VECT_3D_T v);
VECT_3D_T UTILS_VECT_SCALE(VECT_3D_T v, float scalar);
VECT_3D_T UTILS_VECT_ADD(VECT_3D_T v1, VECT_3D_T v2);

float UTILS_RADIANS(float degrees); // converts degrees in radians
float UTILS_DEGREES(float radians); // converts radians in degrees

#endif