#include "utils.h"
#include "math_types.h"
#include <math.h>

uint32_t UTILS_MAX_UI(uint32_t var1, uint32_t var2){
    return var1 > var2 ? var1 : var2;
}
uint32_t UTILS_MIN_UI(uint32_t var1, uint32_t var2){
    return var1 < var2 ? var1 : var2;
}
uint32_t UTILS_MIN_MAX_UI(uint32_t var, uint32_t min, uint32_t max){
	if(var < min){
		return min;
	}
	else if (var > max) {
		return max;
	}
	return var;
}

int32_t UTILS_MAX_I(int32_t var1, int32_t var2){
	return var1 > var2 ? var1 : var2;
}
int32_t UTILS_MIN_I(int32_t var1, int32_t var2){
	return var1 < var2 ? var1 : var2;
}
int32_t UTILS_MIN_MAX_I(int32_t var, int32_t min, int32_t max){
	if(var < min){
		return min;
	}
	else if (var > max) {
		return max;
	}
	return var;
}

float UTILS_MAX_F(float var1, float var2){
    return var1 > var2 ? var1 : var2;
}
float UTILS_MIN_F(float var1, float var2){
    return var1 < var2 ? var1 : var2;
}
float UTILS_MIN_MAX_F(float var, float min, float max){
	if(var < min){
		return min;
	}
	else if (var > max) {
		return max;
	}
	return var;
}

QUAT_T UTILS_QUATERNION_PRODUCT(QUAT_T q1,QUAT_T q2){		// calculates q1*q2
	QUAT_T q_new;
	q_new.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
	q_new.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
	q_new.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
	q_new.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;

	return q_new;
}
QUAT_T UTILS_QUATERNION_NORMALIZE(QUAT_T q){
	float magnitude = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);

	if(magnitude > 1e-8f){
		q.w /= magnitude;
		q.x /= magnitude;
		q.y /= magnitude;
		q.z /= magnitude;
	}

	return q;
}
QUAT_T UTILS_QUATERNION_SCALE(QUAT_T q, float scalar){
	q.w *= scalar;
	q.x *= scalar;
	q.y *= scalar;
	q.z *= scalar;

	return q;
}
QUAT_T UTILS_QUATERNION_ADD(QUAT_T q1, QUAT_T q2){
	q1.w += q2.w;
	q1.x += q2.x;
	q1.y += q2.y;
	q1.z += q2.z;

	return q1;
}
QUAT_T UTILS_QUATERNION_CONJUGATE(QUAT_T q){
	q.x = -q.x;
	q.y = -q.y;
	q.z = -q.z;

	return q;
}

VECT_3D_T UTILS_VECT_CROSS_PRODUCT(VECT_3D_T a, VECT_3D_T b){
	VECT_3D_T cross_product = {0};

	cross_product.x = a.y*b.z - a.z*b.y;
	cross_product.y = a.z*b.x - a.x*b.z;
	cross_product.z = a.x*b.y - a.y*b.x;

	return cross_product;
}
VECT_3D_T UTILS_VECT_NORMALIZE(VECT_3D_T v){
	float magnitude = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);

	if(magnitude > 1e-8f){
		v.x /= magnitude;
		v.y /= magnitude;
		v.z /= magnitude;
	}

	return v;
}
VECT_3D_T UTILS_VECT_SCALE(VECT_3D_T v, float scalar){
	v.x *= scalar;
	v.y *= scalar;
	v.z *= scalar;

	return v;
}
VECT_3D_T UTILS_VECT_ADD(VECT_3D_T v1, VECT_3D_T v2){
	v1.x += v2.x;
	v1.y += v2.y;
	v1.z += v2.z;

	return v1;
}

float UTILS_RADIANS(float degrees){
	return degrees * ((float)M_PI / 180.0f);
}

float UTILS_DEGREES(float radians){
	return radians * (180.0f / (float)M_PI);
}
