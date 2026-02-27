#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <time.h>

void quaternion_product(const float* q1,const float* q2, float* q3){		// calculates q1*q2, saves value in q3
	float q_new[4];
	q_new[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
	q_new[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
	q_new[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
	q_new[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];

	q3[0] = q_new[0]; q3[1] = q_new[1]; q3[2] = q_new[2]; q3[3] = q_new[3];
}

void print_float(const float* q){
	printf("(%f, %fi, %fj, %fk)\n", q[0], q[1], q[2], q[3]);
}

void print_vector(const char* text, const float* v){
	printf(text);
	printf("(%f, %f, %f)\n", v[0], v[1], v[2]);
}

float radians(float degrees){
	return degrees * (M_PI / 180.0f);
}

float degrees(float radians){
	return radians * (180.0f / M_PI);
}

int main(char argv[], int argc){
    float q[4] = {1, 0, 0, 0};
	float errI[3] = {0, 0, 0};
	float errP[3] = {0, 0, 0};
	float kI = 0.01;
	float kP = 0.05;
	printf("Start quaternion");
	print_float(q);
	float dt = 1;
	float wx = radians(15);
	float wy = radians(0);
	float wz = radians(0);
	printf("Gyro rotation vector: \n\n%.2f°/s\n%.2f°/s\n%.2f°/s\n\n over %.2f seconds.\n\n", degrees(wx), degrees(wy), degrees(wz), dt);
	float theta = sqrtf(wx*wx + wy*wy + wz*wz) * dt;
	printf("Theta is: %f\n", theta);
	float norm = sqrtf(wx*wx + wy*wy + wz*wz);
	wx /= norm; wy /= norm; wz /= norm;

	printf("normalized magnitude: %f\n", sqrtf(wx*wx + wy*wy + wz*wz));
    float q_delta[4] = {cosf(theta / 2), wx * sinf(theta / 2), wy * sinf(theta / 2), wz * sinf(theta / 2)};

	printf("Delta quaternion");
	print_float(q_delta);

	quaternion_product(q_delta, q, q);

	for(int i = 0; i < 50; i++){
		wx = 0; wy = 0; wz = 0;

		printf("rotated quaternion:");
		print_float(q);

		float vG[3] = {2 * (q[1]*q[3] - q[0]*q[2]), 2 * (q[0]*q[1] + q[2]*q[3]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]};
		float aG[3] = {0, sinf(radians(13)), cosf(radians(13))};
		float errG[3] = {aG[1]*vG[2] - aG[2]*vG[1], aG[1]*vG[0] - aG[0]*vG[2], aG[0]*vG[1] - aG[1]*vG[0]};
		errI[0] += errG[0] * kI;
		errI[1] += errG[1] * kI;
		errI[2] += errG[2] * kI;

		errP[0] = errG[0] * kP;
		errP[1] = errG[1] * kP;
		errP[2] = errG[2] * kP;

		print_vector("\nGravity extracted from Quaternion: ", vG);
		print_vector("Gravity vector from accelerometer: ", aG);
		print_vector("Error extracted from cross product: ", errG);

		print_vector("\nError I: ", errI);
		print_vector("Error P: ", errP);

		printf("Gyro rotation vector: \n\n%.2f°/s\n%.2f°/s\n%.2f°/s\n\n over %.2f seconds.\n\n", degrees(wx), degrees(wy), degrees(wz), dt);

		wx += errP[0] + errI[0];
		wy += errP[1] + errI[1];
		wz += errP[2] + errI[2];


		printf("Corrected gyro rotation vector: \n\n%.2f°/s\n%.2f°/s\n%.2f°/s\n\n over %.2f seconds.\n\n", degrees(wx), degrees(wy), degrees(wz), dt);
		theta = sqrtf(wx*wx + wy*wy + wz*wz) * dt;
		printf("Theta is: %f\n", theta);
		norm = sqrtf(wx*wx + wy*wy + wz*wz);
		wx /= norm; wy /= norm; wz /= norm;

		printf("normalized magnitude: %f\n", sqrtf(wx*wx + wy*wy + wz*wz));
		q_delta[0] = cosf(theta / 2);
		q_delta[1] = wx * sinf(theta / 2);
		q_delta[2] = wy * sinf(theta / 2);
		q_delta[3] = wz * sinf(theta / 2);

		printf("Delta quaternion");
		print_float(q_delta);

		quaternion_product(q_delta, q, q);

		wx = 0; wy = 0; wz = 0;

		printf("rotated quaternion:");
		print_float(q);
	}

    return 0;
}