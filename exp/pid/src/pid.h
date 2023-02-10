#include "../../../lib/motor_encoder/src/motor.h"

static float deltat = 0.01;
static float kp = 1;
static float ki = 1;
static float kd = 1;

void pid_init(float DELTA_T, float KP, float KI, float KD);
float pid_calc(int16_t result, int16_t target, float error[2], float *integral);