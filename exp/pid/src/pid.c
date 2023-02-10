#include "pid.h"

void pid_init(float DELTA_T, float KP, float KI, float KD) {
    deltat = DELTA_T;
    kp = KP;
    ki = KI;
    kd = KD;
}

float pid_calc(int16_t result, int16_t target, float error[2], float *integral) {
    float p, i, d;

    error[0] = error[1];
    error[1] = result - target;
    *integral += (((float)(error[0] + error[1]) / 2.0) * deltat);

    p = kp * error[1];
    i = ki * (*integral);
    d = kd * ((error[1] - error[0]) / deltat);

    return (p + i + d);
}