/*
 * pid.c
 *
 *  Created on: Jun 19, 2026
 *      Author: Eylül Öztek
 */

#include "pid.h"

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd,
		float sampleTime, float minOutput, float maxOutput) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->sampleTime = sampleTime;
	pid->minOutput = minOutput;
	pid->maxOutput = maxOutput;
	pid->integral = 0.0f;
	pid->prevError = 0.0f;
	pid->output = 0.0f;
}

void PID_SetPoint(PID_Controller_t *pid, float setPoint) {
	pid->setPoint = setPoint;
}

float PID_GetSetPoint(PID_Controller_t *pid) {
	return pid->setPoint;
}
