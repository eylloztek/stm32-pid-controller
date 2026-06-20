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

float PID_Compute(PID_Controller_t *pid, float actualValue) {

	float error = pid->setPoint - actualValue;

	float propotional = pid->Kp * error;

	pid->integral += error * pid->sampleTime;

	if (pid->integral > pid->maxOutput) {
		pid->integral = pid->maxOutput;
	} else if (pid->integral < pid->minOutput) {
		pid->integral = pid->minOutput;
	}

	float integral = pid->Ki * pid->integral;

	float derivative = pid->Kd * (error - pid->prevError) / pid->sampleTime;
	pid->prevError = error;

	pid->output = propotional + integral + derivative;

	if (pid->output > pid->maxOutput) {
		pid->output = pid->maxOutput;
	} else if (pid->output < pid->minOutput) {
		pid->output = pid->minOutput;
	}

	return pid->output;
}

void PID_Reset(PID_Controller_t *pid) {

	pid->integral = 0.0f;
	pid->prevError = 0.0f;
}
