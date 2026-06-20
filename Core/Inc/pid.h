/*
 * pid.h
 *
 *  Created on: Jun 19, 2026
 *      Author: Eylül Öztek
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "main.h"

typedef struct{
	float Kp; // propotional gain
	float Ki; //integral gain
	float Kd; //derivative gain
	float setPoint; //reference value
	float integral; //integral sum
	float prevError; //previous error
	float output; //process variable
	float minOutput; // output max limit
	float maxOutput; //output min limit
	float sampleTime; //sampling period (second)
}PID_Controller_t;

void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd,float sampleTime, float minOutput, float maxOutput);
void PID_SetPoint(PID_Controller_t *pid, float setPoint);
float PID_GetSetPoint(PID_Controller_t *pid);
float PID_Compute(PID_Controller_t *pid, float actualValue);
void PID_Reset(PID_Controller_t *pid);

#endif /* INC_PID_H_ */
