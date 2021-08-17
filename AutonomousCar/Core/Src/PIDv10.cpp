/*
 * PIDv10.cpp
 *
 *  Created on: Apr 19, 2021
 *      Author: dinh quoc
 */

#include "PIDv10.h"
#include <stdint.h>
PIDv10::PIDv10() {
	// TODO Auto-generated constructor stub

}


PIDv10::~PIDv10() {
	// TODO Auto-generated destructor stub
}
PIDv10::PIDv10(float Kp,float Ki,float Kd,float Ts) {
	// TODO Auto-generated constructor stub
	this->Kp=Kp;
	this->Ki=Ki;
	this->Kd=Kd;
	this->Ts=Ts;
}
void PIDv10::PID_Init(void)
{
	pre_I_Part=0;
	I_part=0;
	D_part=0;
	pre_Error=0;
	Error=0;
	setpoint = 0;
	feedback=0;
	Output_Vel=0;
	PID=0;
}

void PIDv10::PID_Compute(float setpoint,float feedback)
{
	this->setpoint=setpoint;
	this->feedback=feedback;

    Error = (float)(setpoint-feedback);
    I_part = pre_I_Part + (float)Error*Ts*1.0;
    D_part = (float)(1.0*(Error - pre_Error)/Ts);
    PID = Kp*Error + Ki*I_part + Kd*D_part;
    pre_Error = Error;
    pre_I_Part = I_part;
}
uint16_t PIDv10::PID_GetPID(void)
{
	return (uint16_t)PID;
}
void PIDv10::PID_ControlVelocity(uint16_t max,uint16_t min)
{
	Output_Vel=Output_Vel+PID;
    if (Output_Vel >= max)
    {
    	Output_Vel = max;
    }
    else if (Output_Vel <=min)
    {
    	Output_Vel=min;
    }
}
uint16_t PIDv10::PID_GetVelocity(void)
{
	return (uint16_t)Output_Vel;
}


