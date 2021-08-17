/*
 * mainUser.cpp
 *
 *  Created on: Apr 19, 2021
 *      Author: dinh quoc
 */
#ifdef __cplusplus
extern "C" {
#endif
#include "PIDv10.h"
#include "FirstOrderSysID.h"
extern float Vel_Feedback_Left;
extern float Vel_Feedback_Right;
extern float Vel_Setpoint_Left;
extern float Vel_Setpoint_Right;
extern uint16_t Vel_pwm_left;
extern uint16_t Vel_pwm_right;
PIDv10 pidLeft(0.1,0,0,0.001);
PIDv10 pidRight(0.1,0,0,0.001);
float zero=0.0;
float pole=0.0;

void user_main(void)
{
 pidLeft.PID_Init();
 pidRight.PID_Init();
 FirstOrderSysID sysid;
 pole=sysid.GetPole();
 zero=sysid.GetZero();


}
void user_loop(void)
{
	pidLeft.PID_Compute(Vel_Setpoint_Left, Vel_Feedback_Left);
	pidRight.PID_Compute(Vel_Setpoint_Right, Vel_Feedback_Right);
	pidLeft.PID_ControlVelocity(999, 0);
	pidRight.PID_ControlVelocity(999, 0);
	Vel_pwm_left= pidLeft.PID_GetVelocity();
	Vel_pwm_right=pidRight.PID_GetVelocity();

}

#ifdef __cplusplus
}
#endif
