/*
 * PIDv10.h
 *
 *  Created on: Apr 19, 2021
 *      Author: dinh quoc
 */

#ifndef SRC_PIDV10_H_
#define SRC_PIDV10_H_
#include <stdint.h>
class PIDv10 {
public:
	PIDv10();
	virtual ~PIDv10();
	PIDv10(float Kp,float Ki,float Kd,float Ts);
	void PID_Compute(float setpoint,float feedback);
	uint16_t PID_GetPID(void);
	void PID_ControlVelocity(uint16_t max,uint16_t min);
	uint16_t PID_GetVelocity(void);
	void PID_Init(void);

private:
	float pre_I_Part=0;
	float I_part=0;
	float D_part=0;
	float pre_Error=0;
	float Error=0;
	float Ki=0;
	float Kp=0;
	float Kd=0;
	float Ts=0;
	float setpoint = 0;
	float feedback=0;
	float Output_Vel=0;
	float PID=0;
};


#endif /* SRC_PIDV10_H_ */
