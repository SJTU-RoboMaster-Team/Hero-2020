/**
  ******************************************************************************
  * File Name          : CANMotor.h
  * Description        : CAN线电机控制
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __CANMOTOR_H
#define __CANMOTOR_H

#include "includes.h"

#define GM_PITCH_GRAVITY_COMPENSATION -6800

#ifdef HERO
#define GM_PITCH_ZERO 	15228
#define GM_YAW_ZERO 		-6110
#endif

#define NORMALIZE_ANGLE180(angle) angle = ((angle) > 180) ? ((angle) - 360) : (((angle) < -180) ? (angle) + 360 : angle)

#define Normal_MOTORINFO_Init(rdc,func,ppid,spid)\
{\
	ESC_C6x0,0,0,0,rdc,\
	{0,0,0},{0,0,0},0,0,1,0,0,func,\
	ppid,spid,CHASSIS_MOTOR_SPEED_PID_DEFAULT,0 \
}

#define Chassis_MOTORINFO_Init(func,spid)\
{\
	ESC_C6x0,0,0,0,1,\
	{0,0,0},{0,0,0},0,0,1,0,0,func,\
	FW_PID_DEFAULT,FW_PID_DEFAULT,spid,0 \
}

#define FW_PID_DEFAULT \
{ \
	0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,\
	0, 0, 0, 0.0, 0.0, 0.0, \
	0, 0, 0, 0.0, 0, \
	{0.0}, \
	&fw_PID_Calc, &fw_PID_Reset \
}

typedef enum
{
	ESC_C6x0=0,
	ESC_6623=1
}ESCtype_e;

typedef struct MotorINFO
{
	ESCtype_e			ESCtype;
	CAN_HandleTypeDef* 	CAN_TYPE;
	uint16_t 			TXID;
	uint16_t			RXID;
	float 				ReductionRate;
	ESCC6x0RxMsg_t		RxMsgC6x0;
	ESC6623RxMsg_t		RxMsg6623;
	double 				TargetAngle;
	uint8_t				s_count;
	uint8_t 			FirstEnter;
	double 				lastRead;
	double 				RealAngle;
	void (*Handle)(struct MotorINFO* id);
	fw_PID_Regulator_t 	positionPID;
	fw_PID_Regulator_t 	speedPID;
	PID_Regulator_t		offical_speedPID;
	int16_t				Intensity;
}MotorINFO;

extern MotorINFO CMFL,CMFR,CMBL,CMBR,GMY,GMP,FRICL,FRICR,STIR,GATE,CLIMB_F,CLIMB_B,FRICAL,FRICAR,FRICBL,FRICBR,GMY_FINE_ADJUSTMENT;
extern MotorINFO *can1[8],*can2[8];
extern uint8_t GMYReseted,GMPReseted;
extern uint8_t GM_Turn_back;
extern uint8_t ChassisTwistState;
void InitMotor(MotorINFO *id);
void Motor_ID_Setting(void);
void setCAN11(void);
void setCAN12(void);
void setCAN21(void);
void setCAN22(void);

#endif /*__ CANMOTOR_H */
