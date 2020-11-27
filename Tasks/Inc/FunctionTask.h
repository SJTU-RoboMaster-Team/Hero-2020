/**
  ******************************************************************************
  * File Name          : FunctionTask.h
  * Description        : 用于记录机器人独有的功能
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __FUNCTIONTASK_H
#define __FUNCTIONTASK_H

#include "includes.h"

//键鼠常量区
#define KEY_W				0x1
#define KEY_S				0x2
#define KEY_A				0x4
#define KEY_D				0x8
#define KEY_SHIFT		0x10
#define KEY_CTRL		0x20
#define KEY_Q				0x40
#define KEY_E				0x80
#define KEY_R				0x100
#define KEY_F				0x200
#define KEY_G				0x400
#define KEY_Z				0x800
#define KEY_X				0x1000
#define KEY_C				0x2000
#define KEY_V				0x4000
#define KEY_B				0x8000

#define MAXHEAT01 	180
#define MAXHEAT02 	240
#define MAXHEAT03 	200
#define COOLDOWN01 	30.0f
#define COOLDOWN02 	40.0f
#define COOLDOWN03	50.0f

#define MAXHEAT11 	200
#define MAXHEAT12 	300
#define MAXHEAT13 	20000//200  300  400  等级
#define COOLDOWN11 	20.0f
#define COOLDOWN12 	40.0f
#define COOLDOWN13  1000.00f //20 40 60 等级

#define OnePush(button,execution)\
{\
	static uint8_t cache;\
	static uint8_t cnt=0;\
	if(cache != (button)){\
		cache = (button);\
		cnt = 0;\
	}\
	else if(cnt == 5){\
		if(cache) execution;\
		cnt=11;\
	}\
	else if(cnt < 5) cnt++;\
}

#define Delay(TIM,execution)\
{\
	static uint16_t time=TIM;\
	if(!time--)\
	{\
		time = TIM;\
		execution;\
	}\
}

typedef enum
{
	SHIFT,
	CTRL,
	SHIFT_CTRL,
	NO_CHANGE,
}KeyboardMode_e;

typedef enum
{
	SHORT_CLICK,
	LONG_CLICK,
	NO_CLICK,
}MouseMode_e;

typedef __packed struct
{
    int16_t forward_back_ref;
    int16_t left_right_ref;
    int16_t rotate_ref;
}ChassisSpeed_Ref_t;

typedef enum 
{
	  MoveMode_CAP_STOP_MODE,
	  MoveMode_CAP_RECHARGE_MODE,
	  MoveMode_CAP_RELEASE_LOW_MODE,
	  MoveMode_CAP_RELEASE_HIGH_MODE,
}KeyBoard_MoveMode;
typedef __packed struct
{
	uint32_t value;
	int8_t flag;						//1阻断，0开放
}Sensor;
extern int32_t auto_counter;
extern int32_t auto_counter_stir;
extern int16_t auto_counter_heat0;
extern int16_t auto_counter_heat1;
extern int16_t auto_counter_radarct;
extern ChassisSpeed_Ref_t ChassisSpeedRef; 
extern uint8_t ChassisTwistState;
extern uint8_t chassis_lock;
extern uint8_t chassis_change_forward_back;
extern int16_t chassis_follow_center;
extern float BulletSpeed;
extern int16_t shoot_cd;
extern uint8_t gate_first_enter;
extern int16_t channelrrow;
extern int16_t channelrcol;
extern int16_t channellrow;
extern int16_t channellcol;
extern uint32_t ShootManyBullet_cnt;
extern uint32_t ADC_value[80];
extern uint8_t control_radar_state;
extern float rotate_speed;

void FunctionTaskInit(void);
void ChassisTwist(void);
void ShootOneBullet(uint8_t);
void ShootBullets(uint8_t, uint8_t);
void RefreshADC(void);
#endif /*__FUNCTIONTASK_H*/
