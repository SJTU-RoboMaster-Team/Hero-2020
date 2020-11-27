/**
  ******************************************************************************
  * File Name          : ControlTask.h
  * Description        : 主控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __CONTROLTASK_H
#define __CONTROLTASK_H

#include "includes.h"

//When you change the struct WorkState_e please ensure that 
//*********************************************
//all your element's value must more than zero 
//*********************************************
//execpt the stop and prepare state.

#define L1	262
#define L1U	277
#define L2	294
#define L2U	311
#define L3	330
#define L4	349
#define L4U	370
#define L5	392
#define L5U	415
#define L6	440
#define L6U	466
#define L7	494

#define M1	523
#define M1U	554
#define M2	587
#define M2U	622
#define M3	659
#define M4	698
#define M4U	740
#define M5	784
#define M5U	831
#define M6	880
#define M6U	932
#define M7	988

#define H1	1046
#define H1U	1109
#define H2	1175
#define H2U	1245
#define H3	1318
#define H4	1397
#define H4U	1480
#define H5	1568
#define H5U	1661
#define H6	1760
#define H6U	1865
#define H7	1976

#define CAN1_SHUTDOWN(i)\
{\
	can1[i]->FirstEnter=1;\
	can1[i]->lastRead=0;\
	can1[i]->RealAngle=0;\
	can1[i]->TargetAngle=0;\
	can1[i]->offical_speedPID.Reset(&(can1[i]->offical_speedPID));\
	(can1[i]->Handle)(can1[i]);\
}

#define CAN2_SHUTDOWN(i)\
{\
	can2[i]->FirstEnter=1;\
	can2[i]->lastRead=0;\
	can2[i]->RealAngle=0;\
	can2[i]->TargetAngle=0;\
	can2[i]->offical_speedPID.Reset(&(can2[i]->offical_speedPID));\
	(can2[i]->Handle)(can2[i]);\
}

typedef struct {
	uint16_t note;
	uint16_t time;
}MusicNote;

#define PLAY(note, time) {\
	while(auto_counter != 0);\
	if(note == 0){ \
		__HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, 0);\
	}else{ \
		__HAL_TIM_SET_AUTORELOAD(&BUZZER_TIM, 1000000 / note); \
		__HAL_TIM_SET_COMPARE(&BUZZER_TIM, TIM_CHANNEL_1, 500000 / note); \
	}\
	auto_counter=time;\
}

#define freq_div(func, times)\
{\
	static uint16_t cnt = 0;\
	if(cnt == 0)\
	{\
		func;\
	}\
	if(cnt++ >= times)\
	{\
		cnt = 0;\
	}\
}

typedef enum
{
	STOP_STATE=-1,
	PREPARE_STATE=0, 
	NORMAL_STATE=1,
	ADDITIONAL_STATE_ONE=2,
	ADDITIONAL_STATE_TWO=3,
}WorkState_e;

extern WorkState_e WorkState;
extern WorkState_e RxWorkState;
extern uint16_t IWDG_counter;
extern uint8_t GM_Turn_back;
void WorkStateFSM(void);

#endif /*__ CONTROLTASK_H */
