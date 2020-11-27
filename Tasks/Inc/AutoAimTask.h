/**
  ******************************************************************************
  * File Name          : AutoAimtask.h
  * Description        : 
  ******************************************************************************
  *
  * Copyright (c) 2019 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */

#ifndef __AUTOAIMTASK_H
#define __AUTOAIMTASK_H

#include "bsp_imu.h"
#include "includes.h"

#ifdef	USE_AUTOAIM

#define RX_ENEMY_START		Enemy_INFO[0]
#define RX_ENEMY_YAW1			Enemy_INFO[1]
#define RX_ENEMY_YAW2 		Enemy_INFO[2]
#define RX_ENEMY_PITCH1 	Enemy_INFO[3]
#define RX_ENEMY_PITCH2		Enemy_INFO[4]
#define RX_ENEMY_DIS1			Enemy_INFO[5]
#define RX_ENEMY_DIS2 		Enemy_INFO[6]
#define RX_ENEMY_END 			Enemy_INFO[7]

#define PREDICT_MODE 1
#define ANTI_GYRO_MODE 2

typedef struct
{
	float yaw;
	float pitch;
}Gimbal_Angle_t;

typedef struct
{
	Gimbal_Angle_t gimbal_angle;//单位：°
	float dist;//单位：mm
	uint16_t fps;
}Autoaim_Info_t;

typedef struct
{
	float k_l;
	float k_theta;
	float k_theta_2;
	float k_l_theta;
	float constant;
}Autoaim_Pitch_Offset_t;

typedef struct
{
	int16_t last_change_time1;
	int16_t last_change_time2;
	bool isPeriod1;
	float half_period_1;
	float half_period_2;
	float center;
}Autoaim_Gyro_t;

typedef struct
{
	float predict_window;
	float anti_gyro_window;
}Shoot_Window_t;

extern uint8_t aim_delay_time;
extern uint8_t aim_mode;
extern uint8_t find_enemy;
extern uint8_t auto_shoot_flag;
extern uint16_t auto_counter_autoaim;
extern Gimbal_Angle_t GM_RealAngle_RCD;
extern imu_t imu_w_RCD;

void InitAutoAim(void);
void Autoaim_Receive_Start(void);
void AutoAimUartRxCpltCallback(void);
void AutoAim(void);
void AutoShoot(uint8_t mode);
Gimbal_Angle_t GM_RealAngle_Rcd(MotorINFO *GMY, MotorINFO *GMP, int aim_delay_time);
imu_t imu_w_rcd(imu_t *imu_current, int aim_delay_time);

#endif /*USE_AUTOAIM*/

#endif /*__AUTOAIMTASK_H*/
