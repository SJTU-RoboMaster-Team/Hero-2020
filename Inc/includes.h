/**
  ******************************************************************************
  * File Name          : includes.h
  * Description        : 统一包含文件
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __INCLUDES_H
#define __INCLUDES_H

//车辆配置
#define HERO

#define BULLET_TYPE 1
#define YAW_DIR (1.0f)
#define PIT_DIR (-1.0f)

//双板通信配置
#define DOUBLE_BOARD_CAN1
//#define DOUBLE_BOARD_CAN2
#if defined (DOUBLE_BOARD_CAN1) || (DOUBLE_BOARD_CAN2)
#define BOARD_MAIN
//#define BOARD_SLAVE
#endif

#define USE_AUTOAIM
#define USE_IMU
#define USE_CHASSIS_FOLLOW
#define USE_HEAT_LIMIT
#define USE_POWERLIMITATION
//#define USE_GIMBAL_ENCODER
//#define REMOTE_CONTROL

#ifdef HERO
	#define CAN11
	#define CAN12
	#define CAN21
	#define CAN22
#endif

#define getbit(x,y) (x >> y) & 1)//获取某一位的值
#define jointbyte(x,y) ((uint16_t)(x << 8) | y)//连接2个byte

#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "iwdg.h"
#include "adc.h"
#include "math.h"
#include "dac.h"

#include "AuxDevice.h"
#include "RemoteTask.h"
#include "FunctionTask.h"
#include "pid_regulator.h"
#include "CANTask.h"
#include "MotorTask.h"
#include "ControlTask.h"
#include "drivers_ramp.h"
#include "AutoAimTask.h"
#include "JudgeTask.h"
#include "Cap2ControlTask.h"
#include "PowerLimitationTask.h"
#include "bsp_imu.h"
#include "KalmanFilter.h"

#endif /* __INCLUDES_H */
