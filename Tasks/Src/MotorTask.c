/**
  ******************************************************************************
  * File Name          : CANMotot.c
  * Description        : CAN电机统一驱动任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

//底盘电机pid
#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,0,{0,0},\
	16.0f,0.0f,2.0f,\
	0,0,0,\
	20000,20000,20000,\
	0,16384,0,0,0,\
	&PID_Calc,&PID_Reset,\
}
//摩擦轮pid
//#define FRIC_MOTOR_SPEED_PID_DEFAULT \
{\
	0,0,{0,0},\
	100.0f,0.005f,7.0f,\
	0,0,0,\
	15000,15000,15000,\
	0,10000,0,0,0,\
	&PID_Calc,&PID_Reset,\
}

//摩擦轮pid
#define FRIC_MOTOR_SPEED_PID_DEFAULT \
{\
	0,0,{0,0},\
	200.0f,0.05f,7.0f,\
	0,0,0,\
	15000,15000,15000,\
	0,10000,0,0,0,\
	&PID_Calc,&PID_Reset,\
}

void ControlNM(MotorINFO *id);
void ControlNM_STIR(MotorINFO *id);
void ControlCM(MotorINFO *id);
void ControlGMY(MotorINFO *id);
void ControlGMP(MotorINFO *id);

uint8_t GMYReseted = 0;
uint8_t GMPReseted = 0;

//**********************************************************************
//					pid(kp,ki,kd,kprM,kirM,kdrM,rM)
//						kprM:kp result Max
//**********************************************************************

//**********************************************************************
//				Chassis_MOTORINFO_Init(func,spid)
//**********************************************************************
MotorINFO CMFL = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMFR = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBL = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO CMBR = Chassis_MOTORINFO_Init(&ControlCM,CHASSIS_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICL = Chassis_MOTORINFO_Init(&ControlCM,FRIC_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICR = Chassis_MOTORINFO_Init(&ControlCM,FRIC_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICAL = Chassis_MOTORINFO_Init(&ControlCM,FRIC_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICAR = Chassis_MOTORINFO_Init(&ControlCM,FRIC_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICBL = Chassis_MOTORINFO_Init(&ControlCM,FRIC_MOTOR_SPEED_PID_DEFAULT);
MotorINFO FRICBR = Chassis_MOTORINFO_Init(&ControlCM,FRIC_MOTOR_SPEED_PID_DEFAULT);

//************************************************************************
//		     Gimbal_MOTORINFO_Init(rdc,func,ppid,spid)
//************************************************************************
//使用云台电机时，请务必确定校准过零点
/*
MotorINFO GMP = Normal_MOTORINFO_Init(1.0,&ControlGMP,
									 fw_PID_INIT(0.45, 0.0, 0.3, 100.0, 100.0, 100.0, 10.0),
									 fw_PID_INIT(16000.0, 200.0, 180.0, 50000.0, 50000.0, 50000.0, 30000.0));
MotorINFO GMY  = Normal_MOTORINFO_Init(1.0,&ControlGMY,
									   fw_PID_INIT(0.35,0,0.8, 	100.0, 0, 100.0, 10.0),
									   fw_PID_INIT(15000.0,300.0,0, 	50000.0, 5000.0, 0, 30000.0));
										 */
MotorINFO GMP = Normal_MOTORINFO_Init(1.0,&ControlGMP,
									 fw_PID_INIT(0.5, 0.02, 0.5, 100.0, 100.0, 100.0, 15.0),
									 fw_PID_INIT(12000.0, 200.0, 1000.0, 50000.0, 50000.0, 50000.0, 30000.0));
								 
MotorINFO GMY = Normal_MOTORINFO_Init(1.0,&ControlGMY,
									 fw_PID_INIT(0.3, 0.02, 1.0, 100.0, 100.0, 100.0, 10.0),  //kp后面改掉了
									 fw_PID_INIT(10000.0, 150.0, 200.0, 50000.0, 50000.0, 50000.0, 30000.0));

//*************************************************************************
//			Normal_MOTORINFO_Init(rdc,func,ppid,spid)
//*************************************************************************
/*
MotorINFO STIR = Normal_MOTORINFO_Init(3591.0f/187.0f,&ControlNM,
								fw_PID_INIT(30.0, 0, 0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(50.0, 0.1, 0.0, 		15000.0, 3000.0, 15000.0, 15000.0));
								*/
MotorINFO STIR = Normal_MOTORINFO_Init(3591.0f/187.0f,&ControlNM_STIR,
								fw_PID_INIT(20.0, 0, 0.2, 	540.0, 0.0, 1080.0, 1080.0),
								fw_PID_INIT(30.0, 0.05, 0.8, 		15000.0, 3000.0, 15000.0, 15000.0));
								
#define CLIMB_SPEED_1 2160.0			
#define CLIMB_SPEED_2 1440.0			
MotorINFO CLIMB_F = Normal_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT(10.0, 0, 0, 	CLIMB_SPEED_1, CLIMB_SPEED_1, CLIMB_SPEED_1, CLIMB_SPEED_1),
								fw_PID_INIT(30, 0.0, 0,15000.0, 15000.0, 15000.0, 15000.0));		
MotorINFO CLIMB_B = Normal_MOTORINFO_Init(19.0,&ControlNM,
								fw_PID_INIT(10.0, 0, 0, 	CLIMB_SPEED_2, CLIMB_SPEED_2, CLIMB_SPEED_2, CLIMB_SPEED_2),
								fw_PID_INIT(30, 0.0, 0,15000.0, 15000.0, 15000.0, 15000.0));	
//MotorINFO CLIMB_F = Normal_MOTORINFO_Init(19.0,&ControlNM,
//								fw_PID_INIT(10.0, 0, 0, 	1080.0, 1080.0, 1080.0, 1080.0),
//								fw_PID_INIT(30, 0.0, 0,15000.0, 15000.0, 15000.0, 15000.0));		
//MotorINFO CLIMB_B = Normal_MOTORINFO_Init(19.0,&ControlNM,
//								fw_PID_INIT(10.0, 0, 0, 	1080.0, 1080.0, 1080.0, 1080.0),
//								fw_PID_INIT(30, 0.0, 0,15000.0, 15000.0, 15000.0, 15000.0));		
MotorINFO GATE = Normal_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT(12.0, 0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(60.0, 0.1, 0.0, 		10000.0, 10000.0, 10000.0, 6000.0));
MotorINFO GMY_FINE_ADJUSTMENT = Normal_MOTORINFO_Init(36.0,&ControlNM,
								fw_PID_INIT(12.0, 0, 0.0, 	1080.0, 1080.0, 1080.0, 1080.0),
								fw_PID_INIT(60.0, 0.1, 0.0, 		10000.0, 10000.0, 10000.0, 6000.0));
								
#ifdef HERO
/*
MotorINFO* can1[8]={0,0,0,0,&GMP,&FRICL,&GMY,&FRICR};
MotorINFO* can2[8]={&CMFL,&CMFR,&CMBL,&CMBR,&CLIMB_F,&CLIMB_B,&STIR,&GATE};
*/

MotorINFO* can1[8]={&FRICAR,&FRICAL,/*&GMY_FINE_ADJUSTMENT*/0,0,0,&GMP,&GMY,0};
MotorINFO* can2[8]={&CMFL,&CMFR,&CMBL,&CMBR,&CLIMB_F,&CLIMB_B,&GATE,&STIR};

//MotorINFO* can1[8]={&GMP,0,0,0,&GMY,0,&FRICL,&FRICR};
//MotorINFO* can2[8]={&CMFL,&CMFR,&CMBL,&CMBR,&CLIMB_F,&CLIMB_B,&STIR,&GATE};
#endif

void ControlNM(MotorINFO* id)
{
	if(id==0) return;
	if(id->s_count == 1)
	{		
		uint16_t 	ThisAngle;	
		double 		ThisSpeed;	
		ThisAngle = id->RxMsgC6x0.angle;				//未处理角度
		if(id->FirstEnter==1) {id->lastRead = ThisAngle;id->FirstEnter = 0;return;}
		if(ThisAngle<=id->lastRead)
		{
			if((id->lastRead-ThisAngle)>4000)//编码器上溢
				id->RealAngle = id->RealAngle + (ThisAngle+8192-id->lastRead) * 360 / 8192.0 / id->ReductionRate;
			else//正常
				id->RealAngle = id->RealAngle - (id->lastRead - ThisAngle) * 360 / 8192.0 / id->ReductionRate;
		}
		else
		{
			if((ThisAngle-id->lastRead)>4000)//编码器下溢
				id->RealAngle = id->RealAngle - (id->lastRead+8192-ThisAngle) *360 / 8192.0 / id->ReductionRate;
			else//正常
				id->RealAngle = id->RealAngle + (ThisAngle - id->lastRead) * 360 / 8192.0 / id->ReductionRate;
		}
		ThisSpeed = id->RxMsgC6x0.RotateSpeed * 6 / id->ReductionRate;
		
		id->Intensity = (int16_t)PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,ThisSpeed);
		
		id->s_count = 0;
		id->lastRead = ThisAngle;
	}
	else
	{
		id->s_count++;
	}		
}

void ControlNM_STIR(MotorINFO* id)
{
	if(id==0) return;
	if(id->s_count == 1)
	{		
		uint16_t 	ThisAngle;	
		double 		ThisSpeed;	
		ThisAngle = id->RxMsgC6x0.angle;				//未处理角度
		if(id->FirstEnter==1) {id->lastRead = ThisAngle;id->FirstEnter = 0;return;}
		if(ThisAngle<=id->lastRead)
		{
			if((id->lastRead-ThisAngle)>6000)//编码器上溢
				id->RealAngle = id->RealAngle + (ThisAngle+8192-id->lastRead) * 360 / 8192.0 / id->ReductionRate;
			else//正常
				id->RealAngle = id->RealAngle - (id->lastRead - ThisAngle) * 360 / 8192.0 / id->ReductionRate;
		}
		else
		{
			if((ThisAngle-id->lastRead)>2000)//编码器下溢
				id->RealAngle = id->RealAngle - (id->lastRead+8192-ThisAngle) *360 / 8192.0 / id->ReductionRate;
			else//正常
				id->RealAngle = id->RealAngle + (ThisAngle - id->lastRead) * 360 / 8192.0 / id->ReductionRate;
		}
		ThisSpeed = id->RxMsgC6x0.RotateSpeed * 6 / id->ReductionRate;
		
		id->Intensity = (int16_t)PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,ThisSpeed);
		
		id->s_count = 0;
		id->lastRead = ThisAngle;
	}
	else
	{
		id->s_count++;
	}		
}

void ControlCM(MotorINFO* id)
{
	//TargetAngle 代作为目标速度
	if(id==0) return;
	id->offical_speedPID.ref = (float)(id->TargetAngle);
	id->offical_speedPID.fdb = id->RxMsgC6x0.RotateSpeed;
	id->offical_speedPID.Calc(&(id->offical_speedPID));
	id->Intensity=(1.30f)*id->offical_speedPID.output;
}

void ControlGMY(MotorINFO* id)
{
	if(id==0) return;

	#ifdef USE_CHASSIS_FOLLOW
	static	uint8_t	ChassisLockRCD = 0;
		#ifndef USE_GIMBAL_ENCODER
			float 	ThisAngle = imu.yaw;
			float 	ThisSpeed = -imu.wz;
			
			if(chassis_lock)
			{
				ThisAngle = - (float)(GM_YAW_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
				ThisSpeed = -imu.wz;
			}
		#else
				float 	ThisAngle = - (float)(GM_YAW_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
			float 	ThisSpeed = -imu.wz;
		#endif
	#else
		double 	ThisAngle = - (double)(GM_YAW_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
		float 	ThisSpeed = -imu.wz;
	#endif
			
	int8_t 	dir;
	if(id->ReductionRate>=0) dir=1;
	else dir=-1;

	if(id->FirstEnter==1) {
		id->lastRead = ThisAngle;
			id->RealAngle = - (double)(GM_YAW_ZERO - id->RxMsgC6x0.angle) * 360.0 / 8192.0 / id->ReductionRate;
		NORMALIZE_ANGLE180(id->RealAngle);
		id->FirstEnter = 0;
		return;
	}
	
	if(ThisAngle <= id->lastRead)
	{
		if((id->lastRead-ThisAngle) > 180)
			 id->RealAngle += (ThisAngle + 360 - id->lastRead)*dir;
		else
			 id->RealAngle -= (id->lastRead - ThisAngle)*dir;
	}
	else
	{
		if((ThisAngle-id->lastRead) > 180)
			 id->RealAngle -= (id->lastRead + 360 - ThisAngle)*dir;
		else
			 id->RealAngle += (ThisAngle - id->lastRead)*dir;
	}
	
	if(chassis_lock != ChassisLockRCD)
	{
		id->TargetAngle = id->RealAngle;
		ChassisLockRCD = chassis_lock;
	}

	if(fabs((GMY.RxMsgC6x0.angle - chassis_follow_center) * 360 / 8192.0f) < 10) GMYReseted = 1;
	
	if(GMYReseted==0) id->positionPID.outputMax = 1.0;
	else id->positionPID.outputMax = 10.0;
	
	id->lastRead = ThisAngle;
	if(chassis_lock)
	{
	  id->positionPID.kp = 0.55;
	}
	else
	{
	  id->positionPID.kp = 0.3;
	}
	id->Intensity = (int16_t)PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,ThisSpeed);
	if(chassis_lock)
	{
  if(id->Intensity>500)
	{
		id->Intensity += 2000;
	}
	else if(id->Intensity<-500)
	{
	  id->Intensity -= 2000;
	}
  }
}

void ControlGMP(MotorINFO* id)
{
	if(id==0) return;
 
	#ifdef USE_CHASSIS_FOLLOW
		static	uint8_t	ChassisLockRCD = 0;
		#ifndef USE_GIMBAL_ENCODER
			float 	ThisAngle = -imu.pit;
			//float 	ThisAngle = - (float)(GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
			float 	ThisSpeed = imu.wy;
			if(chassis_lock)
			{
				ThisAngle = - (float)(GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
			}
		#else
			float 	ThisAngle = - (float)(GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
			float 	ThisSpeed = -imu.wy;
		#endif
	#else
		double 	ThisAngle = - (double)(GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f;
	#endif
	int8_t 	dir;
	if(id->ReductionRate>=0) dir=1;
	else dir=-1;
	
	if(id->FirstEnter==1) {
		id->lastRead = ThisAngle;
		id->RealAngle = - (double)(GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0f / 8192.0f / id->ReductionRate;
		NORMALIZE_ANGLE180(id->RealAngle);
		id->FirstEnter = 0;
		return;
	}
	
	if(ThisAngle <= id->lastRead)
	{
		if((id->lastRead-ThisAngle) > 180)
			 id->RealAngle += (ThisAngle + 360 - id->lastRead)*dir;
		else
			 id->RealAngle -= (id->lastRead - ThisAngle)*dir;
	}
	else
	{
		if((ThisAngle-id->lastRead) > 180)
			 id->RealAngle -= (id->lastRead + 360 - ThisAngle)*dir;
		else
			 id->RealAngle += (ThisAngle - id->lastRead)*dir;
	}
	
	if(fabs(id->RealAngle - id->TargetAngle) < 10) GMPReseted = 1;
	
	
	if(chassis_lock != ChassisLockRCD)
	{
		id->TargetAngle = id->RealAngle;
		ChassisLockRCD = chassis_lock;
	}

	#ifdef HERO
		MINMAX(id->TargetAngle, id->RealAngle + (GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0 / 8192.0 / id->ReductionRate - 32.0f, id->RealAngle + (GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0 / 8192.0 / id->ReductionRate + 20.0f);
		/*
		if(GM_Turn_back)
		{
			MINMAX(id->TargetAngle, id->RealAngle + (GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0 / 8192.0 / id->ReductionRate - 32.0f, id->RealAngle + (GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0 / 8192.0 / id->ReductionRate - 2.5f);
		}
		
		if(ChassisTwistState)
		{
			MINMAX(id->TargetAngle, id->RealAngle + (GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0 / 8192.0 / id->ReductionRate - 32.0f, id->RealAngle + (GM_PITCH_ZERO - id->RxMsgC6x0.angle) * 360.0 / 8192.0 / id->ReductionRate + 5.0f);
		}
		*/
	#endif
	
	if(GMPReseted==0) id->positionPID.outputMax = 1.0;
	else id->positionPID.outputMax = 10.0;
	
	id->lastRead = ThisAngle ;
	int16_t tmp = (int16_t)PID_PROCESS_Double(&(id->positionPID),&(id->speedPID),id->TargetAngle,id->RealAngle,ThisSpeed);
	if(chassis_lock)
	{
	if(tmp>500)
	{
		id->Intensity += 4000;
	}
	else if(tmp<-500)
	{
	  id->Intensity -= 4000;
	}
  }
	MINMAX(tmp, -30000.0-GM_PITCH_GRAVITY_COMPENSATION, 30000.0-GM_PITCH_GRAVITY_COMPENSATION);
	id->Intensity = GM_PITCH_GRAVITY_COMPENSATION + tmp;
}

//CAN
void setCAN11()
{
	CanTxMsgTypeDef pData;
	hcan1.pTxMsg = &pData;
	
	hcan1.pTxMsg->StdId = 0x200;
	hcan1.pTxMsg->ExtId = 0;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can1[i]==0) {
			hcan1.pTxMsg->Data[i*2]   = 0;
			hcan1.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan1.pTxMsg->Data[i*2]   = (uint8_t)(can1[i]->Intensity >> 8);
			hcan1.pTxMsg->Data[i*2+1] = (uint8_t)can1[i]->Intensity;
		}
	}

	if(can1_update == 1 && can1_type == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK)
		{
			Error_Handler();
		}
		
		can1_update = 0;
		
		#ifdef CAN12
			can1_type = 2;
		#else
		#ifdef DOUBLE_BOARD_CAN1
			can1_type = 3;
		#endif
		#endif
		
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}
}
void setCAN12()
{
	CanTxMsgTypeDef pData;
	hcan1.pTxMsg = &pData;
	
	hcan1.pTxMsg->StdId = 0x1ff;
	hcan1.pTxMsg->ExtId = 0;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can1[i+4]==0) {
			hcan1.pTxMsg->Data[i*2]   = 0;
			hcan1.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan1.pTxMsg->Data[i*2]   = (uint8_t)(can1[i+4]->Intensity >> 8);
			hcan1.pTxMsg->Data[i*2+1] = (uint8_t)can1[i+4]->Intensity;
		}
	}

	if(can1_update == 1 && can1_type == 2)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		if(HAL_CAN_Transmit_IT(&hcan1) != HAL_OK)
		{
			Error_Handler();
		}
		
		can1_update = 0;
		
		#ifdef DOUBLE_BOARD_CAN1
			can1_type = 3;
		#else
		#ifdef CAN11
			can1_type = 1;
		#endif
		#endif
		
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
  }
}
void setCAN21()
{
	CanTxMsgTypeDef pData;
	hcan2.pTxMsg = &pData;
	
	hcan2.pTxMsg->StdId = 0x200;
	hcan2.pTxMsg->ExtId = 0;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can2[i]==0) {
			hcan2.pTxMsg->Data[i*2]   = 0;
			hcan2.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan2.pTxMsg->Data[i*2]   = (uint8_t)(can2[i]->Intensity >> 8);
			hcan2.pTxMsg->Data[i*2+1] = (uint8_t)can2[i]->Intensity;
		}
	}

	if(can2_update == 1 && can2_type == 1)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		if(HAL_CAN_Transmit_IT(&hcan2) != HAL_OK)
		{
			Error_Handler();
		}
		
		can2_update = 0;
		
		#ifdef CAN22
			can2_type = 2;
		#else
		#ifdef DOUBLE_BOARD_CAN2
			can2_type = 3;
		#endif
		#endif
		
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
  }
}
void setCAN22()
{
	CanTxMsgTypeDef pData;
	hcan2.pTxMsg = &pData;
	
	hcan2.pTxMsg->StdId = 0x1ff;
	hcan2.pTxMsg->ExtId = 0;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->DLC = 0x08;
	
	for(int i=0;i<4;i++){
		if(can2[i+4]==0) {
			hcan2.pTxMsg->Data[i*2]   = 0;
			hcan2.pTxMsg->Data[i*2+1] = 0;
		}
		else {
			hcan2.pTxMsg->Data[i*2]   = (uint8_t)(can2[i+4]->Intensity >> 8);
			hcan2.pTxMsg->Data[i*2+1] = (uint8_t)can2[i+4]->Intensity;
		}
	}

	if(can2_update == 1 && can2_type == 2)
	{
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		if(HAL_CAN_Transmit_IT(&hcan2) != HAL_OK)
		{
			Error_Handler();
		}
		
		can2_update = 0;
		
		#ifdef DOUBLE_BOARD_CAN2
			can2_type = 3;
		#else
		#ifdef CAN21
			can2_type = 1;
		#endif
		#endif
		
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
  }
}

void InitMotor(MotorINFO *id)
{
	if(id==0) return;
	id->FirstEnter=1;
	id->lastRead=0;
	id->RealAngle=0;
	id->TargetAngle=0;
	id->offical_speedPID.Reset(&(id->offical_speedPID));
	(id->Handle)(id);
	id->Intensity=0;
}

void Motor_ID_Setting()
{
	for(int i=0;i<4;i++)
	{
		if(can1[i]!=0) 
		{
			can1[i]->CAN_TYPE=&hcan1;
			can1[i]->RXID = 0x201+i;
			can1[i]->TXID = 0x200;
		}
		if(can2[i]!=0) 
		{
			can2[i]->CAN_TYPE=&hcan2;
			can2[i]->RXID = 0x201+i;
			can2[i]->TXID = 0x200;
		}
	}
	for(int i=4;i<8;i++)
	{
		if(can1[i]!=0) 
		{
			can1[i]->CAN_TYPE=&hcan1;
			can1[i]->RXID = 0x201+i;
			can1[i]->TXID = 0x1ff;
		}
		if(can2[i]!=0) 
		{
			can2[i]->CAN_TYPE=&hcan2;
			can2[i]->RXID = 0x201+i;
			can2[i]->TXID = 0x1ff;
		}
	}
}
