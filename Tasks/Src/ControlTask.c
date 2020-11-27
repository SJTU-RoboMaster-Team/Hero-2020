/**
  ******************************************************************************
  * File Name          : ControlTask.c
  * Description        : 主控制任务
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

WorkState_e WorkState = PREPARE_STATE;
WorkState_e RxWorkState = PREPARE_STATE;
uint16_t prepare_time = 0;
uint16_t counter = 0;
uint8_t	no_signal = 0;
uint16_t IWDG_counter = 0;
float rotate_speed = 0;
fw_PID_Regulator_t chassis_rotate_pid = fw_PID_INIT(9.0f, 0.5f, 3.0f, 3000.0, 3000.0, 3000.0, 1500.0);

MusicNote SuperMario[] = {
	{H3, 100}, {0, 50}, 
	{H3, 250}, {0, 50}, 
	{H3, 100}, {0, 50}, 
	{0, 150},
	{H1, 100}, {0, 50},  
	{H3, 250}, {0, 50},
	{H5, 250}, {0, 50},
	{0, 300},
	{M5, 250}, {0, 50},
	{0, 300},
	{H1, 250}, {0, 50}
};

void playMusicSuperMario(void){
	HAL_TIM_PWM_Start(&BUZZER_TIM, TIM_CHANNEL_1);
	for(int i = 0; i < sizeof(SuperMario) / sizeof(MusicNote); i++){
			PLAY(SuperMario[i].note, SuperMario[i].time);
	}
	HAL_TIM_PWM_Stop(&BUZZER_TIM, TIM_CHANNEL_1);
}

void IWDG_Handler(void)
{
	#ifdef BOARD_SLAVE
		HAL_IWDG_Refresh(&hiwdg);
	#endif
	
	if(IWDG_counter < 1000)
		IWDG_counter++;
	if(IWDG_counter > 100)
	{
		no_signal = 1;
		WorkState = STOP_STATE;
		inputmode = STOP;
	}
	if(IWDG_counter < 500)
	{
		HAL_IWDG_Refresh(&hiwdg);
	}
}

//状态机切换
void WorkStateFSM(void)
{
	#ifndef BOARD_SLAVE
	switch (WorkState)
	{
		case PREPARE_STATE:				//准备模式
		{
			//if (inputmode == STOP) WorkState = STOP_STATE;
			if(prepare_time < 1000) prepare_time++;
			if(prepare_time >= 1000 && imu.InitFinish == 1 && isCan11FirstRx == 1 && isCan12FirstRx == 1 && isCan21FirstRx == 1 && isCan22FirstRx == 1)//imu初始化完成且所有can电机上电完成后进入正常模式
			{
				playMusicSuperMario();
				chassis_rotate_pid.Reset(&chassis_rotate_pid);
				WorkState = NORMAL_STATE;
				#ifdef BOARD_SLAVE
				WorkState = RxWorkState;
				#endif
				prepare_time = 0;
			}
			for(int i=0;i<8;i++) 
			{
				{
					InitMotor(can1[i]);
					InitMotor(can2[i]);
				}
			}
			#ifdef CAN11
			setCAN11();
			#endif
			#ifdef CAN12
			setCAN12();
			#endif
			#ifdef CAN21
			setCAN21();
			#endif
			#ifdef CAN22
			setCAN22();
			#endif
			FunctionTaskInit();
		}break;
		case NORMAL_STATE:				//正常模式
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			
			if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
			if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
		}break;
		case ADDITIONAL_STATE_ONE:		//附加模式一
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			
			if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
			if(functionmode == LOWER_POS) WorkState = ADDITIONAL_STATE_TWO;
		}break;
		case ADDITIONAL_STATE_TWO:		//附加模式二
		{
			if (inputmode == STOP) WorkState = STOP_STATE;
			
			if(functionmode == UPPER_POS) WorkState = NORMAL_STATE;
			if(functionmode == MIDDLE_POS) WorkState = ADDITIONAL_STATE_ONE;
		}break;
		case STOP_STATE:				//紧急停止
		{
			for(int i=0;i<8;i++) 
			{
				if(can1[i]==&FRICAL || can1[i]==&FRICAR || can1[i]==&FRICBL || can1[i]==&FRICBR)
				{
					CAN1_SHUTDOWN(i);
					InitMotor(can2[i]);
				}
				else if(can2[i]==&FRICAL || can2[i]==&FRICAR || can2[i]==&FRICBL || can2[i]==&FRICBR)
				{
					InitMotor(can1[i]);
					CAN2_SHUTDOWN(i);
				}
				else
				{
					InitMotor(can1[i]);
					InitMotor(can2[i]);
				}
			}
			#ifdef CAN11
			setCAN11();
			#endif
			#ifdef CAN12
			setCAN12();
			#endif
			#ifdef CAN21
			setCAN21();
			#endif
			#ifdef CAN22
			setCAN22();
			#endif
			if (inputmode == REMOTE_INPUT || inputmode == KEY_MOUSE_INPUT)
			{
				WorkState = PREPARE_STATE;
				GMYReseted=0;
				GMPReseted=0;
				ChassisTwistState = 0;
				FunctionTaskInit();
			}
		}break;
		default: break;
	}
	#else
	WorkState = RxWorkState;
	#endif
}

void ControlRotate(void)
{	
	#ifdef USE_CHASSIS_FOLLOW
		ChassisTwist();
		NORMALIZE_ANGLE180(ChassisSpeedRef.rotate_ref);
	#endif
	if(abs(ChassisSpeedRef.rotate_ref) < 15)
	{
		chassis_rotate_pid.componentKi = 0;
	}
	rotate_speed = - YAW_DIR * PID_PROCESS(&chassis_rotate_pid, 0, (float)ChassisSpeedRef.rotate_ref);
}

void Chassis_Data_Decoding()
{
	if(!chassis_lock)
	{
				ControlRotate();
			
		
		float cosPlusSin, cosMinusSin, GMYEncoderAngle;
		
			GMYEncoderAngle = -YAW_DIR * (GM_YAW_ZERO - GMY.RxMsgC6x0.angle) * 6.28f / 8192.0f;
		
		cosPlusSin = cos(GMYEncoderAngle) + sin(GMYEncoderAngle);
		cosMinusSin = cos(GMYEncoderAngle) - sin(GMYEncoderAngle);
		
		#ifdef USE_CHASSIS_FOLLOW
		CMFL.TargetAngle = (	ChassisSpeedRef.forward_back_ref * cosPlusSin + ChassisSpeedRef.left_right_ref * cosMinusSin + rotate_speed)*12;
		CMFR.TargetAngle = (- ChassisSpeedRef.forward_back_ref * cosMinusSin + ChassisSpeedRef.left_right_ref * cosPlusSin + rotate_speed)*12;
		CMBL.TargetAngle = (  ChassisSpeedRef.forward_back_ref * cosMinusSin - ChassisSpeedRef.left_right_ref * cosPlusSin + rotate_speed)*12;
		CMBR.TargetAngle = (- ChassisSpeedRef.forward_back_ref * cosPlusSin - ChassisSpeedRef.left_right_ref * cosMinusSin + rotate_speed)*12;
		#else
		CMFL.TargetAngle = (  ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref	+ rotate_speed)*12;
		CMFR.TargetAngle = (- ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + rotate_speed)*12;
		CMBL.TargetAngle = (  ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + rotate_speed)*12;
		CMBR.TargetAngle = (- ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref	+ rotate_speed)*12;
		#endif
		
		if(GM_Turn_back)
		{
		CMFL.TargetAngle = 0.25*CMFL.TargetAngle;
		CMFR.TargetAngle = 0.25*CMFR.TargetAngle;
		CMBL.TargetAngle = 0.25*CMBL.TargetAngle;
		CMBR.TargetAngle = 0.25*CMBR.TargetAngle;
		}
		
	}
	else
	{
		CMFL.TargetAngle = 0;
		CMFR.TargetAngle = 0;
		CMBL.TargetAngle = 0;
		CMBR.TargetAngle = 0;
	}
}

//主控制循环
void controlLoop()
{
	WorkStateFSM();
	//裁判系统
	getJudgeState();
	freq_div(fakeHeatCalc(), 100);
	//Control_radar(void);//控制雷达 测试用
	//板间can通信
	#ifdef DOUBLE_BOARD_CAN1
	CANTxInfo(&hcan1);
	#endif
	#ifdef DOUBLE_BOARD_CAN2
	CANTxInfo(&hcan2);
	#endif
	//imu解算
	#ifndef BOARD_SLAVE
		mpu_get_data();
		imu_ahrs_update();
		imu_attitude_update();
	#endif
	//超级电容
	freq_div(Cap_Run(),2)
	//自瞄
	GM_RealAngle_RCD = GM_RealAngle_Rcd(&GMY, &GMP, aim_delay_time);
	imu_w_RCD = imu_w_rcd(&imu, aim_delay_time);
	AutoAim();
	
	if(WorkState > 0)
	{
		Chassis_Data_Decoding();
		
		PowerLimitation_liuchang();
		
		for(int i=0;i<8;i++) if(can1[i]!=0) (can1[i]->Handle)(can1[i]);
		for(int i=0;i<8;i++) if(can2[i]!=0) (can2[i]->Handle)(can2[i]);

		#ifdef CAN11
		setCAN11();
		#endif
		#ifdef CAN12
		setCAN12();
		#endif
		#ifdef CAN21
		setCAN21();
		#endif
		#ifdef CAN22
		setCAN22();
		#endif
	}
}

//时间中断入口函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)//1ms时钟`
	{
		HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
		//主循环
		controlLoop();
		
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}
	else if (htim->Instance == htim7.Instance)//ims时钟
	{
		rc_cnt++;
		if(auto_counter > 0) auto_counter--;
		if(auto_counter_stir > 0) auto_counter_stir--;
		if(auto_counter_autoaim > 0) auto_counter_autoaim--;
		if(auto_counter_autoaim == 0) auto_counter_autoaim = 5000;
		if(auto_counter_heat0 > 0) auto_counter_heat0--;
		if(auto_counter_heat1 > 0) auto_counter_heat1--;
		if(channellcol>500) ShootManyBullet_cnt++;
		else ShootManyBullet_cnt=0;
		if(shoot_cd > 0) shoot_cd--;
		if(auto_counter_radarct > 0) auto_counter_radarct--;
		
		//看门狗处理
		IWDG_Handler();
		if (rx_free == 1)
		{
			if( (rc_cnt <= 17) && (rc_first_frame == 1))
			{
				RemoteDataProcess(rc_data);				//遥控器数据解算
				HAL_UART_AbortReceive(&RC_UART);
				rx_free = 0;
				while(HAL_UART_Receive_DMA(&RC_UART, rc_data, 18)!= HAL_OK);
				if(counter >= 5) 
				{
					Referee_Transmit();
					counter = 0;
				}
				else counter++;	
				rc_cnt = 0;
			}
			else
			{
				if(rc_first_frame == 0) 
				{
				   WorkState = PREPARE_STATE;
				   HAL_UART_AbortReceive(&RC_UART);
				   while(HAL_UART_Receive_DMA(&RC_UART, rc_data, 18)!= HAL_OK);
  				 rc_cnt = 0;
				   rc_first_frame = 1;
				}
			}
			rc_update = 0;
		}
	}
}
