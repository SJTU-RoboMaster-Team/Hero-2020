/**
  ******************************************************************************
  *FileName				: AutoAimTask.c
  *Description		: 自瞄程序
  *Author					: 管易恒
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
*/

#include "includes.h"

#ifdef	USE_AUTOAIM

#define USE_PREDICT

//通信系数
#define K_ANGLE														(180.0f / (32768.0f - 1.0f))
#define K_DIST														(10000.0f / (32768.0f - 1.0f))
#define K_YAW															(0.75f)
#define K_PIT															(-0.1f)
//角度补偿
#ifdef HERO
	#define YAW_OFFSET											(-0.2f)
	#define PIT_EXTRA_OFFSET								(-2.0f)
#endif
//预测参数    todo
#ifdef HERO
	#define DT0															(60)
	#define DT1															(150 + aim.dist / BulletSpeed)
	#define DT2															(135)
	#define PREDICT_SHOOTABLE_BOUND_BELOW		(0.4f)
	#define PREDICT_SHOOTABLE_BOUND_ABOVE		(0.5f)
#endif
//反陀螺参数
#define ANTI_GYRO_THRES										(6.0f)
#define ANTI_GYRO_SHOOTABLE_BOUND					(5)

fw_PID_Regulator_t AutoAim_Yaw_pid = fw_PID_INIT(1.2f, 0.05f, 0.1f, 10.0, 8.0, 3.0, 10.0);
fw_PID_Regulator_t AutoAim_Pitch_pid = fw_PID_INIT(1.0f, 0.01f, 0.0f, 10.0, 5.0, 3.0, 10.0);

uint8_t autoaim_recv_msg[8];
Autoaim_Info_t aim;

Gimbal_Angle_t aim_real;
Gimbal_Angle_t aim_output;
Gimbal_Angle_t offset;
Gimbal_Angle_t adjust;
Autoaim_Pitch_Offset_t pitch_offset;

Gimbal_Angle_t GM_RealAngle_RCD;
imu_t imu_w_RCD;

uint8_t aim_delay_time = DT0;
uint16_t predict_time = DT0 + DT2;

uint8_t aim_mode = 0;
uint8_t	find_enemy = 0;
uint8_t	aim_finish = 1;
uint8_t auto_shoot_flag = 0;
uint16_t auto_counter_autoaim = 0;
uint16_t receive_cnt = 0;
float thres = ANTI_GYRO_THRES;
float cur_v = 0;
float cur_a = 0;
float predict_angle[2];

Shoot_Window_t shoot_window;
Autoaim_Gyro_t gyro;

Kalman_Filter_t a_filter;
Kalman_Filter_t d1a_filter;
Kalman_Filter_t d2a_filter;
Kalman_Filter_t output_filter;
Kalman_Filter_t gyro_filter;

//自瞄初始化
//Called in main.c & RemoteTask.c
void InitAutoAim(void)
{
	Autoaim_Receive_Start();
	
	adjust.yaw = 0;
	adjust.pitch = 0;
	
	Kalman_Filter_Init(&a_filter, 1, 1, 0, 1, 0, 0);
	Kalman_Filter_Init(&d1a_filter, 1, 1, 0.62, 1, 1, 0);
	Kalman_Filter_Init(&d2a_filter, 1, 1, 1, 1, 40, 0);
	Kalman_Filter_Init(&output_filter, 1, 1, 4, 1, 20, 0);
	Kalman_Filter_Init(&gyro_filter, 1, 1, 20, 1, 400, 0);
}

//自瞄串口开启接收
void Autoaim_Receive_Start(void)
{
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&autoaim_recv_msg,8)!= HAL_OK)
	{
		Error_Handler();
	}
}

//自瞄UART回调函数
//Called in RemoteTask.c
void AutoAimUartRxCpltCallback()
{
	if(autoaim_recv_msg[0] == 0x7f && autoaim_recv_msg[7] == 0x26)
	{
		int16_t yaw_tmp = jointbyte(autoaim_recv_msg[1], autoaim_recv_msg[2]);
		int16_t pit_tmp = jointbyte(autoaim_recv_msg[3], autoaim_recv_msg[4]);
		int16_t dis_tmp = jointbyte(autoaim_recv_msg[5], autoaim_recv_msg[6]);
//		yaw_tmp = (yaw_tmp > 0x7fff) ? (yaw_tmp - 0xffff) : yaw_tmp;
//		pit_tmp = (pit_tmp > 0x7fff) ? (pit_tmp - 0xffff) : pit_tmp;
//		aim.gimbal_angle.yaw=YAW_DIR*(float)((((autoaim_recv_msg[1]<<8)|autoaim_recv_msg[2])>0x7fff) ? (((RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2)-0xffff) : (RX_ENEMY_YAW1<<8)|RX_ENEMY_YAW2 )*k_angle;
//		aim.pitch=PIT_DIR*(float)((((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)>0x7fff) ? (((RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2)-0xffff) : (RX_ENEMY_PITCH1<<8)|RX_ENEMY_PITCH2 )*k_angle;
//		enemy_dist=(float)((RX_ENEMY_DIS1<<8)|RX_ENEMY_DIS2)*k_distance;
		aim.gimbal_angle.yaw = (float)yaw_tmp * K_ANGLE * K_YAW + offset.yaw;
		aim.gimbal_angle.pitch = (float)pit_tmp * K_ANGLE * K_PIT;
		aim.dist = dis_tmp * K_DIST;
		
		find_enemy = 1;
		aim_finish = 0;
		receive_cnt++;
	}
	if(HAL_UART_Receive_DMA(&AUTOAIM_UART,(uint8_t *)&autoaim_recv_msg,8)!= HAL_OK)
	{
		Error_Handler();
	}
}

//pitch角度补偿计算
//Called in AutoAimProcess()
float Autoaim_Pitch_Offset(Autoaim_Pitch_Offset_t* offset, float speed)
{
	switch((uint8_t)speed)
	{
		case 28:
		{
			offset->k_l = 0.00035f;
			offset->k_theta = -0.0001f;
			offset->k_theta_2 = -0.00009f;
			offset->k_l_theta = 0;
			offset->constant = 0.006f;
		}break;
		case 16:
		{
			offset->k_l = 0.0011f;
			offset->k_theta = -0.0025f;
			offset->k_theta_2 = -0.00028f;
			offset->k_l_theta = 0.000002f;
			offset->constant = 0.02f;
		}break;
		case 15:
		{
			offset->k_l = 0.0012f;
			offset->k_theta = -0.0033f;
			offset->k_theta_2 = -0.00032f;
			offset->k_l_theta = 0.000003f;
			offset->constant = 0.03f;
		}break;
		case 14:
		{
			offset->k_l = 0.0014f;
			offset->k_theta = -0.0046f;
			offset->k_theta_2 = -0.00037f;
			offset->k_l_theta = 0.000004f;
			offset->constant = 0.04f;
		}break;
		case 11:
		{
			offset->k_l = 0.0023f;
			offset->k_theta = -0.0139f;
			offset->k_theta_2 = -0.00061f;
			offset->k_l_theta = 0.000012f;
			offset->constant = 0.10f;
		}break;
		case 9:
		{
			offset->k_l = 0.00324f;
			offset->k_theta = -0.0382f;
			offset->k_theta_2 = -0.00091f;
			offset->k_l_theta = 0.000032f;
			offset->constant = 0.34f;
		}break;
		case 7:
		{
			offset->k_l = 0.00477f;
			offset->k_theta = -0.133f;
			offset->k_theta_2 = -0.0012f;
			offset->k_l_theta = 0.0001f;
			offset->constant = 1.33f;
		}break;
		default: break;
	}
	
	float tmp[5];
	tmp[0] = offset->k_l * aim.dist;
	tmp[1] = offset->k_theta * (-GMP.RealAngle);
	tmp[2] = offset->k_theta_2 * (GMP.RealAngle * GMP.RealAngle);
	tmp[3] = offset->k_l_theta * aim.dist * (-GMP.RealAngle);
	tmp[4] = offset->constant;
	
	return (tmp[0]+tmp[1]+tmp[2]+tmp[3]+tmp[4]+PIT_EXTRA_OFFSET);
}

//未识别到装甲板时卡尔曼滤波初始值设定
//Called in AutoAimDataProcess()
void Autoaim_Kalman_Filter_Reset(uint8_t state)
{
	if(state == 0)
	{
		Kalman_Filter_Set_Output(&a_filter, GMY.RealAngle);
		Kalman_Filter_Set_Output(&d1a_filter, 0);
		Kalman_Filter_Set_Output(&d2a_filter, 0);
		Kalman_Filter_Set_Output(&output_filter, GMY.RealAngle);
		Kalman_Filter_Set_Output(&gyro_filter, GMY.RealAngle);
	}
}

//自瞄数据处理
//Called in AutoAim()
void AutoAimDataProcess(void)
{
	offset.yaw = YAW_OFFSET;
	offset.pitch = Autoaim_Pitch_Offset(&pitch_offset, BulletSpeed);
	
	Autoaim_Kalman_Filter_Reset(find_enemy);
	
	if(!aim_finish)
	{
		static float last_yaw = 0;
		static float last_v = 0;
		
		//计算需要预测的时间
		predict_time = DT0 + DT1 + DT2;
		//角度
		aim_real.yaw = GM_RealAngle_RCD.yaw + aim.gimbal_angle.yaw;
		aim_real.yaw = Kalman_Filter(&a_filter,aim_real.yaw);
		//角速度
		cur_v = (float)(aim_real.yaw - last_yaw) * aim.fps / 1000; //单位°/ms
		last_yaw = aim_real.yaw;
		cur_v = Kalman_Filter(&d1a_filter,cur_v);
		//角加速度
		cur_a = (float)(cur_v - last_v) * aim.fps / 1000; //单位°/ms^2
		last_v = cur_v;
		cur_a = Kalman_Filter(&d2a_filter,cur_a);
		//预测角度
		predict_angle[0] = (float)cur_v * predict_time;
		predict_angle[1] = (float)0.5f * cur_a * predict_time * predict_time / 5;
		MINMAX(predict_angle[0],-15,15);
		float final_prediction = aim_real.yaw + (float)cur_v * predict_time;
		final_prediction = Kalman_Filter(&output_filter, final_prediction);
		
		//获得发射窗口参数
		shoot_window.predict_window = - aim.gimbal_angle.yaw / (cur_v * predict_time);
		
		//陀螺中心滤波
		float gyro_center = aim_real.yaw;
		if(aim_mode == ANTI_GYRO_MODE)
		{
			gyro_center = Kalman_Filter(&gyro_filter,gyro_center);
		}
		//计算陀螺周期
		static float last_final= 0;
		float final_diff = aim_real.yaw - last_final;
		last_final = aim_real.yaw;
		if(fabs(final_diff) > thres)
		{
			if (gyro.last_change_time1 < auto_counter_autoaim)
				gyro.last_change_time1 += 5000;
			if (gyro.last_change_time2 < auto_counter_autoaim)
				gyro.last_change_time2 += 5000;
			
			if(gyro.isPeriod1)
			{
				gyro.half_period_1 = (gyro.last_change_time1 - auto_counter_autoaim);
				gyro.isPeriod1 = false;
				gyro.last_change_time1 = auto_counter_autoaim;
			}
			else
			{
				gyro.half_period_2 = (gyro.last_change_time2 - auto_counter_autoaim);
				gyro.isPeriod1 = true;
				gyro.last_change_time2 = auto_counter_autoaim;
			}
		}

		//自瞄输出值
		if(aim_mode == 0)
		{
			AutoAim_Yaw_pid.Reset(&AutoAim_Yaw_pid);
			AutoAim_Pitch_pid.Reset(&AutoAim_Pitch_pid);
		}
		else
		{
			if(aim_mode == PREDICT_MODE)
			{
				#ifdef USE_PREDICT
					aim_output.yaw = PID_PROCESS(&AutoAim_Yaw_pid, final_prediction + adjust.yaw, GM_RealAngle_RCD.yaw);
				#else
					aim_output.yaw = PID_PROCESS(&AutoAim_Yaw_pid, cur_yaw, GM_RealAngle_RCD.yaw);
				#endif
			}
			else if(aim_mode == ANTI_GYRO_MODE)
			{
				aim_output.yaw = PID_PROCESS(&AutoAim_Yaw_pid, gyro_center + adjust.yaw, GM_RealAngle_RCD.yaw);
			}
			aim_output.pitch = PID_PROCESS(&AutoAim_Pitch_pid, adjust.pitch, aim.gimbal_angle.pitch + offset.pitch);
		}
	}
	//自瞄最大输出值限制
	MINMAX(aim_output.yaw, -15.0f, 15.0f);
	MINMAX(aim_output.pitch, -5.0f, 5.0f);
}

//云台控制
//Called in AutoAim()
void AutoAimControl(void)
{
	if(find_enemy)
	{
		GMY.TargetAngle = GM_RealAngle_RCD.yaw + aim_output.yaw;
		GMP.TargetAngle = GM_RealAngle_RCD.pitch + aim_output.pitch;
		aim_finish = 1;
	}
}

//自瞄串口通信发送函数
//Called in AutoAim()
void AutoAimUartTxInfo(void)
{
	uint8_t data[5];
	int16_t imu_yaw_tmp = (int16_t)(imu.yaw / K_ANGLE);
	
	data[0] = ((RefereeData.GameRobotState.robot_id >= 9) ? 1 : 0);  //1打蓝色0打红色  等级
	data[1] = (uint8_t)((imu_yaw_tmp >> 8) & 0xff);
	data[2] = (uint8_t)((imu_yaw_tmp) & 0xff);
	data[3] = 0;
	data[4] = '\n';
	
	freq_div(if(HAL_UART_Transmit_DMA(&AUTOAIM_UART, (uint8_t*)&data, sizeof(data)) != HAL_OK){Error_Handler();}, 500);
}

//自瞄帧率检测
//Called in AutoAim()
void AutoAimFpsCalc(void)
{
	if(auto_counter_autoaim % 500 == 0)
	{
		aim.fps = receive_cnt * 2;
		receive_cnt = 0;
	}
}
	 
//自瞄
//Called in ControlTask.c
void AutoAim(void)
{
	//200ms未收到自瞄数据find_enemy置0
	if(auto_counter_autoaim % 200 == 0 && aim_finish == 1)
	{
		find_enemy = 0;
	}
	
	//自瞄数据处理
	AutoAimDataProcess();
	//串口数据发送
	AutoAimUartTxInfo();
	//自瞄帧率检测
	AutoAimFpsCalc();
	//云台控制
	switch(aim_mode)
	{
		case PREDICT_MODE:
		case ANTI_GYRO_MODE:
		{
			AutoAimControl();
		}break;
	}
}

//自动发射
//Called in FunctionTask.c
int16_t anti_gyro_shootable_bound = ANTI_GYRO_SHOOTABLE_BOUND;
void AutoShoot(uint8_t mode)
{
	//预测模式自动发射判定
	if(aim_mode == PREDICT_MODE)
	{
		//发射窗口条件判定
		if(cur_v >= 0)
		{
			if(aim.gimbal_angle.yaw <= - cur_v * predict_time * PREDICT_SHOOTABLE_BOUND_BELOW && aim.gimbal_angle.yaw >= - cur_v * predict_time * PREDICT_SHOOTABLE_BOUND_ABOVE && aim.fps > 22)
				auto_shoot_flag = 1;
			else
				auto_shoot_flag = 0;
		}
		else
		{
			if(aim.gimbal_angle.yaw >= - cur_v * predict_time * PREDICT_SHOOTABLE_BOUND_BELOW && aim.gimbal_angle.yaw <= - cur_v * predict_time * PREDICT_SHOOTABLE_BOUND_ABOVE && aim.fps > 22)
				auto_shoot_flag = 1;
			else
				auto_shoot_flag = 0;
		}
	}
	//反陀螺模式自动发射判定
	else if(aim_mode == ANTI_GYRO_MODE)
	{
		if(gyro.last_change_time1 < auto_counter_autoaim)
			gyro.last_change_time1 += 2000;
		if(gyro.last_change_time2 < auto_counter_autoaim)
			gyro.last_change_time2 += 2000;
		//判断周期条件
		if(gyro.half_period_1 / gyro.half_period_2 < 1.2f && gyro.half_period_1/gyro.half_period_2 > 0.8f)
		{
			//判断发射窗口条件
			if(fabs(gyro.last_change_time1-auto_counter_autoaim+DT1-(gyro.half_period_1+gyro.half_period_2)*0.375f) < anti_gyro_shootable_bound)
				auto_shoot_flag = 1;
			else if(fabs(gyro.last_change_time2-auto_counter_autoaim+DT1-(gyro.half_period_1+gyro.half_period_2)*0.375f) < anti_gyro_shootable_bound)
				auto_shoot_flag = 1;
			else
				auto_shoot_flag = 0;
		}
		else
			auto_shoot_flag = 0;
	}
	
	//目标静止
	if(fabs(cur_v) < 0.3f && aim_output.yaw < 0.5f)
		auto_shoot_flag = 1;
	
	//pitch到位判定
	if(fabs(aim_output.pitch) > 5.0f)
		auto_shoot_flag = 0;
	
	if(shoot_cd == 0)
	{
		if(find_enemy && auto_shoot_flag == 1)
		{
			ShootOneBullet(BULLET_TYPE);
		}
	}
}

//云台历史角度记录
//Called in ControlTask.c
#define rcd_amt 200
Gimbal_Angle_t GM_RealAngle_Rcd(MotorINFO *GMY, MotorINFO *GMP, int aim_delay_time)
{
	static Gimbal_Angle_t GM_RealAngle_Rcd[rcd_amt];
	static uint8_t i = 0;
	if(GMYReseted == 1 && GMPReseted == 1)
	{
		GM_RealAngle_Rcd[i].yaw = GMY->RealAngle;
		GM_RealAngle_Rcd[i].pitch = GMP->RealAngle;
		i = (i + 1) % rcd_amt;
	}
	return GM_RealAngle_Rcd[(i + rcd_amt - aim_delay_time) % rcd_amt];
}

//陀螺仪历史角速度记录
//Called in ControlTask.c
imu_t imu_w_rcd(imu_t *imu_current, int aim_delay_time)
{
	static imu_t imu_Rcd[rcd_amt] = {0};
	static uint8_t i = 0;
	if(GMYReseted == 1 && GMPReseted == 1)
	{
		imu_Rcd[i].wx = imu_current->wx;
		imu_Rcd[i].wy = imu_current->wy;
		imu_Rcd[i].wz = imu_current->wz;
		i = (i + 1) % rcd_amt;
	}
	return imu_Rcd[(i + rcd_amt - aim_delay_time) % rcd_amt];
}

#endif /*USE_AUTOAIM*/
