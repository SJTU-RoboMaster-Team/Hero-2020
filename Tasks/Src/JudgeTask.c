/**
  ******************************************************************************
  * File Name          : JudgeTask.c
  * Description        : 裁判系统处理任务，得到裁判系统信息
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
#include "offical_Judge_Handler.h"
#include <stdlib.h>
#include <string.h>

static void Referee_Update_GameStatus(void);
static void Referee_Update_RobotHP(void);
static void Referee_Update_EventData(void);
static void Referee_Update_DartStatus(void);
static void Referee_Update_DartRemaining(void);
static void Referee_Update_Warning(void);
static void Referee_Update_RobotState(void);
static void Referee_Update_PowerHeatData(void);
static void Referee_Update_RobotPos(void);
static void Referee_Update_Buff(void);
static void Referee_Update_AerialEnergy(void);
static void Referee_Update_RobotHurt(void);
static void Referee_Update_ShootData(void);
static void Referee_Update_BulletRemaining(void);
static void Referee_Update_RfidStatus(void);
static void Referee_Update_DartClient(void);
//static void Refresh_Client_Data(void);
//static void Referee_Transmit_UserData(void);
static void Client_Graph_Process(void);
static void Refree_Update_RobotControlData(void);

uint8_t JUDGE_Received = 0;
JudgeState_e JUDGE_State = OFFLINE;
referee_data_t RefereeData;
client_custom_data_t custom_data;
graphic_data_struct_t client_graph;
Pack* pack;

uint16_t maxHeat0 = MAXHEAT03;
uint16_t maxHeat1 = MAXHEAT03;
float fakeHeat0 = 0;
float fakeHeat1 = 0;
float cooldown0 = COOLDOWN03;
float cooldown1 = COOLDOWN13;
robot_status_t cur_robot_status;

int8_t client_graphic_steps = -9;
uint8_t client_graphic_busy = 0;

int16_t auto_counter_radarct = 0;
uint8_t control_radar_state = 0;

uint8_t tmp_judge;
void InitJudgeUart(void){
	//Referee_Transmit();
	srand((unsigned)rc_cnt);
	if(HAL_UART_Receive_DMA(&JUDGE_UART, &tmp_judge, 1) != HAL_OK){
			Error_Handler();
	}
	tx_free = 0;
}

void DivideBit_int(int32_t a){
	client_graph.radius=a&0x3ff;      //取前十位
	client_graph.end_x=(a>>10)&0x7ff; //取中间11位
	client_graph.end_y=(a>>21)&0x7ff;	//取最后十一位
}

void DivideBit_float(float a){
	pack = (Pack*)(&a);
	client_graph.radius = pack->radius;
	client_graph.end_x  = pack->end_x;
	client_graph.end_y  = pack->end_y;	
}

uint8_t receiving = 0;
uint8_t received = 0;
uint8_t buffer[80] = {0}; 
uint8_t buffercnt = 0;
uint16_t cmdID;
void judgeUartRxCpltCallback(void)
{
	if(receiving) 
	{
		if(buffercnt > 40)buffercnt = 4;
		buffer[buffercnt] = tmp_judge;
		buffercnt++;
		
		if(buffercnt == 5)
		{
			if (myVerify_CRC8_Check_Sum(buffer, 5)==0) 
			{
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 7) cmdID = (jointbyte(buffer[6], buffer[5]));
	
		if(buffercnt == 12 && cmdID == 0x0001)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 12))
			{
				Referee_Update_GameStatus();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 10 && cmdID == 0x0002)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 10))
			{
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 41 && cmdID == 0x0003)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 41))
			{
				Referee_Update_RobotHP();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 12 && cmdID == 0x0004)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 12))
			{
				Referee_Update_DartStatus();//新增
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
			
		if(buffercnt == 13 && cmdID == 0x0101)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 13))
			{
				Referee_Update_EventData();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 13 && cmdID == 0x0102)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 13))
			{
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		
		if(buffercnt == 11 && cmdID == 0x0103)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 11))
			{
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 11 && cmdID == 0x0104)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 11))
			{
				Referee_Update_Warning();
				receiving = 0;
				buffercnt = 0;
			}
		}

		if(buffercnt == 10 && cmdID == 0x0105)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 10))
			{
				Referee_Update_DartRemaining();//新增
				Referee_Update_Warning();
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 27 && cmdID == 0x0201)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 27))
			{
				Referee_Update_RobotState();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 25 && cmdID == 0x0202)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 25))
			{
				Referee_Update_PowerHeatData();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}

		if(buffercnt == 25 && cmdID == 0x0203)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 25))
			{
				Referee_Update_RobotPos();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 10 && cmdID == 0x0204)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 10))
			{
				Referee_Update_Buff();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
		
		if(buffercnt == 12 && cmdID == 0x0205)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 12))
			{
				Referee_Update_AerialEnergy();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}

		if(buffercnt == 10 && cmdID == 0x0206)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 10)) 
			{
				Referee_Update_RobotHurt();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}

		if(buffercnt == 15 && cmdID == 0x0207)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 15))
			{
				Referee_Update_ShootData();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}	
		}
		
		if(buffercnt == 11 && cmdID == 0x0208)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 11))
			{
				Referee_Update_BulletRemaining();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}	
		}
		
		if(buffercnt == 13 && cmdID == 0x0209)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 13))
			{
				Referee_Update_RfidStatus();//新增
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}	
		}
		
		if(buffercnt == 21 && cmdID == 0x020A)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 21))
			{
				Referee_Update_DartClient();//新增
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}	
		}
		
		if(buffercnt == 25 && cmdID == 0x0301)
		{
			if (myVerify_CRC16_Check_Sum(buffer, 25))
			{
			  Refree_Update_RobotControlData();
				JUDGE_Received = 1;
				receiving = 0;
				buffercnt = 0;
			}
		}
	}

	else 
	{
		if(tmp_judge == 0xA5)
		{
			receiving = 1;
			buffercnt = 0;
			buffer[0] = tmp_judge;
			buffercnt++;
		}
	}
	if(HAL_UART_Receive_DMA(&JUDGE_UART, &tmp_judge, 1) != HAL_OK)
	{
		Error_Handler();
	}
}

void getJudgeState(void)
{
	static int s_count_judge = 0;
	if(JUDGE_Received==1)
	{
		s_count_judge = 0;
		JUDGE_State = ONLINE;
		JUDGE_Received = 0;
	}
	else
	{
		s_count_judge++;
		if(s_count_judge > 300)//300ms
		{
			JUDGE_State = OFFLINE;
		}
	}
}

void Referee_Update_GameStatus()
{
	RefereeData.GameStatus.game_type = (uint8_t)buffer[7] & 0x0f;
	RefereeData.GameStatus.game_progress = (uint8_t)(buffer[7] >> 4) & 0x0f;
	uint8_t* gd2 = (uint8_t*)&RefereeData.GameStatus.stage_remain_time;
	int8_t tmp1[2] = {buffer[8],buffer[9]};
	for(int i = 0; i<2; i++){gd2[i] = (uint8_t)tmp1[i];}
}

void Referee_Update_RobotHP()
{
	RefereeData.RobotHP.red_1_robot_HP = jointbyte(buffer[8],buffer[7]);
	RefereeData.RobotHP.red_2_robot_HP = jointbyte(buffer[10],buffer[9]);
	RefereeData.RobotHP.red_3_robot_HP = jointbyte(buffer[12],buffer[11]);
	RefereeData.RobotHP.red_4_robot_HP = jointbyte(buffer[14],buffer[13]);
	RefereeData.RobotHP.red_5_robot_HP = jointbyte(buffer[16],buffer[15]);
	RefereeData.RobotHP.red_7_robot_HP = jointbyte(buffer[18],buffer[17]);
	RefereeData.RobotHP.red_outpost_HP = jointbyte(buffer[20],buffer[19]);
	RefereeData.RobotHP.red_base_HP = jointbyte(buffer[22],buffer[21]);
	RefereeData.RobotHP.blue_1_robot_HP = jointbyte(buffer[24],buffer[23]);
	RefereeData.RobotHP.blue_2_robot_HP = jointbyte(buffer[26],buffer[25]);
	RefereeData.RobotHP.blue_3_robot_HP = jointbyte(buffer[28],buffer[27]);
	RefereeData.RobotHP.blue_4_robot_HP = jointbyte(buffer[30],buffer[29]);
	RefereeData.RobotHP.blue_5_robot_HP = jointbyte(buffer[32],buffer[31]);
	RefereeData.RobotHP.blue_7_robot_HP = jointbyte(buffer[34],buffer[33]);
	RefereeData.RobotHP.blue_outpost_HP = jointbyte(buffer[36],buffer[35]);
	RefereeData.RobotHP.blue_base_HP = jointbyte(buffer[38],buffer[37]);
}

void Referee_Update_EventData()
{
	
}

void Referee_Update_DartStatus()
{
	
}

void Referee_Update_DartRemaining()
{
	
}

void Referee_Update_Warning()
{
	RefereeData.refereeWarning.level = (uint8_t)buffer[7];
	RefereeData.refereeWarning.foul_robot_id = (uint8_t)buffer[8];
}

void Referee_Update_RobotState()
{
	uint8_t* grs0 = (uint8_t*)&RefereeData.GameRobotState.robot_id;
	int8_t tmp0[1] = {buffer[7]};
	grs0[0] = (uint8_t)tmp0[0];
	
	uint8_t* grs1 = (uint8_t*)&RefereeData.GameRobotState.robot_level;
	int8_t tmp1[1] = {buffer[8]};
	grs1[0] = (uint8_t)tmp1[0];
	
	uint8_t* grs2 = (uint8_t*)&RefereeData.GameRobotState.remain_HP;
	int8_t tmp2[2] = {buffer[9],buffer[10]};
	for(int i = 0; i<2; i++){
		grs2[i] = (uint8_t)tmp2[i];
	}
	
	uint8_t* grs3 = (uint8_t*)&RefereeData.GameRobotState.max_HP;
	int8_t tmp3[2] = {buffer[11], buffer[12]};
	for(int i = 0; i<2; i++){
		grs3[i] = (uint8_t)tmp3[i];
	}
	
	uint8_t* grs4 = (uint8_t*)&RefereeData.GameRobotState.shooter_heat0_cooling_rate;
	int8_t tmp4[2] = {buffer[13], buffer[14]};
	for(int i = 0; i<2; i++){
		grs4[i] = (uint8_t)tmp4[i];
	}
	
	uint8_t* grs5 = (uint8_t*)&RefereeData.GameRobotState.shooter_heat0_cooling_limit;
	int8_t tmp5[2] = {buffer[15], buffer[16]};
	for(int i = 0; i<2; i++){
		grs5[i] = (uint8_t)tmp5[i];
	}
	
	uint8_t* grs6 = (uint8_t*)&RefereeData.GameRobotState.shooter_heat1_cooling_rate;
	int8_t tmp6[2] = {buffer[17], buffer[18]};
	for(int i = 0; i<2; i++){
		grs6[i] = (uint8_t)tmp6[i];
	}	
	
	uint8_t* grs7 = (uint8_t*)&RefereeData.GameRobotState.shooter_heat1_cooling_limit;
	int8_t tmp7[2] = {buffer[19], buffer[20]};
	for(int i = 0; i<2; i++){
		grs7[i] = (uint8_t)tmp7[i];
	}
	
	uint8_t* grs10 = (uint8_t*)&RefereeData.GameRobotState.max_chassis_power;
	int8_t tmp10 = buffer[23];
	grs10[0] = (uint8_t)tmp10;
}

void Referee_Update_PowerHeatData()
{
	uint8_t* ph0 = (uint8_t*)&RefereeData.PowerHeat.chassis_volt;
	for(int i = 0; i<2; i++){
	ph0[i] = (uint8_t)buffer[i+7];
	}
	
	uint8_t* ph1 = (uint8_t*)&RefereeData.PowerHeat.chassis_current;
	for(int i = 0; i<2; i++){
	ph1[i] = (uint8_t)buffer[i+9];
	}
	
	uint8_t* ph2 = (uint8_t*)&RefereeData.PowerHeat.chassis_power;
	for(int i = 0; i<4; i++){
	ph2[i] = (uint8_t)buffer[i+11];
	}
	
	uint8_t * ph3 = (uint8_t*)&RefereeData.PowerHeat.chassis_power_buffer;
	for(int i = 0; i<2; i++){
	ph3[i] = (uint8_t)buffer[i+15];
	}
	
	uint8_t * ph4 = (uint8_t*)&RefereeData.PowerHeat.shooter_heat0;
	for(int i = 0; i<2; i++){
	ph4[i] = (uint8_t)buffer[i+17];
	}
	
	uint8_t * ph5 = (uint8_t*)&RefereeData.PowerHeat.shooter_heat1;
	for(int i = 0; i<2; i++){
	ph5[i] = (uint8_t)buffer[i+19];
	}
	
	uint8_t * ph6 = (uint8_t*)&RefereeData.PowerHeat.mobile_shooter_heat2;
	for(int i = 0; i<2; i++){
	ph6[i] = (uint8_t)buffer[i+21];
	}
}

void Referee_Update_RobotPos()
{
	uint8_t * rp0 = (uint8_t*)&RefereeData.RobotPos.x;
	for(int i = 0; i<4; i++){
	rp0[i] = (uint8_t)buffer[i+7];
	}
	
	uint8_t * rp1 = (uint8_t*)&RefereeData.RobotPos.y;
	for(int i = 0; i<4; i++){
	rp1[i] = (uint8_t)buffer[i+11];
	}
	
	uint8_t * rp2 = (uint8_t*)&RefereeData.RobotPos.z;
	for(int i = 0; i<4; i++){
	rp2[i] = (uint8_t)buffer[i+15];
	}
	
	uint8_t * rp3 = (uint8_t*)&RefereeData.RobotPos.yaw;
	for(int i = 0; i<4; i++){
	rp3[i] = (uint8_t)buffer[i+19];
	}
}

void Referee_Update_Buff()
{
	RefereeData.Buff.power_rune_buff = (uint8_t)buffer[7];
}

void Referee_Update_AerialEnergy()
{
	uint8_t * ae0 = (uint8_t*)&RefereeData.ShootData.bullet_type;
	ae0[0] = (uint8_t)buffer[7];
	
	uint8_t * ae1 = (uint8_t*)&RefereeData.ShootData.bullet_type;
	for(int i = 0; i<2; i++){
	ae1[i] = (uint8_t)buffer[i+8];
	}
}

void Referee_Update_RobotHurt()
{
	RefereeData.RobotHurt.armor_id = (uint8_t)(buffer[7] & 0xf);
	RefereeData.RobotHurt.hurt_type = (uint8_t)((buffer[7] >> 4) & 0xf);
}

void Referee_Update_ShootData()
{
	uint8_t * sd0 = (uint8_t*)&RefereeData.ShootData.bullet_type;
	sd0[0] = (uint8_t)buffer[7];
	
	uint8_t * sd1 = (uint8_t*)&RefereeData.ShootData.bullet_freq;
	sd1[0] = (uint8_t)buffer[8];
	
	uint8_t * sd2 = (uint8_t*)&RefereeData.ShootData.bullet_speed;
	for(int i = 0; i<4; i++){
	sd2[i] = (uint8_t)buffer[i+9];
	}
}

void Referee_Update_BulletRemaining()
{
	RefereeData.bulletRemaining.bullet_remaining_num = jointbyte(buffer[8],buffer[7]);
}

void Referee_Update_RfidStatus()
{
	
}

void Referee_Update_DartClient()
{
	
}
	
void Refree_Update_RobotControlData()
{
	
}



uint8_t check_buffer[60] = {0};
/*void Refresh_Client_Data()
{
}
*/
/*
void Referee_Transmit_UserData()
{
	Refresh_Client_Data();
	
	uint8_t buffer[28] = {0};
	uint8_t * ud1 = (uint8_t*)&custom_data.data1;
	uint8_t * ud2 = (uint8_t*)&custom_data.data2;
	uint8_t * ud3 = (uint8_t*)&custom_data.data3;
	
	//帧头
	buffer[0] = 0xA5;//数据帧起始字节，固定值为 0xA5
	buffer[1] = 19;//数据帧中 data 的长度,占两个字节
	buffer[2] = 0;
	buffer[3] = 1;//包序号
	buffer[4] = myGet_CRC8_Check_Sum(&buffer[0], 5-1, myCRC8_INIT);//帧头 CRC8 校验
	//cmd ID: 0x0301
	buffer[5] = 0x01;
	buffer[6] = 0x03;
	//数据的内容 ID:0xD180  ,占两个字节
	buffer[7] = 0x80;
	buffer[8] = 0xD1;
	//发送者的 ID, 占两个字节
	buffer[9] = RefereeData.GameRobotState.robot_id;
	buffer[10] = 0;
	//客户端的 ID, 只能为发送者机器人对应的客户端,  占两个字节
	buffer[11] = RefereeData.GameRobotState.robot_id;
	if(buffer[11]>9&&buffer[11]<16)buffer[11]+=6;
	buffer[12] = 0x01;
	//自定义数据
	for(int i=13;i<17;i++) buffer[i] = ud1[i-13];
	for(int i=17;i<21;i++) buffer[i] = ud2[i-17];
	for(int i=21;i<25;i++) buffer[i] = ud3[i-21];
	buffer[25] = custom_data.masks;
	//CRC16校验位，两个字节
	Append_CRC16_Check_Sum(buffer,sizeof(buffer));
	
	for(int i = 0; i < 28; i++)
	{
		check_buffer[i] = buffer[i];
	}

	HAL_UART_Transmit(&JUDGE_UART,(uint8_t *)&buffer,sizeof(buffer),0xff);
}*/

uint8_t debug_buffer[30]={0};
void Referee_Transmit_ClientGraph()
{
	Client_Graph_Process();
	
	uint8_t buffer[30] = {0};
	
	
	//帧头
	buffer[0] = 0xA5;//数据帧起始字节，固定值为 0xA5
	buffer[1] = 21;//数据帧中 data 的长度,占两个字节
	buffer[2] = 0;
	buffer[3] = 1;//包序号
	buffer[4] = myGet_CRC8_Check_Sum(&buffer[0], 5-1, myCRC8_INIT);//帧头 CRC8 校验
	//cmd ID: 0x0301
	buffer[5] = 0x01;
	buffer[6] = 0x03;
	//数据的内容 ID:0x0101  ,占两个字节
	buffer[7] = 0x01;
	buffer[8] = 0x01;
	//发送者的 ID, 占两个字节
	buffer[9] = RefereeData.GameRobotState.robot_id;
	buffer[10] = 0;
	//客户端的 ID, 只能为发送者机器人对应的客户端,  占两个字节
	buffer[11] = (uint16_t)RefereeData.GameRobotState.robot_id;
	buffer[12] = 0x01;
	//自定义图形数据
	buffer[13] = client_graph.graphic_name[0];
	buffer[14] = client_graph.graphic_name[1];
	buffer[15] = client_graph.graphic_name[2];
	
	
	buffer[16] = ((uint8_t)(client_graph.operate_type)|(uint8_t)(client_graph.graphic_type<<3)|(uint8_t)(client_graph.layer<<6));
//	client_graph.operate_type = 0;
	buffer[17] = ((uint8_t)(client_graph.layer>>2)|(uint8_t)(client_graph.color<<2)|(uint16_t)(client_graph.start_angle<<6));
	buffer[18] = ((uint16_t)(client_graph.start_angle>>2)|(uint16_t)(client_graph.end_angle<<7));
	buffer[19] = ((uint16_t)(client_graph.end_angle>>1));
	
	buffer[20] = ((uint16_t)(client_graph.width));
	buffer[21] = ((uint16_t)(client_graph.width>>8)|(uint16_t)(client_graph.start_x<<2));
	buffer[22] = ((uint16_t)(client_graph.start_x>>6)|(uint16_t)(client_graph.start_y<<5));
	buffer[23] = ((uint16_t)(client_graph.start_y>>3));
	
	buffer[24] = ((uint32_t)(client_graph.radius));
	buffer[25] = ((uint32_t)(client_graph.radius>>8)|(uint32_t)(client_graph.end_x<<2));
	buffer[26] = ((uint32_t)(client_graph.end_x>>6)|(uint32_t)(client_graph.end_y<<5));
	buffer[27] = ((uint32_t)(client_graph.end_y>>3));
	for(int i = 0; i < sizeof(buffer)-2; i++)
	{
		debug_buffer[i] = buffer[i];
	}

	Append_CRC16_Check_Sum(buffer,sizeof(buffer));
	
	for(int i = 0; i < sizeof(buffer); i++)
	{
		check_buffer[i] = buffer[i];
	}

	HAL_UART_Transmit(&JUDGE_UART,(uint8_t *)&buffer,sizeof(buffer),0xff);
	client_graphic_busy = 0;
}

char Referee_Transmit_Char[30]={0};
void Referee_Transmit_ClientChar()
{
	Client_Graph_Process();
	
	uint8_t buffer[60] = {0};
	
	
	//帧头
	buffer[0] = 0xA5;//数据帧起始字节，固定值为 0xA5
	buffer[1] = 51;//数据帧中 data 的长度,占两个字节
	buffer[2] = 0;
	buffer[3] = 1;//包序号
	buffer[4] = myGet_CRC8_Check_Sum(&buffer[0], 5-1, myCRC8_INIT);//帧头 CRC8 校验
	//cmd ID: 0x0301
	buffer[5] = 0x01;
	buffer[6] = 0x03;
	//数据的内容 ID:0x0110  ,占两个字节
	buffer[7] = 0x10;
	buffer[8] = 0x01;
	//发送者的 ID, 占两个字节
	buffer[9] = RefereeData.GameRobotState.robot_id;
	buffer[10] = 0;
	//客户端的 ID, 只能为发送者机器人对应的客户端,  占两个字节
	buffer[11] = (uint16_t)RefereeData.GameRobotState.robot_id;
	buffer[12] = 0x01;
	//自定义图形数据
	buffer[13] = client_graph.graphic_name[0];
	buffer[14] = client_graph.graphic_name[1];
	buffer[15] = client_graph.graphic_name[2];
	
	//strcpy(Referee_Transmit_Char,"Can't you show this?");//测试用字符串
	client_graph.end_angle=sizeof(Referee_Transmit_Char);//字符串长度
	buffer[16] = ((uint8_t)(client_graph.operate_type)|(uint8_t)(client_graph.graphic_type<<3)|(uint8_t)(client_graph.layer<<6));
	buffer[17] = ((uint8_t)(client_graph.layer>>2)|(uint8_t)(client_graph.color<<2)|(uint16_t)(client_graph.start_angle<<6));
	buffer[18] = ((uint16_t)(client_graph.start_angle>>2)|(uint16_t)(client_graph.end_angle<<7));
	buffer[19] = ((uint16_t)(client_graph.end_angle>>1));
	
	buffer[20] = ((uint16_t)(client_graph.width));
	buffer[21] = ((uint16_t)(client_graph.width>>8)|(uint16_t)(client_graph.start_x<<2));
	buffer[22] = ((uint16_t)(client_graph.start_x>>6)|(uint16_t)(client_graph.start_y<<5));
	buffer[23] = ((uint16_t)(client_graph.start_y>>3));
	
	buffer[24] = ((uint8_t)(client_graph.radius));
	buffer[25] = ((uint16_t)(client_graph.radius>>8)|(uint16_t)(client_graph.end_x<<2));
	buffer[26] = ((uint16_t)(client_graph.end_x>>6)|(uint16_t)(client_graph.end_y<<5));
	buffer[27] = ((uint16_t)(client_graph.end_y>>3));
	
	
	
	for(int i = 0 ; i < 30 ; i++)
	{
		buffer[i+28]=Referee_Transmit_Char[i];
	}
	
	Append_CRC16_Check_Sum(buffer,sizeof(buffer));
	
	for(int i = 0; i < sizeof(buffer); i++)
	{
		check_buffer[i] = buffer[i];
	}

	HAL_UART_Transmit(&JUDGE_UART,(uint8_t *)&buffer,sizeof(buffer),0xff);
	client_graphic_busy = 0;

}

void Referee_Transmit(void)
{
	if(client_graphic_steps > 0)
		{
		Referee_Transmit_ClientGraph();
	  }
	else
    {
		Referee_Transmit_ClientGraph();
	  //Referee_Transmit_ClientChar();
	  }
}

void fakeHeatCalc(void)
{
	if(JUDGE_State == ONLINE)
	{
		maxHeat0 = RefereeData.GameRobotState.shooter_heat0_cooling_limit;
		cooldown0 = RefereeData.GameRobotState.shooter_heat0_cooling_rate;
		maxHeat1 = RefereeData.GameRobotState.shooter_heat1_cooling_limit;
		cooldown1 = RefereeData.GameRobotState.shooter_heat1_cooling_rate;
		if(auto_counter_heat0==0)
		{
			fakeHeat0 = RefereeData.PowerHeat.shooter_heat0;
		}
		if(auto_counter_heat1==0)
		{
			fakeHeat1 = RefereeData.PowerHeat.shooter_heat1;
		}
	}
	else
	{
		maxHeat0 = MAXHEAT03;
		cooldown0 = COOLDOWN03;
		maxHeat1 = MAXHEAT13;
		cooldown1 = COOLDOWN13;
	}
	
	if(fakeHeat0 >= cooldown0/10) fakeHeat0 -= cooldown0/10;
	else fakeHeat0 = 0;
	if(fakeHeat1 >= cooldown1/10) fakeHeat1 -= cooldown1/10;
	else fakeHeat1 = 0;
}


void Client_Graph_Process(void)
{
	if(client_graphic_busy)
		return;
	if(client_graphic_steps<0)
	{
		client_graph.operate_type = 1;
		client_graph.graphic_type = 0;
	  client_graph.color = 0;
	  client_graph.width = 0;
		client_graph.start_x = 0;
		client_graph.start_y = 0;
		client_graph.radius = 0;
		client_graph.end_x = 0;
		client_graph.end_y = 0;
		client_graph.start_angle = 0;
		client_graph.end_angle = 0;
	}
	switch(client_graphic_steps)
	{
		case -9:                         //全部初始化至空图层，此后只需要使用修改指令（operate_type=2）而非新增指令
		{
			client_graph.layer = 9;
			client_graphic_steps++;
		}break;
		case -8:
		{
		  client_graph.layer = 8;
			client_graphic_steps++;
		}break;
		case -7:
		{
			client_graph.layer = 7;
			client_graphic_steps++;
		}break;
		case -6:
		{
			client_graph.layer = 6;
			client_graphic_steps++;
		}break;
		case -5:
		{
			client_graph.layer = 5;
			client_graphic_steps++;
		}break;
		case -4:
		{
			client_graph.layer = 4;
			client_graphic_steps++;
		}break;
		case -3:
		{
			client_graph.layer = 3;
			client_graphic_steps++;
		}break;
		case -2:
		{
			client_graph.layer = 2;
			client_graphic_steps++;
		}break;
		case -1:
		{
			client_graph.layer = 1;
			client_graphic_steps++;
		}break;
		case 0:
		{
			client_graph.layer = 0;
			client_graphic_steps++;
		}break;
		case 1:                  //目前是测试用图层           
		{
			client_graph.layer = 0;//case 1 对应图层0 后面依次类推
			client_graph.operate_type = 2;
			client_graph.graphic_type = 5;
	    client_graph.color = 0;
	    client_graph.width = 5;
			client_graph.start_x += channelrrow*0.1;
			client_graph.start_y += channelrcol*0.1;
//			client_graph.radius = 23333;
//			client_graph.end_x = 0;
//			client_graph.end_y = 0;
			DivideBit_float(3.58);
			client_graph.start_angle = 50;
			client_graph.end_angle = 4;
		}break;
		case 2:                  //目前是测试用图层           
		{
			client_graph.layer = 0;//case 2 对应图层0 后面依次类推
			client_graph.operate_type = 2;
			client_graph.graphic_type = 5;
	    client_graph.color = 0;
	    client_graph.width = 5;
			client_graph.start_x += channelrrow*0.1;
			client_graph.start_y += channelrcol*0.1;
//			client_graph.radius = 23333;
//			client_graph.end_x = 0;
//			client_graph.end_y = 0;
			DivideBit_float(3.58);
			client_graph.start_angle = 50;
			client_graph.end_angle = 4;
		}break;
		default: break;
	}
	
	for(int i = 0; i < 3; i++)
	{
		client_graph.graphic_name[i] = i;
	}
	
	client_graphic_busy = 1;
}

void Client_Graph_Start(void)
{
	if(client_graphic_steps == 0)
	{
		client_graphic_steps = 1;
	}
}

void Client_Graph_Clear(void)
{
	client_graphic_steps = 1;
	client_graphic_busy = 1;
	
	client_graph.operate_type = 5;
	client_graph.graphic_type = 0;
	client_graph.layer = 0;
	for(int i = 0; i < 5; i++)
	{
		client_graph.graphic_name[i] = client_graphic_steps;
	}
	client_graph.start_x = 0;
	client_graph.start_y = 0;
	client_graph.radius = 0;
	client_graph.end_x = 0;
	client_graph.end_y = 0;
	client_graph.start_angle = 0;
	client_graph.end_angle = 0;
}

//键鼠信息传递  用于测试
void Referee_Transmit_KeyBoard(uint8_t *pData)
{
	uint8_t buffer[30] = {0};
	
	//帧头
	buffer[0] = 0xA5;//数据帧起始字节，固定值为 0xA5
	buffer[1] = 16;//数据帧中 data 的长度,占两个字节
	buffer[2] = 0;
	buffer[3] = 1;//包序号
	buffer[4] = myGet_CRC8_Check_Sum(&buffer[0], 5-1, myCRC8_INIT);//帧头 CRC8 校验
	//cmd ID: 0x0301
	buffer[5] = 0x01;
	buffer[6] = 0x03;
	//数据的内容 ID:0x0202  ,占两个字节
	buffer[7] = 0x02;
	buffer[8] = 0x02;
	//发送者的 ID, 占两个字节
	buffer[9] = RefereeData.GameRobotState.robot_id;
	buffer[10] = 0;
	//接受者ID 雷达站0x0009
	buffer[11] = 0x09;
	buffer[12] = 0x00;
	for(int i = 0;i < sizeof(pData); i++)
	{
		buffer[i+13] = pData[i];
	}
	
	Append_CRC16_Check_Sum(buffer,sizeof(buffer));
	
	for(int i = 0; i < sizeof(buffer); i++)
	{
		check_buffer[i] = buffer[i];
	}
	
	HAL_UART_Transmit(&JUDGE_UART,(uint8_t *)&buffer,sizeof(buffer),0xff);
}

//发送信息给雷达  用于测试
void Control_radar(void)
{
	if(control_radar_state)
	{
		if(auto_counter_radarct == 0)
		{
			Referee_Transmit_KeyBoard(KeyBoarddata);
			auto_counter_radarct = 25;
		}	
	}
}
