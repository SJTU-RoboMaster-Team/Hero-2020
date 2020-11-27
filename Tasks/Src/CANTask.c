/**
  ******************************************************************************
  * File Name          : CANTask.c
  * Description        : CAN通信任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

#define CanRxGetU16(canRxMsg, num) (((uint16_t)canRxMsg.Data[num * 2] << 8) | (uint16_t)canRxMsg.Data[num * 2 + 1])
uint8_t isRcan1Started = 0, isRcan2Started = 0;
uint8_t isCan11FirstRx = 0, isCan12FirstRx = 0, isCan21FirstRx = 0, isCan22FirstRx = 0;
CanRxMsgTypeDef Can1RxMsg,Can2RxMsg;
ESCC6x0RxMsg_t CMFLRx,CMBLRx,CMFRRx,CMBRRx;
uint8_t can1_update = 1;
uint8_t can1_type = 1;
uint8_t can2_update = 1;
uint8_t can2_type = 1;

/********************CAN发送*****************************/
//CAN数据标记发送，保证发送资源正常
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if(hcan == &hcan1){
		can1_update = 1;
	}
	else if(hcan == &hcan2)
	{
		can2_update = 1;
	}
}

/********************CAN******************************/
void InitCanReception()
{
	#ifndef CAN11
	isCan11FirstRx = 1;
	#endif
	#ifndef CAN12
	isCan12FirstRx = 1;
	#endif
	#ifndef CAN21
	isCan21FirstRx = 1;
	#endif
	#ifndef CAN22
	isCan22FirstRx = 1;
	#endif
	
	can1_type=1;
	#ifdef CAN12
		can1_type=2;
	#endif
	#ifdef DOUBLE_BOARD_CAN1
		can1_type=3;
	#endif
	
	can2_type=1;
	#ifdef CAN22
		can2_type=2;
	#endif
	#ifdef DOUBLE_BOARD_CAN2
		can2_type=3;
	#endif
	
	//http://www.eeworld.com.cn/mcu/article_2016122732674_3.html
	hcan1.pRxMsg = &Can1RxMsg;
	/*##-- Configure the CAN1 Filter ###########################################*/
	CAN_FilterConfTypeDef  sFilterConfig;
	sFilterConfig.FilterNumber = 0;//14 - 27//14
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 14;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
	if(HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK){
		Error_Handler(); 
	}
	isRcan1Started = 1;
	
	hcan2.pRxMsg = &Can2RxMsg;
	/*##-- Configure the CAN2 Filter ###########################################*/
	CAN_FilterConfTypeDef sFilterConfig2;
	sFilterConfig2.FilterNumber = 14;//14 - 27//14
	sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig2.FilterIdHigh = 0x0000;
	sFilterConfig2.FilterIdLow = 0x0000;
	sFilterConfig2.FilterMaskIdHigh = 0x0000;
	sFilterConfig2.FilterMaskIdLow = 0x0000;
	sFilterConfig2.FilterFIFOAssignment = 0;
	sFilterConfig2.FilterActivation = ENABLE;
	sFilterConfig2.BankNumber = 14;
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig2);
	if(HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK){
		Error_Handler(); 
	}
	isRcan2Started = 1;
}

//CAN接收中断入口函数
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	uint8_t flag = 0;
	if(hcan == &hcan1)	//CAN1数据
	{
		for(int i=0;i<8;i++)
		{
			if(can1[i]==0)continue;
			if(Can1RxMsg.StdId==can1[i]->RXID)
			{
				if(i<4) isCan11FirstRx = 1;
				else if(i<8) isCan12FirstRx = 1;
				flag=1;
				switch(can1[i]->ESCtype)
				{
					case ESC_C6x0:
					{
						can1[i]->RxMsgC6x0.angle		 = CanRxGetU16(Can1RxMsg, 0);
						can1[i]->RxMsgC6x0.RotateSpeed   = CanRxGetU16(Can1RxMsg, 1);
						can1[i]->RxMsgC6x0.moment		 = CanRxGetU16(Can1RxMsg, 2);
						//云台编码器防越界处理
						if(can1[i]==&GMY)
						{
							//改动该角度时需同时改动底盘掉头程序

								if(can1[i]->RxMsgC6x0.angle > (GM_YAW_ZERO + 2560))
									can1[i]->RxMsgC6x0.angle -= 8192;
								else if(can1[i]->RxMsgC6x0.angle < (GM_YAW_ZERO - 5632))
									can1[i]->RxMsgC6x0.angle += 8192;
						}
						if(can1[i]==&GMP)
						{
							if(can1[i]->RxMsgC6x0.angle > (GM_PITCH_ZERO + 4096))
								can1[i]->RxMsgC6x0.angle -= 8192;
							else if(can1[i]->RxMsgC6x0.angle < (GM_PITCH_ZERO - 4096))
								can1[i]->RxMsgC6x0.angle += 8192;
						}
					}
					case ESC_6623:
					{
						can1[i]->RxMsg6623.angle		 = CanRxGetU16(Can1RxMsg, 0);
						can1[i]->RxMsg6623.realIntensity = CanRxGetU16(Can1RxMsg, 1);
						can1[i]->RxMsg6623.giveIntensity = CanRxGetU16(Can1RxMsg, 2);
					}
				}
			}
		}
		if(Can1RxMsg.StdId==0x200 || Can1RxMsg.StdId==0x1FF)
			flag=1;
		else if(Can1RxMsg.StdId==0x300)
		{
			flag=1;
			#if defined (BOARD_SLAVE) && defined (DOUBLE_BOARD_CAN1)
			switch(Can1RxMsg.Data[0])
			{
				case 0xff: RxWorkState=STOP_STATE; break;
				case 0x00: RxWorkState=PREPARE_STATE; break;
				case 0x01: RxWorkState=NORMAL_STATE; break;
				case 0x02: RxWorkState=ADDITIONAL_STATE_ONE; break;
				case 0x03: RxWorkState=ADDITIONAL_STATE_TWO; break;
				default: RxWorkState=STOP_STATE; break;
			}
			if(RxWorkState == STOP_STATE || RxWorkState == PREPARE_STATE) WorkState = RxWorkState;
			switch(Can1RxMsg.Data[1])
			{
				case 0xff:
				{
					if(Cap_Get_Cap_State() != CAP_STATE_STOP)
					{
						Cap_State_Switch(CAP_STATE_STOP);
					}
				}break;
				case 0x00:
				{
					if(Cap_Get_Cap_State() != CAP_STATE_RECHARGE)
					{
						Cap_State_Switch(CAP_STATE_RECHARGE);
					}
				}break;
				case 0x01:
				{
					if(Cap_Get_Cap_State() != CAP_STATE_RELEASE)
					{
						Cap_State_Switch(CAP_STATE_RELEASE);
					}
				}break;
				default: break;
			}
			
			cap_move_state = Can1RxMsg.Data[2];
			#endif
		}
		else if(Can1RxMsg.StdId==0x301)
		{
			flag = 1;
			#if defined (BOARD_MAIN) && defined (DOUBLE_BOARD_CAN1)
			rx_power_voltage = Can1RxMsg.Data[0] / 5;
			rx_power_current = (double)(Can1RxMsg.Data[1] * 256 + Can1RxMsg.Data[2]) / rx_power_voltage;
			rx_cap_voltage = Can1RxMsg.Data[3] / 5;
			#endif
		}
		if(!flag)
		{
			Error_Handler();
		}
		if(HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK){
			isRcan1Started = 0;
		}else{
			isRcan1Started = 1;
		}
	}
	else if(hcan == &hcan2)//CAN2数据
	{
		for(int i=0;i<8;i++)
		{
			if(can2[i]==0)continue;
			if(Can2RxMsg.StdId==can2[i]->RXID)
			{
				if(i<4) isCan21FirstRx = 1;
				else if(i<8) isCan22FirstRx = 1;
				flag=1;
				switch(can2[i]->ESCtype)
				{
					case ESC_C6x0:
					{
						can2[i]->RxMsgC6x0.angle		 = CanRxGetU16(Can2RxMsg, 0);
						can2[i]->RxMsgC6x0.RotateSpeed   = CanRxGetU16(Can2RxMsg, 1);
						can2[i]->RxMsgC6x0.moment		 = CanRxGetU16(Can2RxMsg, 2);
						//云台编码器防越界处理
						if(can2[i]==&GMY)
						{
							//改动该角度时需同时改动底盘掉头程序

								if(can2[i]->RxMsgC6x0.angle > (GM_YAW_ZERO + 2560))
									can2[i]->RxMsgC6x0.angle -= 8192;
								else if(can2[i]->RxMsgC6x0.angle < (GM_YAW_ZERO - 5632))
									can2[i]->RxMsgC6x0.angle += 8192;
						}
						if(can2[i]==&GMP)
						{
							if(can2[i]->RxMsgC6x0.angle > (GM_PITCH_ZERO + 4096))
								can2[i]->RxMsgC6x0.angle -= 8192;
							else if(can2[i]->RxMsgC6x0.angle < (GM_PITCH_ZERO - 4096))
								can2[i]->RxMsgC6x0.angle += 8192;
						}
					}
					case ESC_6623:
					{
						can2[i]->RxMsg6623.angle		 = CanRxGetU16(Can2RxMsg, 0);
						can2[i]->RxMsg6623.realIntensity = CanRxGetU16(Can2RxMsg, 1);
						can2[i]->RxMsg6623.giveIntensity = CanRxGetU16(Can2RxMsg, 2);
					}
				}
			}
		}
		if(Can2RxMsg.StdId==0x200 || Can2RxMsg.StdId==0x1FF)
			flag=1;
		else if(Can2RxMsg.StdId==0x300)
		{
			flag=1;
			#if defined (BOARD_SLAVE) && defined (DOUBLE_BOARD_CAN2)
			
			#endif
		}
		else if(Can1RxMsg.StdId==0x301)
		{
			flag = 1;
			#if defined (BOARD_MAIN) && defined (DOUBLE_BOARD_CAN2)
			
			#endif
		}
		if(!flag)
		{
			Error_Handler();
		}
		if(HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK)
		{
			isRcan2Started = 0;
		}else{
			isRcan2Started = 1;
		}
	}
}

//CAN线主控板间通信
//Called in controlLoop()
void CANTxInfo(CAN_HandleTypeDef* hcan)
{
	extern int16_t channelrcol;
	
	CanTxMsgTypeDef pData;
	hcan->pTxMsg = &pData;
	
	#ifdef BOARD_MAIN
	hcan->pTxMsg->StdId = 0x300;	//标头为0x300，注意检查是否配对及有无冲突情况
	hcan->pTxMsg->ExtId = 0;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->DLC = 0x08;
	
	switch(WorkState)
	{
		case STOP_STATE: hcan->pTxMsg->Data[0] = 0xff; break;
		case PREPARE_STATE: hcan->pTxMsg->Data[0] = 0x00; break;
		case NORMAL_STATE: hcan->pTxMsg->Data[0] = 0x01; break;
		case ADDITIONAL_STATE_ONE: hcan->pTxMsg->Data[0] = 0x02; break;
		case ADDITIONAL_STATE_TWO: hcan->pTxMsg->Data[0] = 0x03; break;
	}
	
	switch(Cap_Get_Cap_State())
	{
		case CAP_STATE_STOP: hcan1.pTxMsg->Data[1] = 0xff; break;
		case CAP_STATE_RECHARGE: hcan->pTxMsg->Data[1] = 0x00; break;
		case CAP_STATE_RELEASE: hcan->pTxMsg->Data[1] = 0x01; break;
		case CAP_STATE_TEMP_RECHARGE: hcan->pTxMsg->Data[1] = 0x02; break;
	}
	
	if(fabs(CMFL.offical_speedPID.fdb - CMFL.offical_speedPID.ref) > 300 || fabs(CMFR.offical_speedPID.fdb - CMFR.offical_speedPID.ref) > 300 || \
		fabs(CMBL.offical_speedPID.fdb - CMBL.offical_speedPID.ref) > 300 || fabs(CMBR.offical_speedPID.fdb - CMBR.offical_speedPID.ref) > 300 || RefereeData.PowerHeat.chassis_power_buffer < 59.0f || ChassisTwistState)
	{
		hcan->pTxMsg->Data[2] = 1;
	}
	else
		hcan->pTxMsg->Data[2] = 0;
	
	hcan->pTxMsg->Data[3] = 0;
	hcan->pTxMsg->Data[4] = 0;
	hcan->pTxMsg->Data[5] = 0;
	hcan->pTxMsg->Data[6] = 0;
	hcan->pTxMsg->Data[7] = 0;
	#endif
	
	#ifdef BOARD_SLAVE
	hcan->pTxMsg->StdId = 0x301;	//标头为0x301，注意检查是否配对及有无冲突情况
	hcan->pTxMsg->ExtId = 0;
	hcan->pTxMsg->IDE = CAN_ID_STD;
	hcan->pTxMsg->RTR = CAN_RTR_DATA;
	hcan->pTxMsg->DLC = 0x08;
	
	
	hcan->pTxMsg->Data[0] = (uint8_t)(Cap_Get_Power_Voltage() * 5);
	hcan->pTxMsg->Data[1] = (uint8_t)((uint16_t)(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()) >> 8 & 0xff);
	hcan->pTxMsg->Data[2] = (uint8_t)((uint16_t)(Cap_Get_Power_CURR()*Cap_Get_Power_Voltage()) & 0xff);
	hcan->pTxMsg->Data[3] = (uint8_t)(Cap_Get_Cap_Voltage() * 5);
	hcan->pTxMsg->Data[4] = 0;
	hcan->pTxMsg->Data[5] = 0;
	hcan->pTxMsg->Data[6] = 0;
	hcan->pTxMsg->Data[7] = 0;
	#endif

	#ifdef DOUBLE_BOARD_CAN1
	if(can1_update == 1 && can1_type == 3)
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
		#ifdef CAN11
			can1_type = 1;
		#else
		#ifdef CAN12
			can1_type = 2;
		#endif
		#endif
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}
	#endif
	
	#ifdef DOUBLE_BOARD_CAN2
	if(can2_update == 1 && can2_type == 3)
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
		#ifdef CAN21
			can2_type = 1;
		#else
		#ifdef CAN22
			can2_type = 2;
		#endif
		#endif
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
	}
	#endif
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan == &hcan1) 
	{
		if(HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK){
			isRcan1Started = 0;
		}else{
			isRcan1Started = 1;
		}
	}
	else if(hcan == &hcan2)
	{
		if(HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0) != HAL_OK)
		{
			isRcan2Started = 0;
		}else{
			isRcan2Started = 1;
		}
	}
}
