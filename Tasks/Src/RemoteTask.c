/**
  ******************************************************************************
  * File Name          : RemoteTask.c
  * Description        : ң������������
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

uint8_t rc_data[18];
RC_Ctl_t RC_CtrlData;
InputMode_e inputmode = REMOTE_INPUT; 
FunctionMode_e functionmode = UPPER_POS;
RemoteSwitch_t g_switch1;
uint8_t KeyBoarddata[10];

/*�������ݴ���*/   
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
	static uint32_t switch_cnt = 0;

	sw->switch_value_raw = val;
	sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;

	//value1 value2��ֵ��ʵ��һ����
	//value1��4λʼ��Ϊ0
	sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
	(sw->switch_value_buf[sw->buf_index]);

	sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

	sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;	

	//�����������һ������û�и������ݣ����˲���
	if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
	{
		switch_cnt++;	
	}
	else
	{
		switch_cnt = 0;
	}
	//�������ά����һ��ʱ�䣬����������40֡һ�������ݣ���Ѳ�������д��switch_long_value
	if(switch_cnt >= 40)
	{
		sw->switch_long_value = sw->switch_value_buf[sw->buf_index]; 	
	}
	//ָ����һ��������
	sw->buf_last_index = sw->buf_index;
	sw->buf_index++;		
	if(sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
	{
		sw->buf_index = 0;	
	}			
}


//ң�������ݽ���
void RemoteDataProcess(uint8_t *pData)
{
	IWDG_counter = 0;
	
	if(pData == NULL)
	{
			return;
	}
	//ң���� 11*4 + 2*2 = 48����Ҫ 6 Bytes
	//16λ��ֻ����11λ
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
											 ((int16_t)pData[4] << 10)) & 0x07FF;
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	
	//16λ��ֻ�������λ
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

	//�����Ҫ 8 Bytes
	RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

	RC_CtrlData.mouse.press_l = pData[12];
	RC_CtrlData.mouse.press_r = pData[13];
	
	//������Ҫ 2 Bytes = 16 bits ��ÿһλ��Ӧһ����
	RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);

	//����״̬����
	if(RC_CtrlData.rc.s2 == 1) inputmode = REMOTE_INPUT;
	else if(RC_CtrlData.rc.s2 == 3) inputmode = KEY_MOUSE_INPUT; 
	else inputmode = STOP; 
	
	//����״̬����
	if(RC_CtrlData.rc.s1 == 1) functionmode = UPPER_POS; 
	else if(RC_CtrlData.rc.s1 == 3) functionmode = MIDDLE_POS; 
	else functionmode = LOWER_POS;
	
	//���Ͻǲ���״̬��RC_CtrlData.rc.s1����ȡ
	//����ң�����������
	GetRemoteSwitchAction(&g_switch1, RC_CtrlData.rc.s1);
	
	switch(inputmode)
	{
		case REMOTE_INPUT:               
		{
			if(WorkState > 0)
			{ 
				#ifdef REMOTE_CONTROL
				RemoteControlProcess(&(RC_CtrlData.rc));
				#endif
				RemoteControlProcess(&(RC_CtrlData.rc));
			}
		}break;
		case KEY_MOUSE_INPUT:              
		{
			if(WorkState > 0)
			{ 
				#ifdef REMOTE_CONTROL
				RemoteControlProcess(&(RC_CtrlData.rc));
				#endif
				MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key,&(RC_CtrlData.rc));
			}
		}break;
		case STOP:
		{
			
		}break;
	}	
	
	for(int i = 0;i<10;i++)
	{
		KeyBoarddata[i] = pData[6+i];
	}
	
}

//��ʼ��ң��������DMA����
void InitRemoteControl(){
	if(HAL_UART_Receive_DMA(&RC_UART, rc_data, 18) != HAL_OK){
			Error_Handler();
	} 
	FunctionTaskInit();
	rx_free = 1;
}

//ң���������ж���ں������Ӵ˴���ʼִ��
uint8_t rc_first_frame = 0;
uint8_t rc_update = 0;
uint8_t rc_cnt = 0;
uint8_t tx_cnt = 200;

uint8_t  tx_free = 1;
uint8_t  rx_free = 1;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &RC_UART)
	{
		rc_update = 1;
		rx_free = 1;
	}
	else if(UartHandle == &JUDGE_UART)
	{
		judgeUartRxCpltCallback();  //����ϵͳ���ݽ���
	}
	else if(UartHandle == &AUTOAIM_UART)
	{
		#ifdef USE_AUTOAIM
		AutoAimUartRxCpltCallback();
		#endif /*USE_AUTOAIM*/
	}
}
#ifdef CAP_DEBUG
extern uint8_t sendfinish;
#endif /* CAP_DEBUG */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &JUDGE_UART)
	{
		tx_free = 1;
	}
	else if(UartHandle == &CAP_UART)
	{
		#ifdef CAP_DEBUG
		sendfinish = 1;
		#endif /* CAP_DEBUG */
	}
}

uint16_t ERRORTEST;
 void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  ERRORTEST = UartHandle->ErrorCode;
	tx_free = 1;
	uint32_t isrflags   = READ_REG(UartHandle->Instance->SR);//�ֲ����н��������Ҫ�ȶ�SR
	if((__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_PE))!=RESET)
	{
		READ_REG(UartHandle->Instance->DR);//PE���־���ڶ�����DR
		__HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_PE);//���־
		UartHandle->gState = HAL_UART_STATE_READY;
		UartHandle->RxState = HAL_UART_STATE_READY;
	}
	if((__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_FE))!=RESET)
	{
		READ_REG(UartHandle->Instance->DR);//FE���־���ڶ�����DR
		__HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_FE);
		UartHandle->gState = HAL_UART_STATE_READY;
		UartHandle->RxState = HAL_UART_STATE_READY;
	}
	
	if((__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_NE))!=RESET)
	{
		READ_REG(UartHandle->Instance->DR);//NE���־���ڶ�����DR
		__HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_NE);
		UartHandle->gState = HAL_UART_STATE_READY;
		UartHandle->RxState = HAL_UART_STATE_READY;
	}        
	
	if((__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_ORE))!=RESET)
	{
		READ_REG(UartHandle->Instance->CR1);//ORE���־���ڶ�����CR
		__HAL_UART_CLEAR_FLAG(UartHandle, UART_FLAG_ORE);
		UartHandle->gState = HAL_UART_STATE_READY;
		UartHandle->RxState = HAL_UART_STATE_READY;
	}

	if(UartHandle == &RC_UART)
	{
		InitRemoteControl();
	}
	else if(UartHandle == &JUDGE_UART)
	{
		InitJudgeUart();
	}
	else if(UartHandle == &AUTOAIM_UART)
	{
		Autoaim_Receive_Start();
	}
}
