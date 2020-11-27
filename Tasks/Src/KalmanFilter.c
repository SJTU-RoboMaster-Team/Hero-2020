/**
  ******************************************************************************
  *FileName				: KalmanFilter.c
  *Description		: ¿¨¶ûÂüÂË²¨
  *Author					: 
  ******************************************************************************
  *
  * Copyright (c) 2019 Team JiaoLong-ShanghaiJiaoTong University
  * All rights reserved.
  *
  ******************************************************************************
*/

#include "KalmanFilter.h"

void Kalman_Filter_Init(Kalman_Filter_t* filter, float a, float h, float p, float q, float r, float output_int)
{
	filter->A = a;
	filter->H = h;
	filter->P = p;
	filter->Q = q;
	filter->R = r;
	filter->output = output_int;
}

float Kalman_Filter(Kalman_Filter_t* filter, float input)
{
	filter->output = filter->A * filter->output;
	filter->P = filter->A * filter->P * filter->A + filter->Q;
	filter->Kg = filter->P * filter->H / (filter->H * filter->P * filter->H + filter->R);
	filter->output = filter->Kg * input + (1 - filter->Kg) * filter->output;
	filter->P = (1 - filter->Kg * filter->H) * filter->P;
	return filter->output;
}

void Kalman_Filter_Set_Output(Kalman_Filter_t* filter, float val)
{
	filter->output = val;
}
