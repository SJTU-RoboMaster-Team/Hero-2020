/**
  ******************************************************************************
  * File Name          : KalmanFilter.h
  * Description        : 
  ******************************************************************************
  *
  * Copyright (c) 2019 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */

#ifndef __KALMANFILTER_H
#define __KALMANFILTER_H

typedef struct
{
	float A;
	float H;
	float P;
	float Q;
	float R;
	float Kg;
	float output;
}Kalman_Filter_t;

extern void Kalman_Filter_Init(Kalman_Filter_t* filter, float a, float h, float p, float q, float r, float output_init);
extern float Kalman_Filter(Kalman_Filter_t* filter, float input);
extern void Kalman_Filter_Set_Output(Kalman_Filter_t* filter, float val);

#endif /*__KALMANFILTER_H*/
