/**
  ******************************************************************************
  * File Name       : pid_regulator.c
  * Description     : PID函数
  * Author			：林炳辉
  ******************************************************************************
  *
  * Copyright (c) 2017 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  * C语言PID函数实现
  ******************************************************************************
  */
#include "includes.h"

//uint16_t PID_I_CNT=4;
//float pid_biase=0;
void fw_PID_Reset(fw_PID_Regulator_t *pid){
	pid->errorCurr = 0;
	pid->componentKd = 0;
	pid->componentKi = 0;
	pid->componentKp = 0;
	pid->errorLast = 0;
	pid->errorSum = 0;
	pid->feedback = 0;
	pid->output = 0;
	pid->SumCount = 0;
	pid->target = 0;
	for(int i=0;i<PID_I_CNT;i++) pid->err[i] = 0;
	
}

void fw_PID_Calc(fw_PID_Regulator_t *pid){
	pid->errorCurr = pid->target - pid->feedback;
	pid->errorSum += pid->errorCurr - pid->err[pid->SumCount];
	pid->err[pid->SumCount] = pid->errorCurr;
	pid->SumCount = (pid->SumCount + 1) % PID_I_CNT;
	
	pid->componentKp = pid->kp * pid->errorCurr;
	MINMAX(pid->componentKp, -pid->componentKpMax, pid->componentKpMax);
	pid->componentKi = pid->ki * pid->errorSum;
	MINMAX(pid->componentKi, -pid->componentKiMax, pid->componentKiMax);
	pid->componentKd = pid->kd * (pid->errorCurr - pid->errorLast);
	MINMAX(pid->componentKd, -pid->componentKdMax, pid->componentKdMax);
	
	pid->errorLast = pid->errorCurr;
	
	pid->output = pid->componentKp + pid->componentKi + pid->componentKd;
	
	//pid->output = pid->output + pid_biase;
	
	MINMAX(pid->output, -pid->outputMax, pid->outputMax);
}

float PID_PROCESS(fw_PID_Regulator_t* pid,float target, float feedback)
{
	pid->target = target;
	pid->feedback = feedback;
	pid->Calc(pid);
	return pid->output;
}

float PID_PROCESS_Double(fw_PID_Regulator_t* pid_position,fw_PID_Regulator_t* pid_speed,float target, float position_feedback, float velocity_feedback)
{
	//position		
	pid_position->target = target;
	pid_position->feedback = position_feedback;
	pid_position->Calc(pid_position);
	//speed
	pid_speed->target = pid_position->output;
	pid_speed->feedback = velocity_feedback;
	pid_speed->Calc(pid_speed);
	return pid_speed->output;
}

void chassisMixingPID(float kp,float ki,float kd,float kpE,float kiE,float kdE)
{
	/*底盘混合pid运算 @尹云鹏
	*Kp运算矩阵：
	* |FLKp|   |e1  e12  e13  e14|   |kp |
	* |FRKp| = |e2 -e12  e23  e24| * |kpE|
	* |BLKp|   |e3 -e13 -e23  e34|   |kpE|
	* |BRKp|   |e4 -e14 -e24 -e34|   |kpE|
	*Ki,Kd运算类似;
	*/
	static float e1,e2,e3,e4,e12,e13,e14,e23,e24,e34;//偏差，p
	static float s1,s2,s3,s4,s12,s13,s14,s23,s24,s34;//偏差和，i
	static float d1,d2,d3,d4,d12,d13,d14,d23,d24,d34;//偏差差，d
	static float l1,l2,l3,l4,l12,l13,l14,l23,l24,l34;//上一次的偏差，last e
	static float tmpP=0,tmpI=0,tmpD=0,tmp;
	
	e1=CMFL.TargetAngle - CMFL.RxMsgC6x0.RotateSpeed;
	e2=CMFR.TargetAngle - CMFR.RxMsgC6x0.RotateSpeed;e2=-e2;
	e3=CMBL.TargetAngle - CMBL.RxMsgC6x0.RotateSpeed;
	e4=CMBR.TargetAngle - CMBR.RxMsgC6x0.RotateSpeed;e4=-e4;
	e12=e1-e2;e13=e1-e3;e14=e1-e4;
	e23=e2-e3;e24=e2-e4;e34=e3-e4;
	
	s1+=e1;s2+=e2;s3+=e3;s4+=e4;
	s12+=e12;s13+=e13;s14+=e14;
	s23+=e23;s24+=e24;s34+=e34;
	//循环自乘0.99让s收敛
	s1*=0.99f;s2*=0.99f;s3*=0.99f;s4*=0.99f;
	s12*=0.99f;s13*=0.99f;s14*=0.99f;
	s23*=0.99f;s24*=0.99f;s34*=0.99f;
	
	d1=e1-l1;d2=e2-l2;d3=e3-l3;d4=e4-l4;
	d12=e12-l12;d13=e13-l13;d14=e14-l14;
	d23=e23-l23;d24=e24-l24;d34=e34-l34;
	
	l1=e1;l2=e2;l3=e3;l4=e4;
	l12=e12;l13=e13;l14=e14;
	l23=e23;l24=e24;l34=e34;
	
	tmpP=kp*e1+(e12+e13+e14)*kpE;
	MINMAX(tmpP,-15000,15000);
	tmpI=ki*s1+(s12+s13+s14)*kiE;
	MINMAX(tmpI,-15000,15000);
	tmpD=kd*d1+(d12+d13+d14)*kdE;
	MINMAX(tmpD,-15000,15000);
	tmp=tmpP+tmpI+tmpD;
	MINMAX(tmp,-12000,12000);
	CMFL.Intensity=tmp;
	
	tmpP=kp*e2+(-e12+e23+e24)*kpE;
	MINMAX(tmpP,-15000,15000);
	tmpI=ki*s2+(-s12+s23+s24)*kiE;
	MINMAX(tmpI,-15000,15000);
	tmpD=kd*d2+(-d12+d23+d24)*kdE;
	MINMAX(tmpD,-15000,15000);
	tmp=tmpP+tmpI+tmpD;
	MINMAX(tmp,-12000,12000);
	CMFR.Intensity= - tmp;//-
	
	tmpP=kp*e3+(-e13-e23+e34)*kpE;
	MINMAX(tmpP,-15000,15000);
	tmpI=ki*s3+(-s13-s23+s34)*kiE;
	MINMAX(tmpI,-15000,15000);
	tmpD=kd*d3+(-d13-d23+d34)*kdE;
	MINMAX(tmpD,-15000,15000);
	tmp=tmpP+tmpI+tmpD;
	MINMAX(tmp,-12000,12000);
	CMBL.Intensity=tmp;
	
	tmpP=kp*e4+(-e14-e24-e34)*kpE;
	MINMAX(tmpP,-15000,15000);
	tmpI=ki*s4+(-s14-s24-s34)*kiE;
	MINMAX(tmpI,-15000,15000);
	tmpD=kd*d4+(-d14-d24-d34)*kdE;
	MINMAX(tmpD,-15000,15000);
	tmp=tmpP+tmpI+tmpD;
	MINMAX(tmp,-12000,12000);
	CMBR.Intensity= - tmp;//-
}
