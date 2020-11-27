/**
  ******************************************************************************
  * File Name          : FunctionTask.c
  * Description        : 用于记录机器人独有的功能
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

#ifdef HERO
#define FRIC_SPEED_1  3650 //3200  //对应4个等级 10， 12， 14， 16 //3600对应16
#define FRIC_SPEED_2 	3750
#define FRIC_SPEED_3 	4400
#define FRIC_SPEED_4 	5000
#define SHOOT_SPEED_1	7.5f
#define SHOOT_SPEED_2	9.5f
#define SHOOT_SPEED_3	11.0f
#define SHOOT_SPEED_4	15.0f
#endif

//遥控常量
#define RC_CHASSIS_SPEED_REF    		0.85f
#define RC_ROTATE_SPEED_REF 				0.07f
#define RC_GIMBAL_SPEED_REF					0.006f
//遥控器死区
#define IGNORE_RANGE 								0
//速度常量
#define NORMAL_FORWARD_BACK_SPEED 	600
#define NORMAL_LEFT_RIGHT_SPEED  		600/1.5f
#define HIGH_FORWARD_BACK_SPEED 		800
#define HIGH_LEFT_RIGHT_SPEED   		800/1.5f
#define LOW_FORWARD_BACK_SPEED 			300
#define LOW_LEFT_RIGHT_SPEED   			300/1.5f
//扭腰幅度
#define CHASSIS_TWIST_ANGLE_LIMIT		60
//鼠标长按时间阈值
#define MOUSE_LR_RAMP_TICK_COUNT		50
#define MOUSR_FB_RAMP_TICK_COUNT		60
//鼠标灵敏度
#define MOUSE_TO_YAW_ANGLE_INC_FACT		((aim_mode != 0 && find_enemy) ? 0.03f : 0.06f)
#define MOUSE_TO_PITCH_ANGLE_INC_FACT	((aim_mode != 0	&& find_enemy) ? 0.03f : 0.06f)
//云台微调幅度
#define SHOOTMODE_GM_ADJUST_ANGLE		0.05f
//射击变量
#define STIR_STEP_ANGLE 60
#define	LONG_CD		500
#define	SHORT_CD	100

#define GATE_CLOSE	0
#define GATE_OPEN 	1
#define GATE_BLOCK_MOMENT_THRES (3800)
#define GMY_FINE_ADJUSTMENT_BLOCK_MOMENT_THRES  3000

#define FRIC_ON() \
{\
	if(block_flag == 0) ShootState = 1;\
	else ShootState = 0;\
	FRICAL.TargetAngle = -FricSpeed;\
	FRICAR.TargetAngle = FricSpeed;\
}
#define FRIC_OFF() \
{\
	ShootState = 0;\
	FRICAL.TargetAngle = 0;\
	FRICAR.TargetAngle = 0;\
}

void Test_Mode_Handler(void);
void Bullet_Block_Handler(void);
void Gate_Handler(uint8_t gate_state);
void Chassis_forward_back_Handler(void);
void Reset(uint8_t);
void Climb_Handler(void);
void Auto_Climb(void);

KeyboardMode_e KeyboardMode = NO_CHANGE;
KeyboardMode_e LastKeyboardMode = NO_CHANGE;
MouseMode_e MouseLMode = NO_CLICK;
MouseMode_e MouseRMode = NO_CLICK;
//斜坡函数
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
//速度变量
ChassisSpeed_Ref_t ChassisSpeedRef;
//计时变量
uint32_t ShootManyBullet_cnt=0;
int32_t auto_counter=0;
int32_t auto_counter_stir=0;
int16_t auto_counter_heat0=0;
int16_t auto_counter_heat1=0;
int16_t shoot_cd=0;
int16_t gmy_inposition = 0; //0在外面 1在啮合
int16_t gfa0 = 0;//1是完成

//遥控器拨杆数据
int16_t channelrrow = 0;
int16_t channelrcol = 0;
int16_t channellrow = 0;
int16_t channellcol = 0;

uint8_t TestMode = 0;
uint8_t ShootState = 0;
uint8_t ChassisTwistState = 0;
int8_t ChassisTwistGapAngle = 0;
uint8_t chassis_lock = 0;//底盘锁定
int16_t chassis_follow_center = GM_YAW_ZERO;//底盘跟随前方角度
uint8_t chassis_change_forward_back = 0;
uint8_t change_forward_back_rcd = 0;
uint8_t change_forward_back_step = 0;
uint8_t GM_Turn_back = 0;//回头取弹标志

int16_t FricSpeed = FRIC_SPEED_1;//摩擦轮转速
float BulletSpeed = SHOOT_SPEED_4;
uint8_t burst = 0;//无视热量限制
uint8_t block_flag = 0;//卡弹标记
int16_t cur_cd = LONG_CD; //发射间隔

uint8_t gate_state = GATE_CLOSE;//弹仓盖状态
uint8_t gate_first_enter = 1;
uint8_t gate_already_open = 0;

uint8_t ClimbF_first_enter = 1;
uint8_t ClimbB_first_enter = 1;
uint8_t Fric_state = 0;
uint8_t Shoot_first_enter=1;

//上岛
Sensor adf,adb;
uint32_t disf=0,disb=0;
uint8_t AutoClimbing=0;

uint8_t hasReach(MotorINFO* id, double distance)//用于判断电机是否到位
{
	if(fabs(id->RealAngle - id->TargetAngle) < distance)return 1;
	else return 0;
}

//初始化
void FunctionTaskInit()
{
	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
	FBSpeedRamp.ResetCounter(&FBSpeedRamp);
	
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
	
	KeyboardMode=NO_CHANGE;
	
	FRIC_OFF();
}

//******************
//遥控器模式功能编写
//2020/8/3 @王云浩 @徐欣
//******************
#define CLIMB_F_LIMIT_HIGH 660
#define CLIMB_F_LIMIT_LOW  20
#define CLIMB_B_LIMIT_HIGH -10
#define CLIMB_B_LIMIT_LOW -520

#define CLIMB_F_DOWN 660
#define CLIMB_F_UP 20
#define CLIMB_B_DOWN -520
#define CLIMB_B_UP -10

//#define CLIMB_F_LIMIT_HIGH 1000
//#define CLIMB_F_LIMIT_LOW  -200
//#define CLIMB_B_LIMIT_HIGH -5
//#define CLIMB_B_LIMIT_LOW -1000
#define RemoteControl_Mode_CLIMB //切换遥控器配置
//#define RemoteControl_Mode_GIMBAL
//#define RemoteControl_Mode_DEBUG
//#define RemoteControl_Mode_CLIMB_PRIMARY

void RemoteControlProcess(Remote *rc)
{
	static WorkState_e LastState = NORMAL_STATE;
	
	static uint8_t IfShoot = 0;
	
	if(WorkState <= 0) return;
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	/*
	#ifdef RemoteControl_Mode_CLIMB_PRIMARY
	//上台阶遥控器配置
		FRIC_OFF();//不用摩擦轮
		Climb_Handler();
	
		if(WorkState == NORMAL_STATE)
		{//1档，普通运动，前后左右加旋转和自动上岛
			ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
			ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
			ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
			if(channellcol>500)
				AutoClimbing=1;
			Auto_Climb();
			
			#ifdef USE_CHASSIS_FOLLOW
				//GMY.TargetAngle += YAW_DIR * channellrow * RC_GIMBAL_SPEED_REF;
				//GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF;
			#else
				ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
				GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF;
			#endif
			//aim_mode = 0;
			//chassis_lock = 0;
			//ChassisTwistState = 0;
			
//			if(gate_first_enter==0){
//				gate_state=GATE_CLOSE;
//				gate_already_open=0;
//			}
		}
		
		if(WorkState == ADDITIONAL_STATE_ONE)   
		{//2档，快速动台阶+移动+弹仓门
			if(ClimbF_first_enter == 0&&ClimbB_first_enter == 0)
			{
				if(channellcol<-300)
					CLIMB_F.TargetAngle=CLIMB_F_LIMIT_HIGH;
				if(channellcol>300)
					CLIMB_F.TargetAngle=CLIMB_F_LIMIT_LOW;
				if(channellrow<-300)
					CLIMB_B.TargetAngle=CLIMB_B_LIMIT_LOW;
				if(channellrow>300)
					CLIMB_B.TargetAngle=CLIMB_B_LIMIT_HIGH;		
			}
			ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
			ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
			if(gate_first_enter==0){
				gate_state=GATE_CLOSE;
				gate_already_open=0;
			}
		}
		
		if(WorkState == ADDITIONAL_STATE_TWO)
		{//3档，缓慢动台阶+移动+弹仓门
			CLIMB_F.TargetAngle-=channellcol*0.01; //670最下
			CLIMB_B.TargetAngle+=channellrow*0.01; //-530最下
			ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
			ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
			if(gate_first_enter==0&&gate_already_open==0)
				gate_state=GATE_OPEN;
		}
	#endif
		*/
	#ifdef RemoteControl_Mode_CLIMB
		//max=660
		channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
		channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
		channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
		channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
		Climb_Handler();  
		
		if(WorkState == NORMAL_STATE)
		{//1档底盘正常移动
			ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
			ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
			#ifdef USE_CHASSIS_FOLLOW
				GMY.TargetAngle += YAW_DIR * channellrow * RC_GIMBAL_SPEED_REF;
				GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF;
			#else
				ChassisSpeedRef.rotate_ref = -channellrow * RC_ROTATE_SPEED_REF;
				GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF;
			#endif
			
			chassis_lock = 0;
			ChassisTwistState = 0;
			FRIC_OFF();
			aim_mode = PREDICT_MODE;
			HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
		}
		
		if(WorkState == ADDITIONAL_STATE_ONE)   
		{//2档，快速上台阶+正常移动
			FRIC_OFF();
			chassis_lock = 0;
			aim_mode = PREDICT_MODE;
			if(ClimbF_first_enter == 0&&ClimbB_first_enter == 0)
			{
				if(channellcol<-300)
					CLIMB_F.TargetAngle=CLIMB_F_LIMIT_HIGH;
				if(channellcol>300)
					CLIMB_F.TargetAngle=CLIMB_F_LIMIT_LOW;
				if(channellrow<-300)
					CLIMB_B.TargetAngle=CLIMB_B_LIMIT_LOW;
				if(channellrow>300)
					CLIMB_B.TargetAngle=CLIMB_B_LIMIT_HIGH;		
			}
			ChassisSpeedRef.forward_back_ref = channelrcol * RC_CHASSIS_SPEED_REF;
			ChassisSpeedRef.left_right_ref   = channelrrow * RC_CHASSIS_SPEED_REF/3*2;
			HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
		}
		
		if(WorkState == ADDITIONAL_STATE_TWO)
		{//3档,吊射
			chassis_lock = 1;
			ChassisTwistState = 0;
			FRIC_ON();
			
			//if(gmy_inposition)
			{
			  GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF*0.1f;
			  GMY.TargetAngle += YAW_DIR * channellrow * RC_GIMBAL_SPEED_REF*0.1f;
			}
			
			/*
			FRIC_OFF();
			if(block_flag == 0) ShootState = 1;
			else ShootState = 0;
			 */
			aim_mode = 0;
			HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
			
			if(channelrcol>500 && !IfShoot)
			{
				//ShootBullets(BULLET_TYPE,2);
				ShootOneBullet(BULLET_TYPE);
				IfShoot = 1;
			}
			if(channelrrow>500 && !IfShoot)
			{
				ShootOneBullet(BULLET_TYPE);
				IfShoot = 1;
			}
			
			if(channelrrow<100 && IfShoot && channelrcol<100)
			{
				IfShoot = 0;
			}
			
		}
	#endif
		
	#ifdef RemoteControl_Mode_DEBUG
	//调试遥控器配置
		Climb_Handler();//导轨reset
		
		if(WorkState == NORMAL_STATE)
		{
			chassis_lock = 1;
			GMY.TargetAngle += YAW_DIR * channellrow * RC_GIMBAL_SPEED_REF;
			GMP.TargetAngle += PIT_DIR * channellcol * RC_GIMBAL_SPEED_REF;
			aim_mode = 0;
			HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
		}
		if(WorkState == ADDITIONAL_STATE_ONE)   
		{
			
		}
		if(WorkState == ADDITIONAL_STATE_TWO)
		{

		}
	#endif
		
		
	Bullet_Block_Handler();
	Gate_Handler(gate_state);
	Chassis_forward_back_Handler();
	
	LastState = WorkState;
}


uint16_t KM_FORWORD_BACK_SPEED 	= NORMAL_FORWARD_BACK_SPEED;
uint16_t KM_LEFT_RIGHT_SPEED  	= NORMAL_LEFT_RIGHT_SPEED;
void KeyboardModeFSM(Key *key);
void MouseModeFSM(Mouse *mouse);


//------------
//原键鼠模式重分配
//@尹云鹏   controlTask changed
//左拨杆下才为真正的键鼠模式，上、中两档位可编辑
//------------
extern uint8_t sendfinish;  extern int32_t cps[4][4000];//用于串口发送功率数据@唐欣阳

void MouseKeyControlProcess(Mouse *mouse, Key *key,Remote *rc)
{	
	static WorkState_e LastState = NORMAL_STATE;
	static int Key_Z_state = 0;
	static int Key_R_state = 0;
	if(WorkState <= 0) return;
	
	static uint8_t IfShoot = 0;

	#ifdef RemoteControl_Mode_DEBUG
		return;
	#endif
	
	#ifdef RemoteControl_Mode_CLIMB
	//max=660
	channelrrow = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channelrcol = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellrow = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET); 
	channellcol = (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	
	if(WorkState == NORMAL_STATE)
	{
		/*
		if(channelrcol>100)
		{
		ChassisSpeedRef.forward_back_ref = 60;
		}
		else if(channelrcol<-100)
		{
		ChassisSpeedRef.forward_back_ref = -60;
		}
		else
		{
		ChassisSpeedRef.forward_back_ref = 0;
		}
		
		if(channelrrow>100)
		{
		ChassisSpeedRef.left_right_ref = 60;
		}
		else if(channelrrow<-100)
		{
		ChassisSpeedRef.left_right_ref = -60;
		}
		else
		{
		ChassisSpeedRef.left_right_ref = 0;
		}
		*/
		
		ChassisSpeedRef.forward_back_ref = 0.5*channelrcol * RC_CHASSIS_SPEED_REF;
		ChassisSpeedRef.left_right_ref   = 0.5*channelrrow * RC_CHASSIS_SPEED_REF/3*2;
		
		if(channellcol>500)
		{
			ChassisTwistState = 1; 
			chassis_lock = 0;
		}
		else if(channellcol<-500)
		{
			ChassisTwistState = 3; 
			chassis_lock = 0;
		}
		else
		{
			ChassisTwistState = 0; 
		}
		GMY.TargetAngle += YAW_DIR * channellrow * RC_GIMBAL_SPEED_REF;
		FRIC_OFF();
		aim_mode = 0;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	}
	
	control_radar_state = 0;
	if(WorkState == ADDITIONAL_STATE_ONE)
	{
		control_radar_state = 1;
	   Control_radar();
	}
	
	//键鼠模式
	if(WorkState == ADDITIONAL_STATE_TWO)
	{
		MINMAX(mouse->x, -150, 150); 
		MINMAX(mouse->y, -150, 150); 
		//Climb_Handler();  //装好调回来
		#ifdef USE_CHASSIS_FOLLOW
			if((mouse->x > 100) || (mouse->x < -100))
			{
				chassis_lock = 0;
			}
			if(chassis_lock==0)
			{
				GMY.TargetAngle += YAW_DIR * mouse->x * MOUSE_TO_YAW_ANGLE_INC_FACT;
				GMP.TargetAngle -= PIT_DIR * mouse->y * MOUSE_TO_PITCH_ANGLE_INC_FACT;
			}
		#else
			if(chassis_lock==0)
			{
				ChassisSpeedRef.rotate_ref = -mouse->x * RC_ROTATE_SPEED_REF;
			}
		#endif
		
		FRIC_ON();
		aim_mode = 0;
		HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);

		MouseModeFSM(mouse);
		
		switch(MouseRMode)
		{
			case SHORT_CLICK:
			{
				if(ShootState)
				{
					
				}
			}break;
			case LONG_CLICK:
			{
				if(ShootState)
				{
					
				}
			}break;
			default: break;
		}
		
		switch (MouseLMode)
		{
			case SHORT_CLICK:
			{
				if(!IfShoot)
				{
					ShootOneBullet(BULLET_TYPE);
					IfShoot = 1;
				}
			}break;
			case LONG_CLICK:
			{
				//ShootOneBullet(BULLET_TYPE);
			}
			default: 
			{
				IfShoot = 0; 
				break;
			}
		}

		KeyboardModeFSM(key);
		
		//组合键键位
		switch (KeyboardMode)
		{
			case SHIFT_CTRL:
			{
				burst = 1;
				break;
			}
			case CTRL:
			{
				burst = 0;
				
				//上台阶自动
				if(key->v & KEY_Q)
				{
					AutoClimbing=1;
					Auto_Climb();
				}
				
				//上台阶慢  TODO:调参
				if(key->v & KEY_X)
				{
					CLIMB_F.TargetAngle+=1500*0.01; //670最下
				}
				if(key->v & KEY_Z)
				{
					CLIMB_F.TargetAngle-=1500*0.01; //670最下
				}
				if(key->v & KEY_V)
				{
					CLIMB_B.TargetAngle-=1500*0.01; //-530最下
				}
				if(key->v & KEY_C)
				{
					CLIMB_B.TargetAngle+=1500*0.01; //-530最下
				}
				
				break;
			
			}
			
			case SHIFT:
			{
				burst = 0;
				
				//上台阶快
				if(key->v & KEY_X)
				{
					CLIMB_F.TargetAngle=CLIMB_F_LIMIT_HIGH;
				}
				if(key->v & KEY_Z)
				{
					CLIMB_F.TargetAngle=CLIMB_F_LIMIT_LOW;
				}
				if(key->v & KEY_V)
				{
					CLIMB_B.TargetAngle=CLIMB_B_LIMIT_LOW;
				}
				if(key->v & KEY_C)
				{
					CLIMB_B.TargetAngle=CLIMB_B_LIMIT_HIGH;		
				}
				
				break;
			}
			
			case NO_CHANGE:
			{
				burst = 0;
				if(key->v & KEY_R && Key_R_state == 0)
				{	
					chassis_lock = 1;
					Key_R_state = 1;
				}
				if((!(key->v & KEY_R)) && Key_R_state == 1)
				{
					Key_R_state = 0;
				}
				
				//打开弹舱门
				if(key->v &KEY_X)
				{
					gate_state = GATE_OPEN;
				}
				
				//吊射模式云台微调
				if(chassis_lock)
				{
					if(key->v & KEY_W)  			//key: w
						GMP.TargetAngle += PIT_DIR * SHOOTMODE_GM_ADJUST_ANGLE*0.5;
					else if(key->v & KEY_S) 	//key: s
						GMP.TargetAngle -= PIT_DIR * SHOOTMODE_GM_ADJUST_ANGLE*0.5;
					if(key->v & KEY_D)  			//key: d
						GMY.TargetAngle += YAW_DIR * SHOOTMODE_GM_ADJUST_ANGLE*0.5;
					else if(key->v & KEY_A) 	//key: a
						GMY.TargetAngle -= YAW_DIR * SHOOTMODE_GM_ADJUST_ANGLE*0.5;
				}
				
				//底盘运动控制
				if(key->v & KEY_W)  			//key: w
					ChassisSpeedRef.forward_back_ref =  KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
				else if(key->v & KEY_S) 	//key: s
					ChassisSpeedRef.forward_back_ref = -KM_FORWORD_BACK_SPEED* FBSpeedRamp.Calc(&FBSpeedRamp);
				else
				{
					ChassisSpeedRef.forward_back_ref = 0;
					FBSpeedRamp.ResetCounter(&FBSpeedRamp);
				}
				if(key->v & KEY_D)  			//key: D
					ChassisSpeedRef.left_right_ref =  KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
				else if(key->v & KEY_A) 	//key: A
					ChassisSpeedRef.left_right_ref = -KM_LEFT_RIGHT_SPEED * LRSpeedRamp.Calc(&LRSpeedRamp);
				else
				{
					ChassisSpeedRef.left_right_ref = 0;
					LRSpeedRamp.ResetCounter(&LRSpeedRamp);
				}
				
				
				//扭腰、陀螺模式选择
				if(key->v & KEY_Q) {ChassisTwistState = 1; chassis_lock = 0;}
				if(key->v & KEY_E) {ChassisTwistState = 3; chassis_lock = 0;}
				if(key->v & KEY_F) 
				{
					Reset(0);
				}
				//回头取弹
				if(key->v & KEY_Z && Key_Z_state == 0)
				{	
					if(GM_Turn_back == 0)
					{
						gate_state=GATE_OPEN;
						GM_Turn_back = 1;
						chassis_follow_center += 4096.0f;
					}
					Key_Z_state = 1;
				}
				if((!(key->v & KEY_Z)) && Key_Z_state == 1)
				{
					Key_Z_state = 0;
				}
				
				//自瞄模式选择
				if(key->v & KEY_C)
				{
					aim_mode = PREDICT_MODE;
					if(mouse->press_r) AutoShoot(aim_mode);
				}
				else if(key->v & KEY_V)
				{
					aim_mode = ANTI_GYRO_MODE;
					if(mouse->press_r) AutoShoot(aim_mode);
				}
				else if(mouse->press_r)
				{
					if(GMP.RealAngle < -10)
					{
						aim_mode = PREDICT_MODE;
						if(MouseRMode == LONG_CLICK) AutoShoot(aim_mode);
					}
					else
					{
						aim_mode = ANTI_GYRO_MODE;
						AutoShoot(aim_mode);
					}
				}
				else
				{
					aim_mode = 0;
				}
				
				break;
			}
		}
		
		//射频选择
		if(KeyboardMode == SHIFT_CTRL || KeyboardMode == CTRL || (mouse->press_r && mouse->press_l && auto_shoot_flag == 1))
		{
			//cur_cd = SHORT_CD;
			if(shoot_cd > cur_cd)
				shoot_cd = cur_cd;
		}
		else
		{
			cur_cd = LONG_CD;
			if(shoot_cd > cur_cd)
				shoot_cd = cur_cd;
		}
		
		//UI test
		if(KeyboardMode == CTRL && key->v & KEY_B)
		{
			Client_Graph_Start();
		}
		else if(key->v & KEY_B)
		{
			Client_Graph_Clear();
		}
		
		if(LastState != WorkState)
		{
			ChassisTwistState = 0;
		}
		
		//控制雷达测试
		if(KeyboardMode == SHIFT_CTRL && key->v & KEY_B)
		{
			control_radar_state = 1;
		}	
		else
		{
			control_radar_state = 0;
		}
	}
	#endif
	
	//防卡弹处理
  Bullet_Block_Handler();
	Gate_Handler(gate_state);
	Chassis_forward_back_Handler();
	
	LED_Show_SuperCap_Voltage(1);
	
	LastState = WorkState;
}

KeyBoard_MoveMode KeyBoardMoveMode = MoveMode_CAP_RECHARGE_MODE;

void KeyboardModeFSM(Key *key)
{
	if((key->v & 0x30) == 0x30)//Shift_Ctrl
	{
		KeyboardMode=SHIFT_CTRL;
	}
	else if(key->v & KEY_SHIFT)//Shift
	{
		KeyboardMode=SHIFT;
	}
	else if(key->v & KEY_CTRL)//Ctrl
	{
		KeyboardMode=CTRL;
	}
	else
	{
		KeyboardMode=NO_CHANGE;
	}	
	
/*
	SHIFT_CTRL
	*/
	/*
	CTRL
	*/
	if (KeyboardMode != LastKeyboardMode && KeyboardMode == SHIFT_CTRL)
	{ 
		if (KeyBoardMoveMode == MoveMode_CAP_STOP_MODE )
		{
			KeyBoardMoveMode = MoveMode_CAP_RECHARGE_MODE;
		}
		else if (KeyBoardMoveMode == MoveMode_CAP_RECHARGE_MODE || KeyBoardMoveMode == MoveMode_CAP_RELEASE_HIGH_MODE)
		{
			KeyBoardMoveMode = MoveMode_CAP_STOP_MODE;
		}
	}
	/*
	SHIFT
	*/
	if (KeyboardMode == SHIFT){
		if (KeyBoardMoveMode != MoveMode_CAP_RELEASE_HIGH_MODE /*&& KeyBoardMoveMode != MoveMode_CAP_STOP_MODE */){
			KeyBoardMoveMode = MoveMode_CAP_RELEASE_HIGH_MODE;
		}			
	}else{
		if (KeyBoardMoveMode == MoveMode_CAP_RELEASE_HIGH_MODE){
			KeyBoardMoveMode = MoveMode_CAP_RECHARGE_MODE;
		}
	}
	if (KeyBoardMoveMode == MoveMode_CAP_STOP_MODE){
		KeyBoardMoveMode = MoveMode_CAP_RECHARGE_MODE;
	}
	
	/*
	
	*/
	switch (KeyBoardMoveMode){
		case MoveMode_CAP_RELEASE_LOW_MODE://
			if(Cap_Get_Power_Voltage() > 11 && Cap_Get_Cap_State() != CAP_STATE_TEMP_RECHARGE && Cap_Get_Cap_State() != CAP_STATE_RELEASE)
		  {
				#ifndef BOARD_SLAVE
			  Cap_State_Switch(CAP_STATE_RELEASE);
				#endif
		  }
			KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
      KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
			break;
		case MoveMode_CAP_RELEASE_HIGH_MODE: //
			if(Cap_Get_Power_Voltage() > 11.5 && Cap_Get_Cap_State() != CAP_STATE_TEMP_RECHARGE && Cap_Get_Cap_State() != CAP_STATE_RELEASE)
		  {
			  Cap_State_Switch(CAP_STATE_RELEASE);
		  }
		  if(Cap_Get_Cap_Voltage() > 13) 
		  {
			  KM_FORWORD_BACK_SPEED=  HIGH_FORWARD_BACK_SPEED;
			  KM_LEFT_RIGHT_SPEED = HIGH_LEFT_RIGHT_SPEED;
		  }
		  else
		  {
			  KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
			  KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
		  }
			break;
		case MoveMode_CAP_STOP_MODE://
//			if(Cap_Get_Cap_State() != CAP_STATE_STOP)
//		  {
//			  Cap_State_Switch(CAP_STATE_STOP);
//		  }
			KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
      KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
			break;
		case MoveMode_CAP_RECHARGE_MODE://
			if(Cap_Get_Cap_State() != CAP_STATE_RECHARGE)
		  {
				#ifndef BOARD_SLAVE
			  Cap_State_Switch(CAP_STATE_RECHARGE);
				#endif
		  }
			KM_FORWORD_BACK_SPEED=  NORMAL_FORWARD_BACK_SPEED;
      KM_LEFT_RIGHT_SPEED = NORMAL_LEFT_RIGHT_SPEED;
			break;
	}
	/*
	CTRL
	*/
	if (KeyboardMode == CTRL){
     KM_FORWORD_BACK_SPEED=  LOW_FORWARD_BACK_SPEED; 
     KM_LEFT_RIGHT_SPEED = LOW_LEFT_RIGHT_SPEED;
	}
	LastKeyboardMode=KeyboardMode;
}


void MouseModeFSM(Mouse *mouse)
{
	static uint8_t counterl = 0;
	static uint8_t counterr = 0;
	switch (MouseLMode)
	{
		case SHORT_CLICK:
		{
			counterl++;
			if(mouse->press_l == 0)
			{
				MouseLMode = NO_CLICK;
				counterl = 0;
			}
			else if(counterl>=50)
			{
				MouseLMode = LONG_CLICK;
				counterl = 0;
			}
			else
			{
				MouseLMode = SHORT_CLICK;
			}
		}break;
		case LONG_CLICK:
		{
			if(mouse->press_l==0)
			{
				MouseLMode = NO_CLICK;
			}
			else
			{
				MouseLMode = LONG_CLICK;
			}
		}break;
		case NO_CLICK:
		{
			if(mouse->press_l)
			{
				//ShootOneBullet(BULLET_TYPE);
				MouseLMode = SHORT_CLICK;
			}
		}break;
	}
	
	switch (MouseRMode)
	{
		case SHORT_CLICK:
		{
			counterr++;
			if(mouse->press_r == 0)
			{
				MouseRMode = NO_CLICK;
				counterr = 0;
			}
			else if(counterr>=50)
			{
				MouseRMode = LONG_CLICK;
				counterr = 0;
			}
			else
			{
				MouseRMode = SHORT_CLICK;
			}
		}break;
		case LONG_CLICK:
		{
			if(mouse->press_r==0)
			{
				MouseRMode = NO_CLICK;
			}
			else
			{
				MouseRMode = LONG_CLICK;
			}
		}break;
		case NO_CLICK:
		{
			if(mouse->press_r)
			{
				MouseRMode = SHORT_CLICK;
			}
		}break;
	}
}

//用于遥控器模式下超级电容测试模式的控制
void Test_Mode_Handler(void)
{
	static uint8_t counter = 0;
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2))
	{
		counter++;
		if(counter==40)
		{
			TestMode = (TestMode==1)?0:1;
		}
	}
	else
	{
		counter = 0;
	}
	if(TestMode==1)
	{
		HAL_GPIO_WritePin(GPIOF, LED_GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOF, LED_GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, LED_RED_Pin, GPIO_PIN_SET);
	}
}

void ChassisTwist(void)
{
	switch (ChassisTwistState)
	{
		case 1:
		{
			ChassisSpeedRef.rotate_ref = 50;
			//ChassisSpeedRef.rotate_ref = 100;
			break;
		}
		case 2:
		{
			ChassisSpeedRef.rotate_ref = -50;
			//ChassisSpeedRef.rotate_ref = -100;
			break;
		}
		case 3:
		{
			switch (ChassisTwistGapAngle)
			{
				case 0:
				{
					ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT;
					break;
				}
				case CHASSIS_TWIST_ANGLE_LIMIT:
				{
					if(fabs((GMY.RxMsgC6x0.angle - chassis_follow_center) * 360 / 8192.0f - ChassisTwistGapAngle) < 10)
						ChassisTwistGapAngle = -CHASSIS_TWIST_ANGLE_LIMIT;
					break;
				}
				case -CHASSIS_TWIST_ANGLE_LIMIT:
				{
					if(fabs((GMY.RxMsgC6x0.angle - chassis_follow_center) * 360 / 8192.0f - ChassisTwistGapAngle) < 10)
						ChassisTwistGapAngle = CHASSIS_TWIST_ANGLE_LIMIT;
					break;
				}
			}
			ChassisSpeedRef.rotate_ref = (GMY.RxMsgC6x0.angle - chassis_follow_center) * 360 / 8192.0f - ChassisTwistGapAngle;
			break;
		}
		default:
		{
			ChassisTwistGapAngle = 0;
			ChassisSpeedRef.rotate_ref = (GMY.RxMsgC6x0.angle - chassis_follow_center) * 360 / 8192.0f - ChassisTwistGapAngle;
			break;
		}
	}
}

void Chassis_forward_back_Handler(void)
{
	if(!chassis_lock)
	{
		if(change_forward_back_rcd != chassis_change_forward_back)
		{
			change_forward_back_step = 1;
		}

		switch(change_forward_back_step)
		{
			case 2:
			{
				chassis_follow_center = GM_YAW_ZERO - 2048;
				if(fabs((GMY.RxMsgC6x0.angle - chassis_follow_center) * 360 / 8192.0f) < 45)
				{
					change_forward_back_step = 1;
				}
				break;
			}
			case 1:
			{
				chassis_follow_center = GM_YAW_ZERO - (chassis_change_forward_back ? 4096 : 0);
				if(fabs((GMY.RxMsgC6x0.angle - chassis_follow_center) * 360 / 8192.0f) < 45)
				{
					change_forward_back_step = 0;
				}
				break;
			}
			default: change_forward_back_step = 0; break;
		}
		
		change_forward_back_rcd = chassis_change_forward_back;
	}
}

void ShootOneBullet(uint8_t state)
{
	#ifndef USE_HEAT_LIMIT
	STIR.TargetAngle -= STIR_STEP_ANGLE;
	#else
	uint8_t cdflag0 = 0;
	uint8_t cdflag1 = 0;
	if(state == 0)
	{
		cdflag0 = (maxHeat0 - fakeHeat0 < 20 && !burst) ? 1 : 0;
		if(!cdflag0 && ShootState)
		{
			STIR.TargetAngle -= STIR_STEP_ANGLE;
			fakeHeat0 += 10;
			auto_counter_heat0 = 2000;
			shoot_cd = cur_cd;
		}
	}
	if(state == 1)
	{
		cdflag1 = (maxHeat1 - fakeHeat1 < 105 && !burst) ? 1 : 0;
		if(!cdflag1 && ShootState)
		{
			STIR.TargetAngle -= STIR_STEP_ANGLE;
			fakeHeat1 += 100;
			auto_counter_heat1 = 200;
			shoot_cd = cur_cd;
		}
	}
	#endif
}

void ShootBullets(uint8_t state, uint8_t n)
{
	#ifndef USE_HEAT_LIMIT
	STIR.TargetAngle -= STIR_STEP_ANGLE;
	#else
	uint8_t cdflag0 = 0;
	uint8_t cdflag1 = 0;
	if(state == 0)
	{
		cdflag0 = (maxHeat0 - fakeHeat0 < n*20 && !burst) ? 1 : 0;
		if(!cdflag0 && ShootState)
		{
			STIR.TargetAngle -= n*STIR_STEP_ANGLE;
			fakeHeat0 += n*10;
			auto_counter_heat0 = 2000;
			shoot_cd = cur_cd;
		}
	}
	if(state == 1)
	{
		cdflag1 = (maxHeat1 - fakeHeat1 < n*105 && !burst) ? 1 : 0;
		if(!cdflag1 && ShootState)
		{
			STIR.TargetAngle -= n*STIR_STEP_ANGLE;
			fakeHeat1 += n*100;
			auto_counter_heat1 = 200;
			shoot_cd = cur_cd;
		}
	}
	#endif
}

void Bullet_Block_Handler(void)
{
	OnePush(STIR.RxMsgC6x0.moment < -8000,
	{
		STIR.TargetAngle += 1.4 * STIR_STEP_ANGLE;
		auto_counter_stir = 100;
		block_flag = 1;
		ShootState = 0;
	});
	if(block_flag == 1)
	{
		OnePush(auto_counter_stir==0,
		{
			if(STIR.RxMsgC6x0.moment > -500)
			{	
				STIR.TargetAngle -= 0.4 * STIR_STEP_ANGLE;
				ShootState = 1;
				block_flag = 0;
			}
			else
			{
				STIR.TargetAngle += STIR_STEP_ANGLE;
				auto_counter_stir = 100;
			}
		});
	}
}


void Climb_Handler()
{
	if(ClimbF_first_enter == 1)
	{
		CLIMB_F.TargetAngle -= 5;
		if(CLIMB_F.RxMsgC6x0.moment < -5000)
		{
			CLIMB_F.RealAngle=0;
			CLIMB_F.TargetAngle=15;
			ClimbF_first_enter=0;
		}
	}
	/*if(CLIMB_F.RxMsgC6x0.moment < -5000)
		{
			CLIMB_F.RealAngle=0;
			CLIMB_F.TargetAngle=15;
		}
	*/
	if(ClimbB_first_enter == 1)
	{
		CLIMB_B.TargetAngle += 5;
		if(CLIMB_B.RxMsgC6x0.moment > 5000)
		{
			CLIMB_B.RealAngle=0;
			CLIMB_B.TargetAngle=-15;
			ClimbB_first_enter=0;
		}
	}
	/*
	if(CLIMB_B.RxMsgC6x0.moment > 5000)
		{
			CLIMB_B.RealAngle=0;
			CLIMB_B.TargetAngle=-15;
		}
	*/
	
	if(ClimbF_first_enter == 0&&ClimbB_first_enter == 0)
	{
		MINMAX(CLIMB_F.TargetAngle,CLIMB_F_LIMIT_LOW,CLIMB_F_LIMIT_HIGH);
		MINMAX(CLIMB_B.TargetAngle,CLIMB_B_LIMIT_LOW,CLIMB_B_LIMIT_HIGH);
	}
	
}
void RefreshADC()
{
	for(uint16_t i=0;i<80;i++)
	{
		if(i%8==0)adf.value+=ADC_value[i];
		if(i%8==1)adb.value+=ADC_value[i];
	}
	adf.value=adf.value/41;
	adb.value=adb.value/41;
	
	disf=adf.value;
	disb=adb.value;
	if(disf>400) adf.flag=0;else adf.flag=1;
	if(disb>400) adb.flag=0;else adb.flag=1;
	
	
}

uint8_t Already_up=0;
uint8_t Already_down=0;
void Auto_Climb()
{
	if(AutoClimbing==1&&ChassisSpeedRef.forward_back_ref>0&&adf.flag==1&&adb.flag==1&&Already_up==0){
		CLIMB_F.TargetAngle=CLIMB_F_DOWN;
		Already_up=1;
	}
	else if(AutoClimbing==1&&ChassisSpeedRef.forward_back_ref>0&&adf.flag==1&&adb.flag==1&&hasReach(&CLIMB_F,30)&&Already_up==1){
		CLIMB_B.TargetAngle=CLIMB_B_DOWN;
		CLIMB_F.TargetAngle=CLIMB_F_UP;
		Already_up=2;
	}
	else if(AutoClimbing==1&&ChassisSpeedRef.forward_back_ref>0&&adf.flag==1&&adb.flag==1&&hasReach(&CLIMB_F,30)&&hasReach(&CLIMB_B,30)&&Already_up==2){
		CLIMB_B.TargetAngle=CLIMB_B_UP;
		Already_up=0;
		AutoClimbing=0;
	}
	else if(channellcol<-500&&ChassisSpeedRef.forward_back_ref<0&&adf.flag==1&&adb.flag==0&&hasReach(&CLIMB_F,30)&&hasReach(&CLIMB_B,30)&&Already_down==0){
		CLIMB_B.TargetAngle=CLIMB_B_DOWN;
		Already_down=1;
	}
	else if(channellcol<-500&&ChassisSpeedRef.forward_back_ref<0&&adf.flag==0&&adb.flag==0&&hasReach(&CLIMB_F,30)&&hasReach(&CLIMB_B,30)&&Already_down==1){
		CLIMB_B.TargetAngle=CLIMB_B_UP;
		CLIMB_F.TargetAngle=CLIMB_F_DOWN;
		Already_down=2;
	}
	else if(channellcol<-500&&ChassisSpeedRef.forward_back_ref<0&&adf.flag==0&&adb.flag==1&&hasReach(&CLIMB_F,30)&&hasReach(&CLIMB_B,30)&&Already_down==2){
		CLIMB_F.TargetAngle=CLIMB_F_UP;
		Already_down=0;
	}
		
}
void Gate_Handler(uint8_t gate_state)
{
	if(gate_first_enter == 1)
	{
		if(gate_state == GATE_CLOSE&&gate_first_enter==1&&gate_already_open==1)
		{
			gate_first_enter = 0;
			gate_already_open = 0;
		}
		else if(gate_state == GATE_OPEN)
		{
			GATE.TargetAngle -= 5;
			if(GATE.RxMsgC6x0.moment <- GATE_BLOCK_MOMENT_THRES)
			{
				GATE.RealAngle = 0;
				GATE.TargetAngle = 0;
				gate_first_enter = 0;
				gate_already_open = 1;
			}
		}
	}
	if(gate_first_enter == 0)
	{
		if(gate_state == GATE_OPEN&&gate_first_enter==0&&gate_already_open==0)
		{
			gate_first_enter = 1;
			gate_already_open = 1;
		}
		else if(gate_state == GATE_CLOSE)
		{
			GATE.TargetAngle += 5;
			if(GATE.RxMsgC6x0.moment > GATE_BLOCK_MOMENT_THRES)
			{
				GATE.RealAngle = 0;
				GATE.TargetAngle = 0;
				gate_first_enter = 1;
				gate_already_open = 0;
			}
		}
	}
}

void GMY_FINE_ADJUSTMENT_handler(uint8_t gmy_state)
{
	if(gmy_state == 0 && gmy_inposition == 0)//进入过程  //之后要在初始化前加一个判断是否是啮合状态
	{
		GMY.TargetAngle = 45; 
		if(GMY.RealAngle >= 29.5)
		{
			if(gfa0 == 0)
			{
			  GMY_FINE_ADJUSTMENT.TargetAngle -= 5;
			}
			if(GMY_FINE_ADJUSTMENT.RxMsgC6x0.moment <- GMY_FINE_ADJUSTMENT_BLOCK_MOMENT_THRES)
			{
				GMY_FINE_ADJUSTMENT.RealAngle = 0;
				GMY_FINE_ADJUSTMENT.TargetAngle = 0;
				gfa0 = 1;
			}
		}
		if(gfa0 == 1)
		{
		  GMY.TargetAngle = 0;
			GMY_FINE_ADJUSTMENT.TargetAngle  -= 45.0;
		}
		if(gfa0 == 1 && GMY.RealAngle <= 0.5)
		{
		  gmy_inposition = 1;
			gfa0 = 0;
			GMY_FINE_ADJUSTMENT.RealAngle = 0;
			GMY_FINE_ADJUSTMENT.TargetAngle = 0;
		}
	}
	if(gmy_state == 1 && gmy_inposition == 1)//离开过程
	{
		
	}
}

void Reset(uint8_t chassis_forward_back)
{
	ChassisTwistState = 0;
	chassis_lock = 0;
	gate_state = GATE_CLOSE;
	if(GM_Turn_back == 1)
	{
		chassis_follow_center -= 4096.0f;
	}
	GM_Turn_back = 0;
}

