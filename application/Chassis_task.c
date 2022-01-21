#include "Chassis_task.h"

PID    Chassis_3508[4]; //底盘电机PID参数

float  WheelSpeed[4];           //底盘单个电机目标速度
int    Chassis_3508_Current[4]; //底盘赋值电流
int    SPEED_X,SPEED_Y,SPEED_W_Z;

//步兵功率
int    POWER_INITIAL = 40;
int		 POWER_POWER_FIRST[3] = {60,80,100};
int		 POWER_HP_RFIRST[3] = {45,50,55};

//功率限制相关
float  Power_Buffer=0;
float  Capacity_Voltage=0.0;
int		 Power_judge;

//底盘最大速度
int CHASSIS_SPEED_X_MAX = 500;
int CHASSIS_SPEED_Y_MAX = 500;
int CHASSIS_SPEED_W_Z_MAX = 500;

//底盘速度目标值滤波器值
int Filter_value[3];                     //滤波后的值

float    FILTER_A    =    0.2; //滤波系数

//跟随pid
PID CHASSIS_PID_FOLLOW;

/**
  * @brief          单电机速度超阈值处理函数
  * @param[in]      Speed_3508:   3508解算后的目标速度
  * @param[out]     返回值代表是否超过3508阈值
  * @author     		Chen
  */
int LIMIT_3508(int Speed_3508,int Speed_MAX)
{
		if(Speed_3508 > Speed_MAX)
		{
			Speed_3508 = Speed_MAX;
			return  1;
		}
		else if(Speed_3508 < -Speed_MAX)
		{
			Speed_3508 = -Speed_MAX;
			return  1;
		}
		else
		{
			return  0;
			
		}
}

/*
*/
/**
  * @brief          全向轮与麦轮底盘解算 CHASSIS_MODE： 1全向轮 2麦轮
  * @param[in]      SPEED_X: X速度
  * @param[in]      SPEED_Y: Y速度
  * @param[in]      SPEED_W_Z: 旋转速度
  * @author     		Chen
  */
void ChassisMove(int SPEED_X , int SPEED_Y , int SPEED_W_Z)
{
	 int i;
	 #if CHASSIS_MODE == 1
		 WheelSpeed[0] = 10*(-SPEED_X + SPEED_Y + SPEED_W_Z);
		 WheelSpeed[1] = 10*(SPEED_X + SPEED_Y + SPEED_W_Z);
		 WheelSpeed[2] = 10*(-SPEED_X - SPEED_Y + SPEED_W_Z);
		 WheelSpeed[3] = 10*(SPEED_X - SPEED_Y + SPEED_W_Z);
	 #else 
		 WheelSpeed[0] = 10*(SPEED_X - SPEED_Y + SPEED_W_Z);
		 WheelSpeed[1] = 10*( + SPEED_Y + SPEED_W_Z);
		 WheelSpeed[2] = 10*(-SPEED_X + SPEED_Y + SPEED_W_Z);
		 WheelSpeed[3] = 10*( - SPEED_Y + SPEED_W_Z);
	 #endif
	 for( i = 0 ; i<4 ; i++)
	 {
				LIMIT_3508(WheelSpeed[i],10000);
				Chassis_3508_Current[i] = PID_3508_Chassis(WheelSpeed[i], M3508_Receive[i].speed, Chassis_3508[i]) ;
	 }
}


/**
  * @brief          底盘电机初始化
  * @author     		Chen
  */
void Chaiss_PID_Init()
{
	
#if CHASSIS_PID_MODE == 1
	
	Chassis_3508[0].P = 3;	
//	Chassis_3508[0].P = 20; //拨弹轮测试
	
	Chassis_3508[1].P = 3;
	Chassis_3508[2].P = 3;
	Chassis_3508[3].P = 3;
	
	Chassis_3508[0].I = 0; 
//	Chassis_3508[0].I = 2;	//拨弹轮测试
	Chassis_3508[1].I = 0;
	Chassis_3508[2].I = 0;
	Chassis_3508[3].I = 0;
	
	Chassis_3508[0].D = 0;
	Chassis_3508[1].D = 0;
	Chassis_3508[2].D = 0;
	Chassis_3508[3].D = 0;
	
	CHASSIS_PID_FOLLOW.P = 0.1;
	CHASSIS_PID_FOLLOW.I = 0;
	CHASSIS_PID_FOLLOW.D = 0.5;
#else
	
#endif
	

}
int key_test;
/**
  * @brief          键鼠模式控制
  * @param[in]      RC_MODE   传入遥控器拨杆值   2 为遥控器模式  3 为键鼠模式
  * @author     		Chen
  */
void KM_MODE_CONTROL(void)
{
		uint16_t 	KEY_Value;
		uint16_t  KEY_NUM[4];
		uint16_t  RESET;
	  key_test = rc_ctrl.key.v;
		KEY_Value = rc_ctrl.key.v;
		KEY_NUM[0] = KEY_Value&0x0001;
		KEY_NUM[1] = KEY_Value&0x0002;
		KEY_NUM[2] = KEY_Value&0x0004;
		KEY_NUM[3] = KEY_Value&0x0008;
		RESET = KEY_Value&0x0120;
		
	  if(RESET == 0x0120)
		{
			NVIC_SystemReset();
		}
		else
		{
					if(KEY_NUM[0] == 0x0001)
					{
								SPEED_X = CHASSIS_SPEED_X_MAX;
					}
					else if(KEY_NUM[1] == 0x0002)
					{
								SPEED_X = CHASSIS_SPEED_X_MAX;				
					}
					else
					{
								SPEED_X = 0;							
					}
							
					if(KEY_NUM[2] == 0x0004)
					{
								SPEED_Y = CHASSIS_SPEED_Y_MAX;
					}
					else if(KEY_NUM[3] == 0x0008)
					{
								SPEED_Y = -CHASSIS_SPEED_Y_MAX;				
					}
					else
					{
								SPEED_Y = 0;							
					}
		}
		
		
		SPEED_W_Z = Chassis_Follow(); //wz由跟随给出	
}

/**
  * @brief          遥控器到底盘目标值转化
  * @param[in]      RC_MODE   传入遥控器拨杆值   1 为遥控器模式  3 为键鼠模式
  * @author     		Chen
  */
void RC_TO_CHAISS(int  RC_MODE)
{		
		if(RC_MODE == 1)
		{
				SPEED_X 	= CHASSIS_SPEED_X_MAX*rc_ctrl.rc.ch[3]/660;
				SPEED_Y 	= CHASSIS_SPEED_X_MAX*rc_ctrl.rc.ch[2]/660;
				//SPEED_W_Z = Chassis_Follow();		
				SPEED_W_Z = CHASSIS_SPEED_X_MAX*rc_ctrl.rc.ch[0]/660;
			
			  FILTER_A = 0.2;
		}
		else if(RC_MODE == 3)
		{
				KM_MODE_CONTROL();	
			  FILTER_A = 0.2;			
		}
}


/**
  * @brief          功率限制
  * @param[in]      POWER_LIMIT_MODE   功率限制模式   1 为手动控制模式  2 为裁判系统自动读取模式  3  为无限制模式
  * @param[in]      SPEED_X，SPEED_Y，SPEED_W_Z  传入待计算的底盘目标速度
  * @param[in]      POWER  功率限制值
  * @author     		Chen
  */
void POWER_LIMIT(int  POWER_LIMIT_MODE	,int *SPEED_X	,	int *SPEED_Y	,	int *SPEED_W_Z	,int POWER)
{
	 int POWER_MID;
	 float scale;
	
	 if(POWER_LIMIT_MODE == 3)
	 {
		 return ;
	 }
	 
	 switch(POWER_LIMIT_MODE)
	 {
		 case 1 : POWER_MID = Power_judge;break;
		 case 2 :	POWER_MID = POWER			 ;break;
		 default: break;
	 }
	 //计算比例
	 scale = POWER_LIMIT_BEHAVE(	POWER_MID , Power_Buffer , Capacity_Voltage);
	 //按比例缩小速度
	 *SPEED_X 		= scale*(*SPEED_X);
	 *SPEED_Y 		= scale*(*SPEED_Y);
	 *SPEED_W_Z  = scale*(*SPEED_W_Z);
	 //对目标值做低通滤波
	 *SPEED_X 	 = 	Filter_one(*SPEED_X,Filter_value);
	 *SPEED_Y 	 = 	Filter_one(*SPEED_Y,Filter_value+1);
	 *SPEED_W_Z = 	Filter_one(*SPEED_W_Z,Filter_value+2);
	 
	 //usart1_report_imu(*SPEED_X,*SPEED_Y,*SPEED_W_Z,10,100);


}

/**
  * @brief          功率限制具体计算
  * @param[in]      POWER  								功率限制值
  * @param[in]      Power_Buffer  				缓冲能量值
  * @param[in]      capacity_Voltage  		超级电容剩余电量值
  * @author     		Chen
  */
float POWER_LIMIT_BEHAVE(int POWER , float Power_Buffer , float Capacity_Voltage)
{
		float  Scale=1;
		
		return Scale;
}


/**
  * @brief          一阶低通滤波
  * @param[in]      Value  							采样值更新
  * @param[in]      Current_outvalue  	本次滤波输出值
  * @author     		Chen
  */
int Filter_one(int Value,int *Current_outvalue)
{
	int out;
	out = FILTER_A*Value + (1-FILTER_A)*(*Current_outvalue);
	*Current_outvalue = out;
	return out;
}


/**
  * @brief          底盘跟随云台
  * @author     		Chen
 */
int Chassis_Follow()
{
		int value;
		value = -Chassis_Follow_PID( Gimbal_Init_Encoder_Value , YAW_ANGEL , CHASSIS_PID_FOLLOW , 100) ;
	  return value;
}


/**
  * @brief          底盘中断计算任务
  * @author     		Chen
 */
void Chassis_task(void)
{																																																				
		RC_TO_CHAISS(rc_ctrl.rc.s[1])	;																																											  //切换底盘控制模式	并计算底盘总速度
		POWER_LIMIT(	POWER_Lmt_MODE	, &SPEED_X	,	&SPEED_Y	,	&SPEED_W_Z	, POWER_INITIAL );																//功率限制
		ChassisMove(SPEED_X,SPEED_Y,SPEED_W_Z);																																								//底盘电机速度值与电流值计算
		CAN_cmd_chassis(Chassis_3508_Current[0], Chassis_3508_Current[1], Chassis_3508_Current[2], Chassis_3508_Current[3]); 	//电机赋值			

}








