#include "shoot_task.h"


PID SHOOT_PID_LEFT;
PID SHOOT_PID_RIGHT;


PID FRONT_PID; 	//预置轮
PID TRIGGER_PID;//拨弹轮

int SPEED_LEFT;
int SPEED_RIGHT;
/**
 * @brief  发射初始化程序
*/
float P = 25 ,I = 0.2 ,D;
int speed_t = 700;
void Shoot_Init()
{
	SHOOT_PID_LEFT.P = 1.0;
	SHOOT_PID_LEFT.I = 0.1;
	SHOOT_PID_LEFT.D = 0.0;
	
	SHOOT_PID_RIGHT.P = 1.0;
	SHOOT_PID_RIGHT.I = 0.1;
	SHOOT_PID_RIGHT.D = 0.0;
	
	TRIGGER_PID.P = P;
	TRIGGER_PID.I = I;
	TRIGGER_PID.D = D;
	
	FRONT_PID.P = 5.0;
	FRONT_PID.I = 0.0;
	FRONT_PID.D = 0.0;
	

}

int SHOT_LEFT_CURRENT;
int SHOT_RIGHT_CURRENT;


/**
 * @brief  发射摩擦轮控制
*/
void Shoot_caculate(int speed_left,int speed_right)
{
		SHOT_LEFT_CURRENT  = PID_3508_Chassis( speed_left  ,  SHOOT_MOTOR_LEFT.speed  , SHOOT_PID_LEFT  );	
		SHOT_RIGHT_CURRENT = PID_3508_Chassis( -speed_right ,  SHOOT_MOTOR_RIGHT.speed , SHOOT_PID_RIGHT );	
}

int Trigger_Current;
/**
 * @brief  			拨弹轮控制
 * @param[in]   Up_flag  		 拨弹控制   1 开启拨弹  0 关闭拨弹
*/
void Trigger_control(int Up_flag,int speed)
{
		if(Up_flag == 1)
		{
				Trigger_Current = PID_3508_Chassis( -speed ,  TRRIGLE_MOTOR_RIGHT.speed , TRIGGER_PID );
		}
}


/**
 * @brief  预置轮控制
 * @param[in]   Fire_flag  		 开火控制    
 * @param[in]   Fire_Max_Frec  开火最大周期   单位  10ms
 * @param[in]   speed  				 预置轮转速     
*/
int Front_Current;
int Front_control(int Fire_flag,int Fire_Max_Frec , int speed)
{
		Front_Current = PID_3508_Chassis( speed ,  FRONT_MOTOR.speed , FRONT_PID );	
//			static int count = 0;
//			static int time = 0;
//			static int flag = 0;
//			if(count>0)
//			{
//					count--;
//			}
//			
//			if(count<1)
//			{
//					if(Fire_flag == 1)
//					{
//						 flag = 1;
//						 count = Fire_Max_Frec;
//					}
//		  }
//			if(flag == 1)
//			{
//				  time ++ ;
//					Front_Current = PID_3508_Chassis( speed ,  FRONT_MOTOR.speed , FRONT_PID );	

//					if(time>50)
//					{
//							flag = 0;
//							time = 0;
//					}
//					
//			}
//			else if(flag == 0)
//			{
//						Front_Current = PID_3508_Chassis( 0,  FRONT_MOTOR.speed , FRONT_PID );				
//			}
			return 0;
}


/**
 * @brief 		 遥控器数值获取 
 * @param[in]  Mode 遥控器模式选择  3 为键鼠 1 为遥控器
*/
int Shoot_remote_control(int MOD)
{
		switch(MOD)
		{
			case 3 : 
			
			break;

			case 1 : 
			
			break;
			
			default : break;
		}
		return 1;

}


/**
* @brief 发射任务
*/
void Shoot_Task()
{
		Shoot_remote_control(rc_ctrl.rc.s[1]);
	  TRIGGER_PID.P = P;
	  TRIGGER_PID.I = I;
	  TRIGGER_PID.D = D;

		Trigger_control(1,speed_t);
	
		Front_control(1,10,1000);
	
		Shoot_caculate(SPEED_LEFT,SPEED_RIGHT);
	
		CAN2_cmd_shoot( SHOT_LEFT_CURRENT , SHOT_RIGHT_CURRENT , Front_Current , 0 );
}











