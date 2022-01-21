#include "gimbal.h"



int Hoare_Switch  = 1;
int Manual_Switch = 1;
int Gimbal_Init_Encoder_Value;
PID GIMBLE_PID_YAW_Speed;
PID GIMBLE_PID_YAW_Angel_6020;
PID GIMBLE_PID_YAW_Angel_Gyro;

PID GIMBLE_PID_PITCH_Speed;
PID GIMBLE_PID_PITCH_Angel_6020;
PID GIMBLE_PID_PITCH_Angel_Gyro;

int GIMBAL_CONTROL_MODE = 1;

int GIMBAL_YAW_Initial;
/**
 * @brief  云台PID参数初始化
 * 
*/
float p_speed,i_speed,d_speed;
float p_angel,i_angel,d_angel;
int gimbal_pid_init()
{
//		GIMBLE_PID_YAW_Speed.P = p_speed;
//		GIMBLE_PID_YAW_Speed.I = i_speed;
//		GIMBLE_PID_YAW_Speed.D = d_speed;
//	  /*    YAW速度环   */
//	  GIMBLE_PID_YAW_Angel_6020.P = p_angel;
//	  GIMBLE_PID_YAW_Angel_6020.I = i_angel;
//	  GIMBLE_PID_YAW_Angel_6020.D = d_angel;
//	  /*    YAW 6020 角度环   */
	
		GIMBLE_PID_YAW_Speed.P = 20;
		GIMBLE_PID_YAW_Speed.I = 0;
		GIMBLE_PID_YAW_Speed.D = 2;
	  /*    YAW速度环   */
	  GIMBLE_PID_YAW_Angel_6020.P = 10;
	  GIMBLE_PID_YAW_Angel_6020.I = 0;
	  GIMBLE_PID_YAW_Angel_6020.D = 0;
	  /*    YAW 6020 角度环   */
	
	
	  GIMBLE_PID_YAW_Angel_Gyro.P = 150;
	  GIMBLE_PID_YAW_Angel_Gyro.I = 25;
	  GIMBLE_PID_YAW_Angel_Gyro.D = 19;
	  /*    YAW 陀螺仪 角度环   */
	
	 
	
		GIMBLE_PID_PITCH_Speed.P = 15;
		GIMBLE_PID_PITCH_Speed.I = 0;
		GIMBLE_PID_PITCH_Speed.D = 1;
	  /*    PITCH速度环   */
	  GIMBLE_PID_PITCH_Angel_6020.P = 10;
	  GIMBLE_PID_PITCH_Angel_6020.I = 0.2;
	  GIMBLE_PID_PITCH_Angel_6020.D = 5;
	  /*    PITCH 6020 角度环   */

//		GIMBLE_PID_PITCH_Speed.P = p_speed;
//		GIMBLE_PID_PITCH_Speed.I = i_speed;
//		GIMBLE_PID_PITCH_Speed.D = d_speed;
//	  /*    PITCH速度环   */
//	  GIMBLE_PID_PITCH_Angel_6020.P = p_angel;
//	  GIMBLE_PID_PITCH_Angel_6020.I = i_angel;
//	  GIMBLE_PID_PITCH_Angel_6020.D = d_angel;
//	  /*    PITCH 6020 角度环   */




	  GIMBLE_PID_PITCH_Angel_Gyro.P = 0.3;
	  GIMBLE_PID_PITCH_Angel_Gyro.I = 0.02;
	  GIMBLE_PID_PITCH_Angel_Gyro.D = 0.05;
	  /*    PITCH 陀螺仪 角度环   */

	
	
		return 1;

}

/**
 * @brief  云台校准
 * @exp    云台上电后会进入无力状态，   当云台转动至霍尔传感器时，云台保存此时的云台编码器值
 * @param  Gimbal_Init_Encoder_Value    云台初始值
 * @param  Hoare_Switch    							霍尔开关值										 低触发
 * @param  Manual_Switch    						手动校准开关值  *防止霍尔失效* 低触发
*/
int gimbal_cali(void)
{
		Hoare_Switch = 0 ; //模拟霍尔开关打开
		while(Hoare_Switch && Manual_Switch)
		{
			;
		}
		Gimbal_Init_Encoder_Value = YAW_6020_DATA.angel;
		return 1;
}

/**
 * @brief  云台初始化
*/
int gimbal_init(void)
{	
		while( PITCH_6020_DATA.angel == 0)
		{
				HAL_Delay(100);
			 	HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_12);
		}
		gimbal_pid_init();
		gimbal_cali();
		
	  Gimbal_Angel_handle();
		
		YAW_remote   = YAW_ANGEL;
	  PITCH_remote = PITCH_ANGEL;

//	  YAW_remote   = YAW_CTNU;
//	  PITCH_remote = PITCH_CTNU;
		
		return 1;
}


/**
* @brief      		云台YAW串级PID实现 *最简单控制方式*
* @param[in]  		Set_Angel   							目标角度
* @param[in]  		Gimbal_Control_Mode   		云台控制模式  1 为6020目标控制  2  为陀螺仪目标控制
* @param[out]  		Set   										6020电压赋值
*/
int PID_YAW_GIMBAL_DUO( int Set_Angel  , int Gimbal_Control_Mode )
{
	 int speed_goal;
	 int Set;
	 if(Gimbal_Control_Mode == 1)
	 {
		 speed_goal = PID_GIMBAL_ANGEL(Set_Angel , YAW_ANGEL , GIMBLE_PID_YAW_Angel_6020 ,10000 );//YAW_6020_DATA.angel 范围0 - 8198
	 }
	 else if(Gimbal_Control_Mode == 2)
	 {
		 speed_goal = -PID_GIMBAL_ANGEL(0.1*Set_Angel , YAW_CTNU , GIMBLE_PID_YAW_Angel_Gyro ,10000 );		 //YAW 范围0 - 360
	 }
	 else if(Gimbal_Control_Mode == 3)
	 {
		 speed_goal = PID_GIMBAL_ANGEL( 0.1*Set_Angel , yaw_angel , GIMBLE_PID_PITCH_Angel_Gyro ,10000 );		 //YAW 范围0 - 360 
	 }

   Set 				= PID_GIMBAL_SPEED(speed_goal, YAW_6020_DATA.speed , GIMBLE_PID_YAW_Speed);
	 return Set	;	
}

/**
* @brief      		云台PITCH串级PID实现 *最简单控制方式*
* @param[in]  		Set_Angel   							目标角度
* @param[in]  		Gimbal_Control_Mode   		云台控制模式  1 为6020目标控制  2  为陀螺仪目标控制
* @param[out]  		Set   										6020电压赋值
*/
int PID_PITCH_GIMBAL_DUO( int Set_Angel  , int Gimbal_Control_Mode )
{
	 int speed_goal;
	 int Set;
	 if(Gimbal_Control_Mode == 1)
	 {
		 speed_goal = PID_GIMBAL_ANGEL( Set_Angel , PITCH_ANGEL , GIMBLE_PID_PITCH_Angel_6020 ,10000 );//YAW_6020_DATA.angel 范围0 - 8198
	 }
	 else if(Gimbal_Control_Mode == 2)
	 {
		 speed_goal = PID_GIMBAL_ANGEL( 0.1*Set_Angel , PITCH_CTNU , GIMBLE_PID_PITCH_Angel_Gyro ,10000 );		 //YAW 范围0 - 360
	 }
	 else if(Gimbal_Control_Mode == 3)
	 {
		 speed_goal = -PID_GIMBAL_ANGEL( 0.1*Set_Angel , pitch_angel , GIMBLE_PID_PITCH_Angel_Gyro ,10000 );		 //YAW 范围0 - 360 
	 }
   Set 	= PID_GIMBAL_SPEED(speed_goal, PITCH_6020_DATA.speed , GIMBLE_PID_PITCH_Speed);
	 return Set	;	
}


long int YAW_remote,PITCH_remote; //遥控器控制的目标值
long int YAW_ANGEL,PITCH_ANGEL;   //6020编码器连续值

/**
 * @brief      云台编码器角度值连续计算
*/
int Gimbal_Angel_handle()
{
	 static int YAW_TEMP,PITCH_TEMP; //上次的6020角度值
	 static int i_yaw,i_pitch;
	
	 if(YAW_TEMP - YAW_6020_DATA.angel > 6000) 
	 {
			i_yaw++;
	 }	 
	 else if(YAW_6020_DATA.angel - YAW_TEMP > 6000)
	 {
		 i_yaw--;
	 }
	 
	 if(PITCH_TEMP - PITCH_6020_DATA.angel > 6000) 
	 {
			i_pitch++;
	 }	 
	 else if(PITCH_6020_DATA.angel - PITCH_TEMP > 6000)
	 {
		 i_pitch--;
	 }
	 
	 PITCH_ANGEL = PITCH_6020_DATA.angel+i_pitch*8198;		 
	 
	 YAW_ANGEL = YAW_6020_DATA.angel+i_yaw*8198;		 

	 YAW_TEMP = YAW_6020_DATA.angel;
	 PITCH_TEMP = PITCH_6020_DATA.angel;
	 return 1;
}
/**
 * @brief      云台遥控器值获取
 * @param[in]  Mode 遥控器模式选择  3 为键鼠 1 为遥控器
*/
int Gimbal_remote(int Mode)
{
		switch(Mode)
		{
			case 3 : 
		YAW_remote   +=10*rc_ctrl.mouse.x;
		PITCH_remote +=10*rc_ctrl.mouse.y;
			break;

			case 1 : 
		//YAW_remote 	 +=0.1*rc_ctrl.rc.ch[0];
		PITCH_remote +=0.1*rc_ctrl.rc.ch[1];
			break;
			
			default : break;
			
		}
		return 1;
	
}

int YAW_CURRENT,PITCH_CURRENT;

/**
 * @brief  云台任务
*/
int GIMBAL_TASK(void)
{
		Gimbal_Angel_handle();
	
		Gimbal_remote(rc_ctrl.rc.s[1]);
		YAW_CURRENT = PID_YAW_GIMBAL_DUO(	YAW_remote , GIMBAL_CONTROL_MODE );
		PITCH_CURRENT = PID_PITCH_GIMBAL_DUO(  PITCH_remote  , GIMBAL_CONTROL_MODE );		
		CAN_cmd_gimbal( YAW_CURRENT , Trigger_Current , 0 , 0 );
		CAN2_cmd_gimbal(PITCH_CURRENT);			
		return 0 ;
}





