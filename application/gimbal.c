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
 * @brief  ��̨PID������ʼ��
 * 
*/
float p_speed,i_speed,d_speed;
float p_angel,i_angel,d_angel;
int gimbal_pid_init()
{
//		GIMBLE_PID_YAW_Speed.P = p_speed;
//		GIMBLE_PID_YAW_Speed.I = i_speed;
//		GIMBLE_PID_YAW_Speed.D = d_speed;
//	  /*    YAW�ٶȻ�   */
//	  GIMBLE_PID_YAW_Angel_6020.P = p_angel;
//	  GIMBLE_PID_YAW_Angel_6020.I = i_angel;
//	  GIMBLE_PID_YAW_Angel_6020.D = d_angel;
//	  /*    YAW 6020 �ǶȻ�   */
	
		GIMBLE_PID_YAW_Speed.P = 20;
		GIMBLE_PID_YAW_Speed.I = 0;
		GIMBLE_PID_YAW_Speed.D = 2;
	  /*    YAW�ٶȻ�   */
	  GIMBLE_PID_YAW_Angel_6020.P = 10;
	  GIMBLE_PID_YAW_Angel_6020.I = 0;
	  GIMBLE_PID_YAW_Angel_6020.D = 0;
	  /*    YAW 6020 �ǶȻ�   */
	
	
	  GIMBLE_PID_YAW_Angel_Gyro.P = 150;
	  GIMBLE_PID_YAW_Angel_Gyro.I = 25;
	  GIMBLE_PID_YAW_Angel_Gyro.D = 19;
	  /*    YAW ������ �ǶȻ�   */
	
	 
	
		GIMBLE_PID_PITCH_Speed.P = 15;
		GIMBLE_PID_PITCH_Speed.I = 0;
		GIMBLE_PID_PITCH_Speed.D = 1;
	  /*    PITCH�ٶȻ�   */
	  GIMBLE_PID_PITCH_Angel_6020.P = 10;
	  GIMBLE_PID_PITCH_Angel_6020.I = 0.2;
	  GIMBLE_PID_PITCH_Angel_6020.D = 5;
	  /*    PITCH 6020 �ǶȻ�   */

//		GIMBLE_PID_PITCH_Speed.P = p_speed;
//		GIMBLE_PID_PITCH_Speed.I = i_speed;
//		GIMBLE_PID_PITCH_Speed.D = d_speed;
//	  /*    PITCH�ٶȻ�   */
//	  GIMBLE_PID_PITCH_Angel_6020.P = p_angel;
//	  GIMBLE_PID_PITCH_Angel_6020.I = i_angel;
//	  GIMBLE_PID_PITCH_Angel_6020.D = d_angel;
//	  /*    PITCH 6020 �ǶȻ�   */




	  GIMBLE_PID_PITCH_Angel_Gyro.P = 0.3;
	  GIMBLE_PID_PITCH_Angel_Gyro.I = 0.02;
	  GIMBLE_PID_PITCH_Angel_Gyro.D = 0.05;
	  /*    PITCH ������ �ǶȻ�   */

	
	
		return 1;

}

/**
 * @brief  ��̨У׼
 * @exp    ��̨�ϵ����������״̬��   ����̨ת��������������ʱ����̨�����ʱ����̨������ֵ
 * @param  Gimbal_Init_Encoder_Value    ��̨��ʼֵ
 * @param  Hoare_Switch    							��������ֵ										 �ʹ���
 * @param  Manual_Switch    						�ֶ�У׼����ֵ  *��ֹ����ʧЧ* �ʹ���
*/
int gimbal_cali(void)
{
		Hoare_Switch = 0 ; //ģ��������ش�
		while(Hoare_Switch && Manual_Switch)
		{
			;
		}
		Gimbal_Init_Encoder_Value = YAW_6020_DATA.angel;
		return 1;
}

/**
 * @brief  ��̨��ʼ��
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
* @brief      		��̨YAW����PIDʵ�� *��򵥿��Ʒ�ʽ*
* @param[in]  		Set_Angel   							Ŀ��Ƕ�
* @param[in]  		Gimbal_Control_Mode   		��̨����ģʽ  1 Ϊ6020Ŀ�����  2  Ϊ������Ŀ�����
* @param[out]  		Set   										6020��ѹ��ֵ
*/
int PID_YAW_GIMBAL_DUO( int Set_Angel  , int Gimbal_Control_Mode )
{
	 int speed_goal;
	 int Set;
	 if(Gimbal_Control_Mode == 1)
	 {
		 speed_goal = PID_GIMBAL_ANGEL(Set_Angel , YAW_ANGEL , GIMBLE_PID_YAW_Angel_6020 ,10000 );//YAW_6020_DATA.angel ��Χ0 - 8198
	 }
	 else if(Gimbal_Control_Mode == 2)
	 {
		 speed_goal = -PID_GIMBAL_ANGEL(0.1*Set_Angel , YAW_CTNU , GIMBLE_PID_YAW_Angel_Gyro ,10000 );		 //YAW ��Χ0 - 360
	 }
	 else if(Gimbal_Control_Mode == 3)
	 {
		 speed_goal = PID_GIMBAL_ANGEL( 0.1*Set_Angel , yaw_angel , GIMBLE_PID_PITCH_Angel_Gyro ,10000 );		 //YAW ��Χ0 - 360 
	 }

   Set 				= PID_GIMBAL_SPEED(speed_goal, YAW_6020_DATA.speed , GIMBLE_PID_YAW_Speed);
	 return Set	;	
}

/**
* @brief      		��̨PITCH����PIDʵ�� *��򵥿��Ʒ�ʽ*
* @param[in]  		Set_Angel   							Ŀ��Ƕ�
* @param[in]  		Gimbal_Control_Mode   		��̨����ģʽ  1 Ϊ6020Ŀ�����  2  Ϊ������Ŀ�����
* @param[out]  		Set   										6020��ѹ��ֵ
*/
int PID_PITCH_GIMBAL_DUO( int Set_Angel  , int Gimbal_Control_Mode )
{
	 int speed_goal;
	 int Set;
	 if(Gimbal_Control_Mode == 1)
	 {
		 speed_goal = PID_GIMBAL_ANGEL( Set_Angel , PITCH_ANGEL , GIMBLE_PID_PITCH_Angel_6020 ,10000 );//YAW_6020_DATA.angel ��Χ0 - 8198
	 }
	 else if(Gimbal_Control_Mode == 2)
	 {
		 speed_goal = PID_GIMBAL_ANGEL( 0.1*Set_Angel , PITCH_CTNU , GIMBLE_PID_PITCH_Angel_Gyro ,10000 );		 //YAW ��Χ0 - 360
	 }
	 else if(Gimbal_Control_Mode == 3)
	 {
		 speed_goal = -PID_GIMBAL_ANGEL( 0.1*Set_Angel , pitch_angel , GIMBLE_PID_PITCH_Angel_Gyro ,10000 );		 //YAW ��Χ0 - 360 
	 }
   Set 	= PID_GIMBAL_SPEED(speed_goal, PITCH_6020_DATA.speed , GIMBLE_PID_PITCH_Speed);
	 return Set	;	
}


long int YAW_remote,PITCH_remote; //ң�������Ƶ�Ŀ��ֵ
long int YAW_ANGEL,PITCH_ANGEL;   //6020����������ֵ

/**
 * @brief      ��̨�������Ƕ�ֵ��������
*/
int Gimbal_Angel_handle()
{
	 static int YAW_TEMP,PITCH_TEMP; //�ϴε�6020�Ƕ�ֵ
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
 * @brief      ��̨ң����ֵ��ȡ
 * @param[in]  Mode ң����ģʽѡ��  3 Ϊ���� 1 Ϊң����
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
 * @brief  ��̨����
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





