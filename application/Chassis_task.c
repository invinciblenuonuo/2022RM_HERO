#include "Chassis_task.h"

PID    Chassis_3508[4]; //���̵��PID����

float  WheelSpeed[4];           //���̵������Ŀ���ٶ�
int    Chassis_3508_Current[4]; //���̸�ֵ����
int    SPEED_X,SPEED_Y,SPEED_W_Z;

//��������
int    POWER_INITIAL = 40;
int		 POWER_POWER_FIRST[3] = {60,80,100};
int		 POWER_HP_RFIRST[3] = {45,50,55};

//�����������
float  Power_Buffer=0;
float  Capacity_Voltage=0.0;
int		 Power_judge;

//��������ٶ�
int CHASSIS_SPEED_X_MAX = 500;
int CHASSIS_SPEED_Y_MAX = 500;
int CHASSIS_SPEED_W_Z_MAX = 500;

//�����ٶ�Ŀ��ֵ�˲���ֵ
int Filter_value[3];                     //�˲����ֵ

float    FILTER_A    =    0.2; //�˲�ϵ��

//����pid
PID CHASSIS_PID_FOLLOW;

/**
  * @brief          ������ٶȳ���ֵ������
  * @param[in]      Speed_3508:   3508������Ŀ���ٶ�
  * @param[out]     ����ֵ�����Ƿ񳬹�3508��ֵ
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
  * @brief          ȫ���������ֵ��̽��� CHASSIS_MODE�� 1ȫ���� 2����
  * @param[in]      SPEED_X: X�ٶ�
  * @param[in]      SPEED_Y: Y�ٶ�
  * @param[in]      SPEED_W_Z: ��ת�ٶ�
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
  * @brief          ���̵����ʼ��
  * @author     		Chen
  */
void Chaiss_PID_Init()
{
	
#if CHASSIS_PID_MODE == 1
	
	Chassis_3508[0].P = 3;	
//	Chassis_3508[0].P = 20; //�����ֲ���
	
	Chassis_3508[1].P = 3;
	Chassis_3508[2].P = 3;
	Chassis_3508[3].P = 3;
	
	Chassis_3508[0].I = 0; 
//	Chassis_3508[0].I = 2;	//�����ֲ���
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
  * @brief          ����ģʽ����
  * @param[in]      RC_MODE   ����ң��������ֵ   2 Ϊң����ģʽ  3 Ϊ����ģʽ
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
		
		
		SPEED_W_Z = Chassis_Follow(); //wz�ɸ������	
}

/**
  * @brief          ң����������Ŀ��ֵת��
  * @param[in]      RC_MODE   ����ң��������ֵ   1 Ϊң����ģʽ  3 Ϊ����ģʽ
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
  * @brief          ��������
  * @param[in]      POWER_LIMIT_MODE   ��������ģʽ   1 Ϊ�ֶ�����ģʽ  2 Ϊ����ϵͳ�Զ���ȡģʽ  3  Ϊ������ģʽ
  * @param[in]      SPEED_X��SPEED_Y��SPEED_W_Z  ���������ĵ���Ŀ���ٶ�
  * @param[in]      POWER  ��������ֵ
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
	 //�������
	 scale = POWER_LIMIT_BEHAVE(	POWER_MID , Power_Buffer , Capacity_Voltage);
	 //��������С�ٶ�
	 *SPEED_X 		= scale*(*SPEED_X);
	 *SPEED_Y 		= scale*(*SPEED_Y);
	 *SPEED_W_Z  = scale*(*SPEED_W_Z);
	 //��Ŀ��ֵ����ͨ�˲�
	 *SPEED_X 	 = 	Filter_one(*SPEED_X,Filter_value);
	 *SPEED_Y 	 = 	Filter_one(*SPEED_Y,Filter_value+1);
	 *SPEED_W_Z = 	Filter_one(*SPEED_W_Z,Filter_value+2);
	 
	 //usart1_report_imu(*SPEED_X,*SPEED_Y,*SPEED_W_Z,10,100);


}

/**
  * @brief          �������ƾ������
  * @param[in]      POWER  								��������ֵ
  * @param[in]      Power_Buffer  				��������ֵ
  * @param[in]      capacity_Voltage  		��������ʣ�����ֵ
  * @author     		Chen
  */
float POWER_LIMIT_BEHAVE(int POWER , float Power_Buffer , float Capacity_Voltage)
{
		float  Scale=1;
		
		return Scale;
}


/**
  * @brief          һ�׵�ͨ�˲�
  * @param[in]      Value  							����ֵ����
  * @param[in]      Current_outvalue  	�����˲����ֵ
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
  * @brief          ���̸�����̨
  * @author     		Chen
 */
int Chassis_Follow()
{
		int value;
		value = -Chassis_Follow_PID( Gimbal_Init_Encoder_Value , YAW_ANGEL , CHASSIS_PID_FOLLOW , 100) ;
	  return value;
}


/**
  * @brief          �����жϼ�������
  * @author     		Chen
 */
void Chassis_task(void)
{																																																				
		RC_TO_CHAISS(rc_ctrl.rc.s[1])	;																																											  //�л����̿���ģʽ	������������ٶ�
		POWER_LIMIT(	POWER_Lmt_MODE	, &SPEED_X	,	&SPEED_Y	,	&SPEED_W_Z	, POWER_INITIAL );																//��������
		ChassisMove(SPEED_X,SPEED_Y,SPEED_W_Z);																																								//���̵���ٶ�ֵ�����ֵ����
		CAN_cmd_chassis(Chassis_3508_Current[0], Chassis_3508_Current[1], Chassis_3508_Current[2], Chassis_3508_Current[3]); 	//�����ֵ			

}








