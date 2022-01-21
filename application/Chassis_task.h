#ifndef _CHASSIS_TASK_
#define _CHASSIS_TASK_

#include "main.h"
#include "pid.h"
#include "level_control.h"


#define CHASSIS_MODE  			1  //1Ϊȫ���� 0Ϊ�����ķ��
#define CHASSIS_PID_MODE    1  //����PID��ʼ����ʽ


#define MOTOR_DISTANCE_TO_CENTER_L  1.74f      //�������־����һ��
#define MOTOR_DISTANCE_TO_CENTER_l  2.25f			 //ǰ�����־����һ��


#define    POWER_Lmt_MODE          1        //��������ģʽ


extern PID   Chassis_3508[4]; //���̵��ֵ
extern float  WheelSpeed[4];           //����Ŀ���ٶ�
extern int    Chassis_3508_Current[4]; //���̸�ֵ����
extern int 	  SPEED_X,SPEED_Y,SPEED_W_Z;
extern int    POWER_INITIAL ;
extern int		POWER_POWER_FIRST[3] ;
extern int		POWER_HP_RFIRST[3] ;
extern int CHASSIS_SPEED_X_MAX;
extern int CHASSIS_SPEED_Y_MAX ;
extern int CHASSIS_SPEED_W_Z_MAX ;


/**
  * @brief          ȫ���ֵ��̽���
  * @param[in]      SPEED_X: X�ٶ�
  * @param[in]      SPEED_Y: Y�ٶ�
  * @param[in]      SPEED_W_Z: ��ת�ٶ�
	* @author    			Chen
  */
void ChassisMove(int SPEED_X , int SPEED_Y , int SPEED_W_Z);


/**
  * @brief          ȫ���ֵ��̸�Ч���ƶ���ʽ����
  * @param[in]      SPEED_X: X�ٶ�
  * @param[in]      SPEED_Y: Y�ٶ�
  * @param[in]      SPEED_W_Z: ��ת�ٶ�
	* @author    			Chen
  */
void ChassisMove_DEMO(int SPEED_X , int SPEED_Y , int SPEED_W_Z);


/**
  * @brief          ��ң����������Ŀ��ֵת��
  * @param[in]      RC_MODE   ����ң��������ֵ   2 Ϊң����ģʽ  3 Ϊ����ģʽ
  * @author     		Chen
  */
void RC_TO_CHAISS(int  RC_MODE);

/**
  * @brief          ���̵����ʼ��
  * @author     		Chen
  */
void Chaiss_PID_Init(void);

/**
  * @brief          ��������
  * @param[in]      POWER_LIMIT_MODE   ��������ģʽ   1 Ϊ�ֶ�����ģʽ  2 Ϊ����ϵͳ�Զ���ȡģʽ
  * @author     		Chen
  */
void POWER_LIMIT(int  POWER_LIMIT_MODE	,int *SPEED_X	,	int *SPEED_Y	,	int *SPEED_W_Z	,int POWER);
/**
  * @brief          �������ƾ������
  * @param[in]      POWER  								��������ֵ
  * @param[in]      Power_Buffer  				��������ֵ
  * @param[in]      capacity_Voltage  		��������ʣ�����ֵ
  * @author     		Chen
  */
float POWER_LIMIT_BEHAVE(int POWER , float Power_Buffer , float Capacity_Voltage);

/**
  * @brief          �����жϼ�������
  * @author     		Chen
  */
void Chassis_task(void);

/**
  * @brief          һ�׵�ͨ�˲�
  * @author     		Chen
  */
int Filter_one(int Value,int *Current_outvalue);

/**
  * @brief          ���̸�����̨
  * @author     		Chen
 */
int Chassis_Follow(void);

#endif
