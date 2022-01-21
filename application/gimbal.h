#ifndef _GIMBAL_H
#define _GIMBAL_H

#include "CAN_receive.h"
#include "pid.h"
#include "INS_task.h"
#include "remote_control.h"
#include "main.h"



#endif


extern long int YAW_remote,PITCH_remote;
extern long int YAW_ANGEL,PITCH_ANGEL;
extern int Gimbal_Init_Encoder_Value;

/**
 * @brief  ��̨PID������ʼ��
 * 
*/
int gimbal_pid_init(void);

/**
 * @brief  ��̨У׼
 * @exp    ��̨�ϵ����������״̬��   ����̨ת��������������ʱ����̨�����ʱ����̨������ֵ
 * @param  Gimbal_Init_Encoder_Value    ��̨��ʼֵ
 * @param  Hoare_Switch    							��������ֵ										 �ʹ���
 * @param  Manual_Switch    						�ֶ�У׼����ֵ  *��ֹ����ʧЧ* �ʹ���
*/
int gimbal_cali(void);


/**
 * @brief  ��̨��ʼ��
*/
int gimbal_init(void);

/**
* @brief      		��̨����PIDʵ�� *��򵥿��Ʒ�ʽ*
* @param[in]  		Set_Angel   							Ŀ��Ƕ�
* @param[in]  		Gimbal_Control_Mode   		��̨����ģʽ  1 Ϊ6020Ŀ�����  2  Ϊ������Ŀ�����
* @param[out]  		Set   										6020��ѹ��ֵ
*/
int PID_YAW_GIMBAL_DUO( int Set_Angel  , int Gimbal_Control_Mode );

/**
 * @brief  ��̨����
*/
int GIMBAL_TASK(void);


/**
 * @brief      ��̨�Ƕ�ֵ��������
*/
int Gimbal_Angel_handle(void);



