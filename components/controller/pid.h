/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "struct_typedef.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

} pid_type_def;

typedef struct pid  
{
	float P;
	float I;
	float D;

	float Set;      //�趨ֵ
	float Get;      //��ǰֵ
	
	
	float Err;     //��� Set - Get
  float Serr;    //�ۼ����
  float Derr;    //���΢��
  float Err_Last;

	int max_output;
	int integral_limit;
	
}PID;

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

/**
	* @brief      ���̵������PID����
	* @param  Set_speed   Ŀ���ٶ�
	* @param  act_speed    ʵ���ٶ�
	* @param  PID_TYPE    PID�ṹ��
	* @author    			Chen
*/

int PID_3508_Chassis(int Set_speed , int act_speed , PID  PID_TYPE) ;

/**
* @brief      ��̨����PID�ٶȻ�
* @param  		Set_speed   Ŀ���ٶ�
* @param  		act_speed   ʵ���ٶ�
* @param  		PID_TYPE    PID�ṹ��
*/
int PID_GIMBAL_SPEED(int Set_speed , int act_speed , PID  PID_TYPE)  ;
	
/**
* @brief      ��̨����PID�ǶȻ�
* @param  		Set_Angel   Ŀ��Ƕ�
* @param  		Act_Angel   ʵ�ʽǶ�
* @param  		PID_TYPE    PID�ṹ��
*/
int PID_GIMBAL_ANGEL(int Set_Angel , int Act_Angel , PID  PID_TYPE , int MAX_SPEED)  ;

/**
* @brief    ���̸���PID����
*/
int Chassis_Follow_PID(int Set_Angel , int act_Angel , PID  PID_TYPE , int max_speed)  ;

#endif
