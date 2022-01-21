/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
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
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "pid.h"
#include "main.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

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
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

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
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

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
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}


/**
* @brief    ���̵������PID����
* @param  	Set_speed    Ŀ���ٶ�
* @param  	act_speed    ʵ���ٶ�
* @param  	PID_TYPE     PID�ṹ��
*/
int PID_3508_Chassis(int Set_speed , int act_speed , PID  PID_TYPE)  
{
	  int  Set_3508;
	  PID_TYPE.Err = Set_speed - act_speed;
	  PID_TYPE.Serr += PID_TYPE.Err;
	  PID_TYPE.Derr = PID_TYPE.Err -  PID_TYPE.Err_Last;
	  Set_3508  =   PID_TYPE.P*PID_TYPE.Err +  PID_TYPE.D*PID_TYPE.Derr + PID_TYPE.I*PID_TYPE.Serr;
	  PID_TYPE.Err_Last = PID_TYPE.Err;
	  if(Set_3508 > 16384)
		{
			Set_3508 = 16384;
		}		
		if(Set_3508 < -16384)
		{
			Set_3508 = -16384;
		}
	  return    Set_3508;
}

/**
* @brief      ��̨����PID�ǶȻ�
* @param  		Set_Angel   Ŀ��Ƕ�
* @param  		Act_Angel   ʵ�ʽǶ�
* @param  		PID_TYPE    PID�ṹ��
*/
int PID_GIMBAL_ANGEL(int Set_Angel , int Act_Angel , PID  PID_TYPE , int MAX_SPEED)  
{
	int Set_Speed;
		PID_TYPE.Err = Set_Angel - Act_Angel;
	  PID_TYPE.Serr += PID_TYPE.Err;
	  PID_TYPE.Derr = PID_TYPE.Err -  PID_TYPE.Err_Last;
	  Set_Speed  =   PID_TYPE.P*PID_TYPE.Err +  PID_TYPE.D*PID_TYPE.Derr + PID_TYPE.I*PID_TYPE.Serr;
	  PID_TYPE.Err_Last = PID_TYPE.Err;

	  if(Set_Speed > MAX_SPEED)
		{
			Set_Speed = MAX_SPEED;
		}
		
		if(Set_Speed < -MAX_SPEED)
		{
			Set_Speed = -MAX_SPEED;
		}
	
	return  Set_Speed;
}


/**
* @brief      ��̨����PID�ٶȻ�
* @param  		Set_speed   Ŀ���ٶ�
* @param  		act_speed   ʵ���ٶ�
* @param  		PID_TYPE    PID�ṹ��
*/
int PID_GIMBAL_SPEED(int Set_speed , int act_speed , PID  PID_TYPE)  
{
	  int  Set_6020;
	  PID_TYPE.Err = Set_speed - act_speed;
	  PID_TYPE.Serr += PID_TYPE.Err;
	  PID_TYPE.Derr = PID_TYPE.Err -  PID_TYPE.Err_Last;
	  Set_6020  =   PID_TYPE.P*PID_TYPE.Err +  PID_TYPE.D*PID_TYPE.Derr + PID_TYPE.I*PID_TYPE.Serr;
	  PID_TYPE.Err_Last = PID_TYPE.Err;
	  if(Set_6020 > 30000)
		{
			Set_6020 = 30000;
		}
		
		if(Set_6020 < -30000)
		{
			Set_6020 = -30000;
		}

	  return    Set_6020;
}


/**
* @brief    ���̸���PID����
*/
int Chassis_Follow_PID(int Set_Angel , int act_Angel , PID  PID_TYPE , int max_speed)  
{
	  int  Set_chassis_wz;
	  PID_TYPE.Err = Set_Angel - act_Angel;
	  PID_TYPE.Serr += PID_TYPE.Err;
	  PID_TYPE.Derr = PID_TYPE.Err -  PID_TYPE.Err_Last;
	  Set_chassis_wz  =   PID_TYPE.P*PID_TYPE.Err +  PID_TYPE.D*PID_TYPE.Derr + PID_TYPE.I*PID_TYPE.Serr;
	  PID_TYPE.Err_Last = PID_TYPE.Err;
	  if(Set_chassis_wz > max_speed)
		{
			Set_chassis_wz = max_speed;
		}
		
		if(Set_chassis_wz < -max_speed)
		{
			Set_chassis_wz = -max_speed;
		}

	  return    Set_chassis_wz;
}













