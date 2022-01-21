/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
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
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
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
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
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
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
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
* @brief    底盘单个电机PID计算
* @param  	Set_speed    目标速度
* @param  	act_speed    实际速度
* @param  	PID_TYPE     PID结构体
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
* @brief      云台串级PID角度环
* @param  		Set_Angel   目标角度
* @param  		Act_Angel   实际角度
* @param  		PID_TYPE    PID结构体
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
* @brief      云台串级PID速度环
* @param  		Set_speed   目标速度
* @param  		act_speed   实际速度
* @param  		PID_TYPE    PID结构体
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
* @brief    底盘跟随PID计算
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













