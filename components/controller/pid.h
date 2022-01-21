/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
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
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} pid_type_def;

typedef struct pid  
{
	float P;
	float I;
	float D;

	float Set;      //设定值
	float Get;      //当前值
	
	
	float Err;     //误差 Set - Get
  float Serr;    //累计误差
  float Derr;    //误差微分
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
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
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
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

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
extern void PID_clear(pid_type_def *pid);

/**
	* @brief      底盘单个电机PID计算
	* @param  Set_speed   目标速度
	* @param  act_speed    实际速度
	* @param  PID_TYPE    PID结构体
	* @author    			Chen
*/

int PID_3508_Chassis(int Set_speed , int act_speed , PID  PID_TYPE) ;

/**
* @brief      云台串级PID速度环
* @param  		Set_speed   目标速度
* @param  		act_speed   实际速度
* @param  		PID_TYPE    PID结构体
*/
int PID_GIMBAL_SPEED(int Set_speed , int act_speed , PID  PID_TYPE)  ;
	
/**
* @brief      云台串级PID角度环
* @param  		Set_Angel   目标角度
* @param  		Act_Angel   实际角度
* @param  		PID_TYPE    PID结构体
*/
int PID_GIMBAL_ANGEL(int Set_Angel , int Act_Angel , PID  PID_TYPE , int MAX_SPEED)  ;

/**
* @brief    底盘跟随PID计算
*/
int Chassis_Follow_PID(int Set_Angel , int act_Angel , PID  PID_TYPE , int max_speed)  ;

#endif
