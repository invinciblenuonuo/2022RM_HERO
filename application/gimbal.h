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
 * @brief  云台PID参数初始化
 * 
*/
int gimbal_pid_init(void);

/**
 * @brief  云台校准
 * @exp    云台上电后会进入无力状态，   当云台转动至霍尔传感器时，云台保存此时的云台编码器值
 * @param  Gimbal_Init_Encoder_Value    云台初始值
 * @param  Hoare_Switch    							霍尔开关值										 低触发
 * @param  Manual_Switch    						手动校准开关值  *防止霍尔失效* 低触发
*/
int gimbal_cali(void);


/**
 * @brief  云台初始化
*/
int gimbal_init(void);

/**
* @brief      		云台串级PID实现 *最简单控制方式*
* @param[in]  		Set_Angel   							目标角度
* @param[in]  		Gimbal_Control_Mode   		云台控制模式  1 为6020目标控制  2  为陀螺仪目标控制
* @param[out]  		Set   										6020电压赋值
*/
int PID_YAW_GIMBAL_DUO( int Set_Angel  , int Gimbal_Control_Mode );

/**
 * @brief  云台任务
*/
int GIMBAL_TASK(void);


/**
 * @brief      云台角度值连续计算
*/
int Gimbal_Angel_handle(void);



