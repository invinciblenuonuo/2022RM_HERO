#ifndef _SHOOT_TASK_
#define _SHOOT_TASK_


#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "level_control.h"
extern int SPEED_LEFT;
extern int SPEED_RIGHT;
extern int Trigger_Current;


/**
 * @brief  发射初始化程序
*/
void Shoot_Init(void);

/**
 * @brief  发射任务
*/
void Shoot_caculate(int speed_left,int speed_right);

/**
* @brief 发射任务
*/
void Shoot_Task(void);

#endif
