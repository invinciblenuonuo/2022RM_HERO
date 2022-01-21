#ifndef _CHASSIS_TASK_
#define _CHASSIS_TASK_

#include "main.h"
#include "pid.h"
#include "level_control.h"


#define CHASSIS_MODE  			1  //1为全向轮 0为麦克纳姆轮
#define CHASSIS_PID_MODE    1  //底盘PID初始化方式


#define MOTOR_DISTANCE_TO_CENTER_L  1.74f      //左右麦轮距离的一半
#define MOTOR_DISTANCE_TO_CENTER_l  2.25f			 //前后麦轮距离的一半


#define    POWER_Lmt_MODE          1        //功率限制模式


extern PID   Chassis_3508[4]; //底盘电机值
extern float  WheelSpeed[4];           //底盘目标速度
extern int    Chassis_3508_Current[4]; //底盘赋值电流
extern int 	  SPEED_X,SPEED_Y,SPEED_W_Z;
extern int    POWER_INITIAL ;
extern int		POWER_POWER_FIRST[3] ;
extern int		POWER_HP_RFIRST[3] ;
extern int CHASSIS_SPEED_X_MAX;
extern int CHASSIS_SPEED_Y_MAX ;
extern int CHASSIS_SPEED_W_Z_MAX ;


/**
  * @brief          全向轮底盘解算
  * @param[in]      SPEED_X: X速度
  * @param[in]      SPEED_Y: Y速度
  * @param[in]      SPEED_W_Z: 旋转速度
	* @author    			Chen
  */
void ChassisMove(int SPEED_X , int SPEED_Y , int SPEED_W_Z);


/**
  * @brief          全向轮底盘高效率移动方式解算
  * @param[in]      SPEED_X: X速度
  * @param[in]      SPEED_Y: Y速度
  * @param[in]      SPEED_W_Z: 旋转速度
	* @author    			Chen
  */
void ChassisMove_DEMO(int SPEED_X , int SPEED_Y , int SPEED_W_Z);


/**
  * @brief          到遥控器到底盘目标值转化
  * @param[in]      RC_MODE   传入遥控器拨杆值   2 为遥控器模式  3 为键鼠模式
  * @author     		Chen
  */
void RC_TO_CHAISS(int  RC_MODE);

/**
  * @brief          底盘电机初始化
  * @author     		Chen
  */
void Chaiss_PID_Init(void);

/**
  * @brief          功率限制
  * @param[in]      POWER_LIMIT_MODE   功率限制模式   1 为手动控制模式  2 为裁判系统自动读取模式
  * @author     		Chen
  */
void POWER_LIMIT(int  POWER_LIMIT_MODE	,int *SPEED_X	,	int *SPEED_Y	,	int *SPEED_W_Z	,int POWER);
/**
  * @brief          功率限制具体计算
  * @param[in]      POWER  								功率限制值
  * @param[in]      Power_Buffer  				缓冲能量值
  * @param[in]      capacity_Voltage  		超级电容剩余电量值
  * @author     		Chen
  */
float POWER_LIMIT_BEHAVE(int POWER , float Power_Buffer , float Capacity_Voltage);

/**
  * @brief          底盘中断计算任务
  * @author     		Chen
  */
void Chassis_task(void);

/**
  * @brief          一阶低通滤波
  * @author     		Chen
  */
int Filter_one(int Value,int *Current_outvalue);

/**
  * @brief          底盘跟随云台
  * @author     		Chen
 */
int Chassis_Follow(void);

#endif
