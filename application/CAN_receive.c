/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "main.h"
#include "cmsis_os.h"

#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
电机数据, 0:底盘电机1 3508电机,  1:底盘电机2 3508电机,2:底盘电机3 3508电机,3:底盘电机4 3508电机;
4:yaw云台电机 6020电机; 5:pitch云台电机 6020电机; 6:拨弹电机 2006电机*/
static motor_measure_t motor_chassis[7];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
		
motor_DATA_t    YAW_6020_DATA;
motor_DATA_t    PITCH_6020_DATA;
motor_DATA_t    SHOOT_MOTOR_LEFT;
motor_DATA_t    SHOOT_MOTOR_RIGHT;
motor_DATA_t    TRRIGLE_MOTOR_RIGHT;
motor_DATA_t    FRONT_MOTOR;

motor_DATA_t		M3508_Receive[4];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  * @author    			Chen
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
#if  TEST_MODE == 0

    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		if(hcan == &hcan2)
		{
				switch (rx_header.StdId)
				{
						//底盘
						case CAN_3508_M1_ID:	M3508_Receive[0].angel = rx_data[0] << 8 | rx_data[1] ;
																	M3508_Receive[0].speed = rx_data[2] << 8 | rx_data[3] ;
																	M3508_Receive[0].current = rx_data[4] << 8 | rx_data[5] ;	break;
						//底盘
						case CAN_3508_M2_ID:	M3508_Receive[1].angel = rx_data[0] << 8 | rx_data[1] ;
																	M3508_Receive[1].speed = rx_data[2] << 8 | rx_data[3] ;
																	M3508_Receive[1].current = rx_data[4] << 8 | rx_data[5] ;	break;
						//底盘
						case CAN_3508_M3_ID:  M3508_Receive[2].angel = rx_data[0] << 8 | rx_data[1] ;
																	M3508_Receive[2].speed = rx_data[2] << 8 | rx_data[3] ;
																	M3508_Receive[2].current = rx_data[4] << 8 | rx_data[5] ;	break;
			
						//底盘	
						case CAN_3508_M4_ID:	M3508_Receive[3].angel = rx_data[0] << 8 | rx_data[1] ;
																	M3508_Receive[3].speed = rx_data[2] << 8 | rx_data[3] ;
																	M3508_Receive[3].current = rx_data[4] << 8 | rx_data[5] ;	break;
						//YAW
						case CAN_YAW_MOTOR_ID: YAW_6020_DATA.angel = rx_data[0] << 8 | rx_data[1] ;
																	 YAW_6020_DATA.speed = rx_data[2] << 8 | rx_data[3] ;
																	 YAW_6020_DATA.current = rx_data[4] << 8 | rx_data[5] ;	break;
						//拨弹										 
						case CAN_TRIGGER_MOTOR_ID: 
																	 TRRIGLE_MOTOR_RIGHT.angel = rx_data[0] << 8 | rx_data[1] ;
																	 TRRIGLE_MOTOR_RIGHT.speed = rx_data[2] << 8 | rx_data[3] ;
																	 TRRIGLE_MOTOR_RIGHT.current = rx_data[4] << 8 | rx_data[5] ;	break;
																 
						case CAN_GIMBAL_ALL_ID:	
						{
								static uint8_t i = 0;
								//get motor id
								i = rx_header.StdId - CAN_3508_M1_ID;
								get_motor_measure(&motor_chassis[i], rx_data);
								detect_hook(CHASSIS_MOTOR1_TOE + i);
								break;
						}

						default:
						{
								break;
						}
				}
		 }
		else if(hcan == &hcan1)
		{
				switch (rx_header.StdId)
				{
						//PITCH
						case CAN_PIT_MOTOR_ID: PITCH_6020_DATA.angel = rx_data[0] << 8 | rx_data[1] ;
																	 PITCH_6020_DATA.speed = rx_data[2] << 8 | rx_data[3] ;
																	 PITCH_6020_DATA.current = rx_data[4] << 8 | rx_data[5] ;	break;
					
						//左摩擦轮										 
						case CAN_SHOOT_MOTOR_LEFT_ID: 
																	 SHOOT_MOTOR_LEFT.angel = rx_data[0] << 8 | rx_data[1] ;
																	 SHOOT_MOTOR_LEFT.speed = rx_data[2] << 8 | rx_data[3] ;
																	 SHOOT_MOTOR_LEFT.current = rx_data[4] << 8 | rx_data[5] ;	break;
						
						//右摩擦轮											 
						case CAN_SHOOT_MOTOR_RIGHT_ID: 
																	 SHOOT_MOTOR_RIGHT.angel = rx_data[0] << 8 | rx_data[1] ;
																	 SHOOT_MOTOR_RIGHT.speed = rx_data[2] << 8 | rx_data[3] ;
																	 SHOOT_MOTOR_RIGHT.current = rx_data[4] << 8 | rx_data[5] ;	break;
						
						//预置	
						case CAN_FRONT_MOTOR_ID:
																	 FRONT_MOTOR.angel = rx_data[0] << 8 | rx_data[1] ;
																	 FRONT_MOTOR.speed = rx_data[2] << 8 | rx_data[3] ;
																	 FRONT_MOTOR.current = rx_data[4] << 8 | rx_data[5] ;	break;					
				}

		}
		
#else
		
		CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		if(hcan == &hcan1)
		{
				switch (rx_header.StdId)
				{
						//PITCH
						case CAN_PIT_MOTOR_ID: PITCH_6020_DATA.angel = rx_data[0] << 8 | rx_data[1] ;
																	 PITCH_6020_DATA.speed = rx_data[2] << 8 | rx_data[3] ;
																	 PITCH_6020_DATA.current = rx_data[4] << 8 | rx_data[5] ;	break;
					
						//左摩擦轮										 
						case CAN_SHOOT_MOTOR_LEFT_ID: 
																	 SHOOT_MOTOR_LEFT.angel = rx_data[0] << 8 | rx_data[1] ;
																	 SHOOT_MOTOR_LEFT.speed = rx_data[2] << 8 | rx_data[3] ;
																	 SHOOT_MOTOR_LEFT.current = rx_data[4] << 8 | rx_data[5] ;	break;
						
						//右摩擦轮											 
						case CAN_SHOOT_MOTOR_RIGHT_ID: 
																	 SHOOT_MOTOR_RIGHT.angel = rx_data[0] << 8 | rx_data[1] ;
																	 SHOOT_MOTOR_RIGHT.speed = rx_data[2] << 8 | rx_data[3] ;
																	 SHOOT_MOTOR_RIGHT.current = rx_data[4] << 8 | rx_data[5] ;	break;
						
						//预置	
						case CAN_FRONT_MOTOR_ID:
																	 FRONT_MOTOR.angel = rx_data[0] << 8 | rx_data[1] ;
																	 FRONT_MOTOR.speed = rx_data[2] << 8 | rx_data[3] ;
																	 FRONT_MOTOR.current = rx_data[4] << 8 | rx_data[5] ;	break;
					
				}

		}
#endif
}



/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = 0;
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = 0;
    gimbal_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN2_cmd_shoot(int16_t shoot_l, int16_t shoot_r , int16_t front, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x200;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (shoot_l >> 8);
    gimbal_can_send_data[1] = shoot_l;
    gimbal_can_send_data[2] = (shoot_r >> 8);
    gimbal_can_send_data[3] = shoot_r;
    gimbal_can_send_data[4] = (front >> 8);
    gimbal_can_send_data[5] = front;
    gimbal_can_send_data[6] = 0;
    gimbal_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}


/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void CAN2_cmd_gimbal(int16_t gimbal_pitch)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x1ff;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = 0;
    gimbal_can_send_data[1] = 0;
    gimbal_can_send_data[2] = (gimbal_pitch >> 8);
    gimbal_can_send_data[3] = gimbal_pitch;
    gimbal_can_send_data[4] = 0;
    gimbal_can_send_data[5] = 0;
    gimbal_can_send_data[6] = 0;
    gimbal_can_send_data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

void can_filter_init(void)
{
	
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}


/**
  * @brief          任意电机测试函数
  * @param[in]      Value             : 电机直接赋值	
  * @param[in]      SEND_ID           : 电机发送ID		0x1ff / 0x200
  * @param[in]      MOTOR_ID          : 电机ID   			范围：1-8
  * @param[in]      CAN_ID          	: CAN编号   		1 / 2 
  * @author     		Chen
  */
void CAN_MOTOR_CHECK( int16_t Value , int16_t SEND_ID , int16_t MOTOR_ID ,int CAN_ID)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = SEND_ID;
    chassis_tx_message.IDE 	 = CAN_ID_STD;
    chassis_tx_message.RTR   = CAN_RTR_DATA;
    chassis_tx_message.DLC   = 0x08;
		if(MOTOR_ID == 1 || MOTOR_ID == 5)
		{
    chassis_can_send_data[0] = Value >> 8;
    chassis_can_send_data[1] = Value;
		}
		else if(MOTOR_ID == 2 || MOTOR_ID == 6)
		{
    chassis_can_send_data[2] = Value >> 8;
    chassis_can_send_data[3] = Value;
		}
		else if(MOTOR_ID == 3 || MOTOR_ID == 7)
		{
    chassis_can_send_data[4] = Value >> 8;
    chassis_can_send_data[5] = Value;
		}
		else if(MOTOR_ID == 4 || MOTOR_ID == 8)
		{
    chassis_can_send_data[6] = Value >> 8;
    chassis_can_send_data[7] = Value;
		}
		
		if(CAN_ID == 1)
		{
			HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);			
		}
		else if(CAN_ID == 2)
		{
			HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);				
		}
		else
		{
			HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);							
		}
}

