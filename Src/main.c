/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_delay.h"
#include "usart.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
unsigned char aRxBuffer1[1];		// 用来接收串口1发送的数据
unsigned char aRxBuffer2[1];		// 用来接收串口2发送的数据
int flag_cap = 0;               //电容串口初始化完成标志

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_I2C3_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
		printf("HAL READY\r\n");
    delay_init(); 
		


		//延时初始化
		HAL_TIM_Base_Start(&htim4);
    //start pwm channel
    //开启PWM通道
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

		GYRO_SIX_AXIS_Init();
		//陀螺仪初始化
		printf("gyro READY\r\n");
		
		Chaiss_PID_Init();
		printf("Chassis PID READY\r\n");
		//底盘PID初始化
		
		can_filter_init();
		printf("gimbal READY\r\n");
		//can_filter初始化

		printf("CAN READY\r\n");
		gimbal_init();
		//云台初始化
		Shoot_Init();
		//发射初始化
		printf("SHOOT READY\r\n");

		remote_control_init();
		printf("REMOTE READY\r\n");

		HAL_TIM_Base_Start_IT(&htim3);
		//陀螺仪处理中断开启
		
		HAL_TIM_Base_Start_IT(&htim2);
		
		//CAN发送中断开启
		printf("TIM READY\r\n");
		
		
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);  //receive interrupt
		//__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);  //idle interrupt
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);  //receive interrupt
		printf("UART READY\r\n");
		CAP_SEND( 0x00 , 0x06 );
		
		
		buzzer_start();

	/* 未用FreeRtos时以下两个函数需要被注释 
	MX_FREERTOS_Init();
	osKernelStart();
	*/
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  //MX_FREERTOS_Init();

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief          计算中断
  * @param[out]     角度值保存在 INS_angle[] 中
  * @retval         
  * @author    			Chen
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	  
	  static int htim2_count[4];
		
				if (htim == (&htim3)) 	//陀螺仪处理 1ms
				{
						GYRO_SIX_AXIS();
				}
				
				if (htim == (&htim2)) //can发送中断 10ms
				{
									htim2_count[0]++;
									htim2_count[1]++;
								
									/*    第二更新速度等级执行任务 频率 1/10    */
									
									if(htim2_count[0]>10)
									{
										htim2_count[0]=0;										
									}
									
									/*    第三更新速度等级执行任务 频率 1/20   */
									
									if(htim2_count[1]>20)	
									{
										 htim2_count[1]=0;	
										 HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_11);
									}
									/*第一更新速度等级执行任务*/
									
									LEVEL_CONTROL(1);	
									Shoot_Task();									
									GIMBAL_TASK();											
									Chassis_task();
				}	  
}



/**
 * @brief   串口1接收中断 接收电容数据
 * @author  Chen
*/
unsigned char CAP_DATA[20];
int count = 0;
float V;

//void USART1_IRQHandler(void)  
//{
//		flag_cap = 1;
//    if(huart1.Instance->SR & UART_FLAG_RXNE)
//    {
//        CAP_DATA[count++] = huart1.Instance->DR;
//		}
//		else if(huart1.Instance->SR & UART_FLAG_IDLE)
//    {

//         CAP_DATA[count++] = huart1.Instance->DR;
//				 count = 0;
//						if(CAP_DATA[0] == 0xff)
//						{
//									data_v.ch[0] = CAP_DATA[1];
//									data_v.ch[1] = CAP_DATA[2];
//									data_v.ch[2] = CAP_DATA[3];
//									data_v.ch[3] = CAP_DATA[4];
//														
//								
//									data_power.ch[0] = CAP_DATA[5];
//									data_power.ch[1] = CAP_DATA[6];
//									data_power.ch[2] = CAP_DATA[7];
//									data_power.ch[3] = CAP_DATA[8];
//								
//									data_v_out.ch[0] = CAP_DATA[9];
//									data_v_out.ch[1] = CAP_DATA[10];
//									data_v_out.ch[2] = CAP_DATA[11];
//									data_v_out.ch[3] = CAP_DATA[12];
//								  
//									BUCK_SWITCH = CAP_DATA[13];
//									POWER_SETED = CAP_DATA[14];
//								  //usart1_report_imu(data_v.f*1000,data_power.f*1000,data_v_out.f*1000,0,0);
//						}
//    }
//}
unsigned char GimbalRxBuffer[50];
unsigned int  GimbalRealBuf[7] = {0};
unsigned int  goal;
unsigned int  fire;
unsigned int  yaw_f;
int  yaw_angel;
unsigned int  pitch_f;
int  pitch_angel;



void USART1_IRQHandler(void)  
{
		if(huart1.Instance->SR & UART_FLAG_RXNE)
		{
				GimbalRxBuffer[count++] = huart1.Instance->DR;
				if (GimbalRxBuffer[count - 1] == 0x45)
				{
						if (GimbalRxBuffer[0] == 0x53)
						{
							goal = GimbalRxBuffer[1]; /* 是否有目标 */
							fire = GimbalRxBuffer[2]; /* 是否开火  */
							
							yaw_f = GimbalRxBuffer[3]; /* yaw正负 正:1 负:0  */
							GimbalRealBuf[3] = GimbalRxBuffer[5] << 8 | GimbalRxBuffer[4]; /* yaw偏差  */
							pitch_f = GimbalRxBuffer[6]; /* pitch正负 正:1 负:0  >*/
							GimbalRealBuf[5] = GimbalRxBuffer[8] << 8 | GimbalRxBuffer[7]; /*  pitch偏差  */
							
							if(yaw_f == 1)
							{
								yaw_angel = -GimbalRealBuf[3];
							}
							else
							{
								yaw_angel = GimbalRealBuf[3];							
							}
							
							if(pitch_f == 1)
							{
								pitch_angel = -GimbalRealBuf[5];
							}
							else
							{
								pitch_angel = GimbalRealBuf[5];								
							}
							
							
							GimbalRealBuf[6] = GimbalRxBuffer[10] << 8 | GimbalRxBuffer[9]; /*  距离  */
							count = 0;
							//memset(GimbalRxBuffer, 0x00, sizeof(GimbalRxBuffer));
						}
						else
						{
							count = 0;
						}
				}
		}
}

void USART6_IRQHandler(void)  
{
    volatile uint8_t receive;   
    if(huart6.Instance->SR & UART_FLAG_RXNE)
    {
        receive = huart6.Instance->DR;
    }
}




/**
 * @brief  电容控制
 * @param[in]  POWER_LEVEL    0X00 - 0X03  代表三个等级
 * @param[in]  BUCK_CONTROL   0x05 电容充电关  0x06电容充电开
*/
void CAP_SEND(unsigned char POWER_LEVEL,unsigned char BUCK_CONTROL)
{
	unsigned char ch[4];
	ch[0] = 0xff;
	ch[1] = POWER_LEVEL;
	ch[2] = BUCK_CONTROL;
	ch[3] = 0xaa;
	HAL_UART_Transmit(&huart1,ch,4,0xFFFF);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
