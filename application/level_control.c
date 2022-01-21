#include "level_control.h"


/**
  * @brief          等级控制 改变底盘目标速度最大值
  * @param[in]      LEVEL  				等级  0 - 3
  * @author     		Chen
  */
void LEVEL_CONTROL(int LEVEL)
{
		if(LEVEL == 0)
		{
				CHASSIS_SPEED_X_MAX 	= 500;
				
				CHASSIS_SPEED_Y_MAX 	= 500;
			
				CHASSIS_SPEED_W_Z_MAX = 500;			
			
				SPEED_LEFT  = 9000;
			  SPEED_RIGHT = 9000;
		}
		else if(LEVEL == 1)
		{
				CHASSIS_SPEED_X_MAX 	= 700;
			
				CHASSIS_SPEED_Y_MAX 	= 700;
			
				CHASSIS_SPEED_W_Z_MAX = 700;	
			
				SPEED_LEFT  = 9000;
			  SPEED_RIGHT = 9000;
			
		}
		else if(LEVEL == 2)
		{
				CHASSIS_SPEED_X_MAX 	= 900;
			
				CHASSIS_SPEED_Y_MAX 	= 900;
			
				CHASSIS_SPEED_W_Z_MAX = 900;	
			
				SPEED_LEFT  = 9000;
			  SPEED_RIGHT = 9000;
			
		}
		else if(LEVEL == 3)
		{
				CHASSIS_SPEED_X_MAX 	= 1000;
			
				CHASSIS_SPEED_Y_MAX 	= 1000;
			
				CHASSIS_SPEED_W_Z_MAX = 1000;
			
				SPEED_LEFT  = 9000;
			  SPEED_RIGHT = 9000;
			
		}
}
