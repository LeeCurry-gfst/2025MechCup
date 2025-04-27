#include "HardwareCodeInc.h"

uint8_t step_state=2;
/**
* @brief 步进电机驱动控制
* @param argument: Not used
* @retval None
*/
void StartSteppingMotor(void *argument)
{
	for(;;)
	{
		if(BLEctrl_Command.Contianer_updown == 1)
		{
			if(step_state == 2)
				step_state =0;
			if(step_state==0)
			{
				step_state=1;//表示已经执行了上升操作
				HAL_GPIO_WritePin(GPIO_STEP,GPIO_PIN_DIR,(GPIO_PinState)1);
				__HAL_TIM_SetCompare(&htim_step,TIM_CHANNEL_STEP,22);
				osDelay(4040);
				__HAL_TIM_SetCompare(&htim_step,TIM_CHANNEL_STEP,0);
			}
		}
		if(BLEctrl_Command.Contianer_updown == 0)
		{
			if(step_state == 1)
				step_state =0;
			if(step_state==0)
			{
				step_state=2;//表示已经执行了下降操作
				HAL_GPIO_WritePin(GPIO_STEP,GPIO_PIN_DIR,(GPIO_PinState)0);
				__HAL_TIM_SetCompare(&htim_step,TIM_CHANNEL_STEP,22);
				osDelay(4040);
				__HAL_TIM_SetCompare(&htim_step,TIM_CHANNEL_STEP,0);
			}
		}
		osDelay(50);
	}
}

float ARM1_Start=72.68f-135.f+360.0f;
float ARM2_Start=21.0f-90.f+360.f;

float ARM3_Start=122.78f-135.f+360.0f;
float ARM4_Start=117.42f-135.f+360.f;

/* USER CODE BEGIN Header_StartMechArm */
/**
* @brief 机械臂驱动控制/三通阀启动/风机启动/气泵启动
* @param argument: Not used
* @retval None
*/
void StartMechArm(void *argument)
{
	for(;;)
	{
		//解算机械臂角度
		
		int16_t temp_cmp =500 ;
		uint8_t temp_arm = 0;
		
		temp_arm = (ARM1_Start <= BLEctrl_Command.Arm1)? 0:1;
		temp_cmp= (int)(2000.f*(BLEctrl_Command.Arm1+temp_arm*360.f-ARM1_Start)/270.f);
		if(temp_cmp <500) temp_cmp =500;
		if(temp_cmp >2500) temp_cmp = 2500;//避免超调
		temp_cmp =3000-temp_cmp;
		__HAL_TIM_SetCompare(&htim_arm,TIM_CHANNEL_ARM1,temp_cmp);
		
		temp_arm = (ARM2_Start <= BLEctrl_Command.Arm2)? 0:1;
		temp_cmp= (int)(2000.f*(BLEctrl_Command.Arm2+temp_arm*360.f-ARM2_Start)/180.f);
		if(temp_cmp <500) temp_cmp =500;
		if(temp_cmp >2500) temp_cmp = 2500;
		temp_cmp =500+(2500-temp_cmp);
		__HAL_TIM_SetCompare(&htim_arm,TIM_CHANNEL_ARM2,temp_cmp);
		
		temp_arm = (ARM3_Start <= BLEctrl_Command.Arm3)? 0:1;
		temp_cmp= 500+(int)(2000.f*(BLEctrl_Command.Arm3+temp_arm*360.f-ARM3_Start)/270.f);
		if(temp_cmp <500) temp_cmp =500;
		if(temp_cmp >2500) temp_cmp = 2500;
		__HAL_TIM_SetCompare(&htim_arm,TIM_CHANNEL_ARM3,temp_cmp);
		
		temp_arm = (ARM4_Start <= BLEctrl_Command.Arm4)? 0:1;
		temp_cmp= 500+(int)(2000.f*(BLEctrl_Command.Arm4+temp_arm*360.f-ARM4_Start)/270.f);
		if(temp_cmp <500) temp_cmp =500;
		if(temp_cmp >2500) temp_cmp = 2500;
		__HAL_TIM_SetCompare(&htim_arm,TIM_CHANNEL_ARM4,temp_cmp);
		
		if(BLEctrl_Command.Arm5 == 1)//开启三通阀
		{
			__HAL_TIM_SetCompare(&htim_sucker,TIM_CHANNEL_VALVE,20001);//原本为舵机线控制，临时修改为继电器控制
		}
		else
		{
			__HAL_TIM_SetCompare(&htim_sucker,TIM_CHANNEL_VALVE,0);
		}
		
		if(BLEctrl_Command.Pump == 1)//开启气泵
		{
			__HAL_TIM_SetCompare(&htim_sucker,TIM_CHANNEL_PUMP,20001);//原本为舵机线控制，临时修改为继电器控制
		}
		else
		{
			__HAL_TIM_SetCompare(&htim_sucker,TIM_CHANNEL_PUMP,0);
		}
		if(BLEctrl_Command.Cleaner == 1)//开启吸尘器风机
		{
			HAL_GPIO_WritePin(GPIO_FAN,GPIO_PIN_FAN,(GPIO_PinState)1);
		}
		else
		{
			HAL_GPIO_WritePin(GPIO_FAN,GPIO_PIN_FAN,(GPIO_PinState)0);
		}
		osDelay(100);
	}
}
uint8_t Launchstate=0;
/**
* @brief 倒出结构控制,倾斜后一秒开闸倒出
* @param argument: Not used
* @retval None
*/
void StartLaunch(void *argument)
{
	for(;;)
	{
		if(BLEctrl_Command.Contianer_pourout == 1)
		{
			if(Launchstate == 2)
				Launchstate =0;
			if(Launchstate==0)
			{
				Launchstate=1;
				__HAL_TIM_SetCompare(&htim_launch,TIM_CHANNEL_POUROUT,1166);
				osDelay(1000);//延时一秒后开闸
				__HAL_TIM_SetCompare(&htim_launch,TIM_CHANNEL_DROP,500);
			}
		}
		if(BLEctrl_Command.Contianer_pourout == 0)
		{
			if(Launchstate == 1)
				Launchstate =0;
			if(Launchstate==0)
			{
				Launchstate=2;
				__HAL_TIM_SetCompare(&htim_launch,TIM_CHANNEL_DROP,1500);
				osDelay(500);//延时0.5s回收
				__HAL_TIM_SetCompare(&htim_launch,TIM_CHANNEL_POUROUT,500);
			}
		}
		
		osDelay(50);
	}
}

float ARM1_InitAngle=135.f;
float ARM2_InitAngle=90.f;
float ARM3_InitAngle=135.f;
float ARM4_InitAngle=135.f;

void Others_Init(void)
{
	HAL_TIM_PWM_Start(&htim_arm,TIM_CHANNEL_ARM1);
	HAL_TIM_PWM_Start(&htim_arm,TIM_CHANNEL_ARM2);
	HAL_TIM_PWM_Start(&htim_arm,TIM_CHANNEL_ARM3);
	HAL_TIM_PWM_Start(&htim_arm,TIM_CHANNEL_ARM4);
	HAL_TIM_PWM_Start(&htim_launch,TIM_CHANNEL_DROP);
	HAL_TIM_PWM_Start(&htim_launch,TIM_CHANNEL_POUROUT);
	HAL_TIM_PWM_Start(&htim_sucker,TIM_CHANNEL_VALVE);
	HAL_TIM_PWM_Start(&htim_sucker,TIM_CHANNEL_PUMP);
	HAL_TIM_PWM_Start(&htim_step,TIM_CHANNEL_STEP);
	
	//机械臂保持初始角度
	
	int16_t temp_cmp =500 ;
	temp_cmp= 500+(int)(2000.f*ARM1_InitAngle/270.f);
	__HAL_TIM_SetCompare(&htim_arm,TIM_CHANNEL_ARM1,temp_cmp);
	temp_cmp= 500+(int)(2000.f*ARM2_InitAngle/180.f);
	__HAL_TIM_SetCompare(&htim_arm,TIM_CHANNEL_ARM2,temp_cmp);
	temp_cmp= 500+(int)(2000.f*ARM3_InitAngle/270.f);
	__HAL_TIM_SetCompare(&htim_arm,TIM_CHANNEL_ARM3,temp_cmp);
	temp_cmp= 500+(int)(2000.f*ARM4_InitAngle/270.f);
	__HAL_TIM_SetCompare(&htim_arm,TIM_CHANNEL_ARM4,temp_cmp);
	
	
	//投放结构保持初态0度
	__HAL_TIM_SetCompare(&htim_launch,TIM_CHANNEL_DROP,1166);//90度->0度
	__HAL_TIM_SetCompare(&htim_launch,TIM_CHANNEL_POUROUT,500);//0度->50度
	
	//关闭风机、三通阀、气泵
	HAL_GPIO_WritePin(GPIO_FAN,GPIO_PIN_FAN,(GPIO_PinState)0);
	__HAL_TIM_SetCompare(&htim_sucker,TIM_CHANNEL_VALVE,0);
	__HAL_TIM_SetCompare(&htim_sucker,TIM_CHANNEL_PUMP,0);
	
	//步进电机启动，配置16细分，不输出脉冲
	HAL_GPIO_WritePin(GPIO_STEP,GPIO_PIN_EN,(GPIO_PinState)0);
	HAL_GPIO_WritePin(GPIO_STEP,GPIO_PIN_MS1,(GPIO_PinState)1);
	HAL_GPIO_WritePin(GPIO_STEP,GPIO_PIN_MS2,(GPIO_PinState)1);
	HAL_GPIO_WritePin(GPIO_STEP,GPIO_PIN_DIR,(GPIO_PinState)0);
	__HAL_TIM_SetCompare(&htim_step,TIM_CHANNEL_STEP,0);
}
