//********
//�˲��ִ�������ʵʱ2���ADC�밴���źţ�����
//*��λ���뿪��
//*���ڵ�λ����PID��ADCֵ��ȡ
//*���ڼ���Դ������ѹ��ADCֵ��ȡ
//********
#include "HardwareCodeInc.h"
float Batttery_Q;//ʣ���������ѹֱ�Ӷ�Ӧ����׼ȷ��
float Batttery_U;//ʣ���ص�ѹ

uint8_t Key1_Value=0;
uint8_t Key2_Value=0;
uint8_t Key3_Value=0;
uint8_t Key4_Value=0;
uint8_t Key5_Value=0;
uint8_t Key6_Value=0;
uint8_t Key7_Value=0;
uint8_t Key8_Value=0;


uint8_t Monitor_BLE_Arm=0;
uint8_t Monitor_BLE_rocker=0;

/**
* @brief ���ӣ�FreeRTOS����
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStatus_Monitor */
void StartStatus_Monitor(void *argument)
{
	TickType_t previousWakeTime=xTaskGetTickCount ();
	for(;;)
	{
		HAL_ADC_Start_DMA (&hadc3 ,Kpid ,4);
		
		//��ص����жϴ���
//		Batttery_U=Kpid[3]/4096.f*3.3f*6.f;//ADC��׼��ѹ3.3V����ѹ6��
//		if(Batttery_U <=14.8f)
//		{
//			BuzzerState = Buzzer_LowBattery;
//		}
//		Batttery_Q=(16.8f-Batttery_U)/(16.8f-14.f);//�������ռ��
		
		//������������,100msû�н��յ���Ϣ�Զ�����
		if(Monitor_BLE_rocker == 0)
		{
			BLEctrl_Command.Chassis_S = 0;
			BLEctrl_Command.Chassis_R = 0;
			BLEctrl_Command.Chassis_X = 0;
			
			wheel[1].Target = 0;
			wheel[2].Target = 0;
			wheel[3].Target = 0;
			wheel[4].Target = 0;
//			
			BuzzerState = Buzzer_InterruptionRocker;
		}
		else BuzzerState = Buzzer_OFF;

		
		if(BLEarmStatus == 0)
		{
			BuzzerState = Buzzer_InterruptionArm;
		}
		else
			BuzzerState = Buzzer_OFF;
		Monitor_BLE_rocker = 0,BLEarmStatus=0;
		
		vTaskDelayUntil (&previousWakeTime,pdMS_TO_TICKS (400));
		
		Key1_Value=HAL_GPIO_ReadPin(GPIO_KEY1,GPIO_KEY1_PIN);
		Key2_Value=HAL_GPIO_ReadPin(GPIO_KEY2,GPIO_KEY2_PIN);
		Key3_Value=HAL_GPIO_ReadPin(GPIO_KEY3,GPIO_KEY3_PIN);
		Key4_Value=HAL_GPIO_ReadPin(GPIO_KEY4,GPIO_KEY4_PIN);
		Key5_Value=HAL_GPIO_ReadPin(GPIO_KEY5,GPIO_KEY5_PIN);
		Key6_Value=HAL_GPIO_ReadPin(GPIO_KEY6,GPIO_KEY6_PIN);
		Key7_Value=HAL_GPIO_ReadPin(GPIO_KEY7,GPIO_KEY7_PIN);
		Key8_Value=HAL_GPIO_ReadPin(GPIO_KEY8,GPIO_KEY8_PIN);
	}
}
