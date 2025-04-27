#include "HardwareCodeInc.h"

//********
//�˲��ִ������������������������֣�����
//*����������
//*�͵���������
//*ʹ��TIM12 CHANNEL_1������Դ�������ĵײ����
//********


/***************************************���׶���*************************************************/
typedef enum
{
	F1=262,F2=294,F3=330,F4=349,F5=392,F6=440,F7=494,
	C1=524,C2=588,C3=660,C4=740,C5=820,C6=880,C7=988,
	S1=1048,S2=1176,S3=1320,S4=1480,S5=1640,S6=1760,S7=1976
	
	//Flat 1 - C1 - Shape7
	
}Tone;

typedef enum
{
	n1=2000,n2=1000,n4=500,n8=250,n16=125
	
	//����
	
}Note;



Buzzer_State BuzzerState=Buzzer_OFF;
//������
Tone Buzzer_StartUp_Tone[]={
	C6,S3,S2
};
Note Buzzer_StartUp_Note[]={
	n4,n8,n8
};
Tone Buzzer_InterruptionRocker_Tone[]={
	C1
};
Note Buzzer_InterruptionRocker_Note[]={
	n2
};

/***************************************���׶���********************************************************/

	
void Buzzer_StartUp(void)
{
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);//����PWM
	for(int i=0; i<sizeof (Buzzer_StartUp_Note); i++)
	{
		__HAL_TIM_SetAutoreload(&htim12,1000000/Buzzer_StartUp_Tone[i]);
		__HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_1,1000000/Buzzer_StartUp_Tone[i]/2);
		HAL_Delay(Buzzer_StartUp_Note[i]);
	}
	__HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_1,0);
}

/**
* @brief ���������ţ�FreeRTOS����
* @param argument: Not used
* @retval None
*/
void StartBuzzer(void *argument)
{
	for(;;)
	{
		switch(BuzzerState)
		{
			case 2:
				__HAL_TIM_SetAutoreload(&htim12,1000000/Buzzer_InterruptionRocker_Tone[0]);
				__HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_1,1000000/Buzzer_InterruptionRocker_Tone[0]/2);
				osDelay(250);
				__HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_1,0);
				osDelay(250);
				__HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_1,1000000/Buzzer_InterruptionRocker_Tone[0]/2);
				osDelay(250);
				__HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_1,0);
				osDelay(250);
			case 1:
				__HAL_TIM_SetAutoreload(&htim12,1000000/Buzzer_InterruptionRocker_Tone[0]);
				__HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_1,1000000/Buzzer_InterruptionRocker_Tone[0]/2);
				osDelay(500);
				__HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_1,0);
				osDelay(500);
				break;
			case 0:
				__HAL_TIM_SetCompare(&htim12,TIM_CHANNEL_1,0);
				osDelay(1000);
				break;
			default :
				osDelay(1000);
				break;
		}
	}

}

