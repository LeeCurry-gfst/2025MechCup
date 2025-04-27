//*************
//���н�������Ŀ��ƺ���λ�ڴ˻�freertos.c
//���������������ա����Դ���ͨ�š�OLED��ʾ����ʾ
//*************

#include "HardwareCodeInc.h"

struct BLEctrl BLEctrl_Command;
//==============���Դ��ڵ������===========================================

uint8_t CmpRxDa;//����һ�ֽڵ��Դ�������
uint8_t UI_CmpPrintfFlag=0;//���Դ��ڴ�ӡ���ݱ�־λ
//========================================================================


//==============ҡ��ң����������===========================================
uint8_t BLErockerRxDa; //����һ�ֽ�������������
uint8_t BLErockerStatus;//�������ڽ��ձ�־λ���͵�ƽ��ƣ�0����û����������

//========================================================================

//==============��е���Զ����������������==================================

uint8_t BLEarmRxDa; //����һ�ֽ�������������
uint8_t BLEarmStatus;//�������ڽ��ձ�־λ���͵�ƽ��ƣ�0����û����������
//========================================================================

uint16_t Test_PumpCmp=2500;//��������ռ�ձ�
uint16_t Test_ValveCmp=500;//������ͨ��ռ�ձ�
/**
  * @brief  ��ʼ��ң�ؽ������ݽṹ�壬����δ�յ�����ʱ�������ƴ���
  * @param  argument: Not used
  * @retval None
  */
void BLECtrl_RecDataInit(void)
{
	BLEctrl_Command.Chassis_S = 0;
	BLEctrl_Command.Chassis_X = 0;
	BLEctrl_Command.Chassis_R = 0;
	BLEctrl_Command.Arm1 = ARM1_Start+135.f-360.f;
	BLEctrl_Command.Arm2 = ARM2_Start+90.f;
	BLEctrl_Command.Arm3 = ARM3_Start+135.f-360.f;
	BLEctrl_Command.Arm4 = ARM4_Start+135.f-360.f;
	BLEctrl_Command.Arm5 = 0;
	BLEctrl_Command.Contianer_pourout = 0;
	BLEctrl_Command.Pump = 0;
	BLEctrl_Command.Contianer_updown = 0;
	BLEctrl_Command.Cleaner = 0;
}
/**
  * @brief  ���Դ���ͨ�ţ�����ָ������������
  * @param  argument: Not used
  * @retval None
  */
void UI_CmpPrintf(void)
{
	switch(UI_CmpPrintfFlag)
	{
		case 0:
//			Serial_Printf("Work!\r\n");
			break;
		case 1://BLE��������
			Serial_Printf("ChaS:%+05.2f\r\n",BLEctrl_Command.Chassis_S);
			Serial_Printf("ChaR:%+05.2f\r\n",BLEctrl_Command.Chassis_R);
			Serial_Printf("Arm1:%+05.2f\r\n",BLEctrl_Command.Arm1);
			Serial_Printf("Arm2:%+05.2f\r\n",BLEctrl_Command.Arm2);
			Serial_Printf("Arm3:%+05.2f\r\n",BLEctrl_Command.Arm3);
			Serial_Printf("Arm4:%+05.2f\r\n",BLEctrl_Command.Arm4);
			Serial_Printf("Arm5:%d\r\n",BLEctrl_Command.Arm5);
			Serial_Printf("Pour:%d\r\n",BLEctrl_Command.Contianer_pourout);
			Serial_Printf("Pump:%d\r\n",BLEctrl_Command.Pump);
			Serial_Printf("Updown:%d\r\n",BLEctrl_Command.Contianer_updown);
			Serial_Printf("Cleaner:%d\r\n",BLEctrl_Command.Cleaner);
			break;
		case 2://����״̬
			Serial_Printf("SM1:Act:%+04.1f% Tar:%+04.1f\r\n",wheel[1].Actual,wheel[1].Target);
//			Serial_Printf("SM2:A:%+04.1f% T:%+04.1f M:%+04.1f\r\n",wheel[2].Actual,wheel[2].Target,wheel[2].Torque );
//			Serial_Printf("SM3:A:%+04.1f% T:%+04.1f M:%+04.1f\r\n",wheel[3].Actual,wheel[3].Target,wheel[3].Torque );
//			Serial_Printf("SM4:A:%+04.1f% T:%+04.1f M:%+04.1f\r\n",wheel[4].Actual,wheel[4].Target,wheel[4].Torque );
			Serial_Printf("rpm:%+04.1f\r\n",C610[1].Speed_rpm );
			Serial_Printf("Out:%+04.1f\r\n",wheel[1].Out  ); 
			
			Serial_Printf("Kp:%04.2f %04d\r\n",wheel[1].Kp ,Kpid[0]);
			Serial_Printf("Ki:%04.2f %04d\r\n",wheel[1].Ki ,Kpid[1]);
			Serial_Printf("Kd:%04.2f %04d\r\n",wheel[1].Kd ,Kpid[2]); 
			break;
		case 3://��е��Ŀ��Ƕ�ң������
			Serial_Printf("Arm1:%04.2f\r\n",BLEctrl_Command.Arm1);
			Serial_Printf("Arm2:%04.2f\r\n",BLEctrl_Command.Arm2);
			Serial_Printf("Arm3:%04.2f\r\n",BLEctrl_Command.Arm3);
			Serial_Printf("Arm4:%04.2f\r\n",BLEctrl_Command.Arm4);
			Serial_Printf("Ki:%04.2f %04d\r\n",wheel[1].Ki ,Kpid[1]);
			Serial_Printf("Kd:%04.2f %04d\r\n",wheel[1].Kd ,Kpid[2]); 
			break;
		case 4://�����˶�ѧ����
			Serial_Printf("SM1:A:%+04.1f% T:%+04.1f M:%+04.1f\r\n",wheel[1].Actual,wheel[1].Target,wheel[1].Torque);
			Serial_Printf("SM2:A:%+04.1f% T:%+04.1f M:%+04.1f\r\n",wheel[2].Actual,wheel[2].Target,wheel[2].Torque );
			Serial_Printf("SM3:A:%+04.1f% T:%+04.1f M:%+04.1f\r\n",wheel[3].Actual,wheel[3].Target,wheel[3].Torque );
			Serial_Printf("SM4:A:%+04.1f% T:%+04.1f M:%+04.1f\r\n",wheel[4].Actual,wheel[4].Target,wheel[4].Torque );
			break;
		case 0xff://���Դ���ͨ��
			Serial_Printf ("HelldWorld\r\n");
			break;
		default :
			Serial_Printf ("WRONG%d\r\n",UI_CmpPrintfFlag);
	}
//	Serial_Printf("Act:%+04.1f\r\n",wheel[2].Actual  ); 
//	Serial_Printf("Tar:%+04.1f\r\n",wheel[2].Target  ); 
//	Serial_Printf("RPM:%+04.1f\r\n",C610[2].Speed_rpm  ); 
//	Serial_Printf("Out:%+04.1f\r\n",wheel[2].Out  ); 
//	
//	Serial_Printf("Kp:%04.2f %04d\r\n",wheel[2 ].Kp ,Kpid[0]);
//	Serial_Printf("Ki:%04.2f %04d\r\n",wheel[2 ].Ki ,Kpid[1]);
//	Serial_Printf("Kd:%04.2f %04d\r\n",wheel[2 ].Kd ,Kpid[2]); 
}

/**
  * @brief  OLED��ʾ����ʾ
  * @param  argument: Not used
  * @retval None
  */
void UI_OLEDPrintf(void)
{
//	/*ң��������ʾ*/
	OLED_Printf (0,0,OLED_6X8,"TX:%+5.2f",BLEctrl_Command.Chassis_X);
	OLED_Printf (0,8,OLED_6X8,"TY:%+5.2f",BLEctrl_Command.Chassis_S);
	OLED_Printf (0,16,OLED_6X8,"TR:%+5.2f",BLEctrl_Command.Chassis_R);
	
//	/*���ʵʱ״̬��ʾ*/
	OLED_Printf(0,24,OLED_6X8 ,"M1:Act%+08.1fTar%+04.0f",wheel[1].Actual,wheel[1].Target );
	OLED_Printf(0,32,OLED_6X8 ,"M2:Act%+08.1fTar%+04.0f",wheel[2].Actual,wheel[2].Target );
	OLED_Printf(0,40,OLED_6X8 ,"M3:Act%+08.1fTar%+04.0f",wheel[3].Actual,wheel[3].Target );
	OLED_Printf(0,48,OLED_6X8 ,"M4:Act%+08.1fTar%+04.0f",wheel[4].Actual,wheel[4].Target );	
//	
//	/*�ڻ�PID��ֵ��ʾ*/
	OLED_Printf(60,0,OLED_6X8 ,"Kp:%04.1f",wheel[1].Kp );
	OLED_Printf(60,8,OLED_6X8 ,"Ki:%04.1f",wheel[1].Ki);
	OLED_Printf(60,16,OLED_6X8,"Kd:%04.1f",wheel[1].Kd);

//	/*���ʵʱ״̬��ʾ*/
	OLED_Printf(60,24,OLED_6X8 ,"Arm1:%04.1f",BLEctrl_Command.Arm1 );
	OLED_Printf(60,32,OLED_6X8 ,"Arm2:%04.1f",BLEctrl_Command.Arm2 );
	OLED_Printf(60,40,OLED_6X8 ,"Arm3:%04.1f",BLEctrl_Command.Arm3 );
	OLED_Printf(60,48,OLED_6X8 ,"Arm4:%04.1f",BLEctrl_Command.Arm4 );	
//	/*��������̬����չʾ*/
//	OLED_Printf(0,24,OLED_6X8 ,"M1:Act%+08.1fTar%+04.0f",wheel[1].Actual,wheel[1].Target );
//	OLED_Printf(0,32,OLED_6X8 ,"M2:Act%+08.1fTar%+04.0f",wheel[2].Actual,wheel[2].Target );
//	OLED_Printf(0,40,OLED_6X8 ,"M3:Act%+08.1fTar%+04.0f",wheel[3].Actual,wheel[3].Target );
//	OLED_Printf(0,48,OLED_6X8 ,"M4:Act%+08.1fTar%+04.0f",wheel[4].Actual,wheel[4].Target );	
//	
//	/*��������̬����չʾ*/
//	OLED_Printf(0,24,OLED_6X8 ,"M1:Act%+08.1fTar%+04.0f",wheel[1].Actual,wheel[1].Target );
//	OLED_Printf(0,32,OLED_6X8 ,"M2:Act%+08.1fTar%+04.0f",wheel[2].Actual,wheel[2].Target );
//	OLED_Printf(0,40,OLED_6X8 ,"M3:Act%+08.1fTar%+04.0f",wheel[3].Actual,wheel[3].Target );
//	OLED_Printf(0,48,OLED_6X8 ,"M4:Act%+08.1fTar%+04.0f",wheel[4].Actual,wheel[4].Target );	
	
	//������ʾ����ʾ
	OLED_Update();
}

/**************************************FreeRTOS����*****************************************************/

/**
* @brief ʵ������ҡ��ң�����ݽ���
* @param argument: Not used
* @retval None
*/
void StartUI_BLErocker(void *argument)
{
  /* USER CODE BEGIN StartUI_TeleControl */
  /* Infinite loop */
	uint8_t Rx_Buffer[32];
	uint8_t index=0;
	memset(Rx_Buffer,'\0',32);
  for(;;)
  {
	  /* ��ȡ��Ϣ�����е����ݵ�Rx_Buffer��ֱ����ȡ��\r */		
		do
		{
			if(xQueueReceive(BLErockerRecHandle,&Rx_Buffer[index],( TickType_t ) 10) == pdTRUE)
			{
				index++;
			}				
			else
				break;
		}while(Rx_Buffer[index-1] != '\r');
		/* ������� ���� ����Rx_Buffer�У�������һ�������һ��*/
		if(index !=0)
		{
			//ע���Ѿ��޸�Ϊ��ң�����ź�
			
			if(Rx_Buffer[0]==0xAA && Rx_Buffer [BLErockerLen-1]==0xEE) //�������ݰ��ṹ
			{
				Monitor_BLE_rocker = 1;
				
				BLEctrl_Command.Chassis_X =-( Rx_Buffer [1]-0x80)/128.0f*Chassis_ForwardMax;//ת���ٶȣ���λ��/s-->x
				BLEctrl_Command.Chassis_S =( Rx_Buffer [2]-0x80)/128.0f*Chassis_ForwardMax;//תǰ���ٶȣ���λcm/s-->r
				BLEctrl_Command.Chassis_R =-( Rx_Buffer [7]-0x80)/128.0f*Chassis_rotateMax;//ת�����ٶȣ���λcm/s->y
				
				if(fabs(BLEctrl_Command.Chassis_X) > fabs(BLEctrl_Command.Chassis_S)) BLEctrl_Command.Chassis_S = 0;
				else BLEctrl_Command.Chassis_X = 0;
				
				BLEctrl_Command.Cleaner =Rx_Buffer[3];//���
				BLEctrl_Command.Pump =Rx_Buffer[4];//����
				BLEctrl_Command.Contianer_updown =Rx_Buffer[5];//̧��
				BLEctrl_Command.Contianer_pourout =Rx_Buffer[6];//����
				
				if(fabs(BLEctrl_Command.Chassis_S )<=0.2f*Chassis_ForwardMax) BLEctrl_Command.Chassis_S =0;
				if(fabs(BLEctrl_Command.Chassis_X )<=0.2f*Chassis_ForwardMax) BLEctrl_Command.Chassis_X =0;
				if(fabs(BLEctrl_Command.Chassis_R )<=0.2f*Chassis_rotateMax) BLEctrl_Command.Chassis_R =0;
				
				BLErockerStatus=1;//����ҡ��ң�����ɹ����ձ�־λ
//				uint8_t BLE_OK[]="BLE1OK";
//				HAL_UART_Transmit (&huart1,BLE_OK,6,1000);
				Chassis_TeleCommandExecute();//�˶�ѧ����
			}
			
			
			if(Rx_Buffer[0]==0xAA && Rx_Buffer [BLEarmLen-1]==0xEE) //�������ݰ��ṹ
			{
				BLEctrl_Command.Arm1 =(float)(( (Rx_Buffer [1])<<8 )+ Rx_Buffer[2])/4096.f*360.f;//ת360��
				BLEctrl_Command.Arm2 =(float)(( (Rx_Buffer [3])<<8 )+ Rx_Buffer[4])/4096.f*360.f;//ת360��
				
				BLEctrl_Command.Arm1 = 360.f-BLEctrl_Command.Arm1;
				BLEctrl_Command.Arm2 = 360.f-BLEctrl_Command.Arm2;
				
				BLEctrl_Command.Arm3 =(float)(( (Rx_Buffer [5])<<8 )+ Rx_Buffer[6])/4096.f*360.f;//ת360��
				BLEctrl_Command.Arm4 =(float)(( (Rx_Buffer [7])<<8 )+ Rx_Buffer[8])/4096.f*360.f;//ת360��
				BLEctrl_Command.Arm5 =Rx_Buffer[9];
				
				BLEarmStatus=1;//����ҡ��ң�����ɹ����ձ�־λ
//				uint8_t BLE_OK[]="BLEOK";
//				HAL_UART_Transmit (&huart2,BLE_OK,5,1000);
			}
			memset(Rx_Buffer,'\0',32);
			index=0;
		}
		osDelay(5);
  }
}

uint8_t BLE_armTEMP[16];

/**
* @brief ʵ�������Զ��������ң�����ݽ���
* @param argument: Not used
* @retval None
*/
void StartUI_BLEarm(void *argument)
{
  /* USER CODE BEGIN StartUI_TeleControl */
  /* Infinite loop */
	uint8_t Rx_Buffer[32];
	uint8_t index=0;
	memset(Rx_Buffer,'\0',32);
  for(;;)
  {
	  /* ��ȡ��Ϣ�����е����ݵ�Rx_Buffer��ֱ����ȡ��\r */		
		do
		{
			if(xQueueReceive(TmpArmRecHandle,&Rx_Buffer[index],portMAX_DELAY) == pdTRUE)
			{
				index++;
			}				
			else
				break;
		}while(Rx_Buffer[index-1] != '\r');
		
		/* ������� ���� ����Rx_Buffer�У�������һ�������һ��*/
		if(index !=0)
		{
			if(Rx_Buffer[0]==0xAA && Rx_Buffer [BLEarmLen-1]==0xEE) //�������ݰ��ṹ
			{
				BLEctrl_Command.Arm1 =(float)(( (Rx_Buffer [1])<<8 )+ Rx_Buffer[2])/4096.f*360.f;//ת360��
				BLEctrl_Command.Arm2 =(float)(( (Rx_Buffer [3])<<8 )+ Rx_Buffer[4])/4096.f*360.f;//ת360��
				
				BLEctrl_Command.Arm1 = 360.f-BLEctrl_Command.Arm1;
				BLEctrl_Command.Arm2 = 360.f-BLEctrl_Command.Arm2;
				
				BLEctrl_Command.Arm3 =(float)(( (Rx_Buffer [5])<<8 )+ Rx_Buffer[6])/4096.f*360.f;//ת360��
				BLEctrl_Command.Arm4 =(float)(( (Rx_Buffer [7])<<8 )+ Rx_Buffer[8])/4096.f*360.f;//ת360��
				BLEctrl_Command.Arm5 =Rx_Buffer[9];
				
				BLEarmStatus=1;//����ҡ��ң�����ɹ����ձ�־λ
//				uint8_t BLE_OK[]="BLEOK";
//				HAL_UART_Transmit (&huart2,BLE_OK,5,1000);
			}
			memset(Rx_Buffer,'\0',32);
			index=0;
		}
		
		
		osDelay(5);

//		BLEctrl_Command.Arm1 =(float)(( (BLE_armTEMP [1]-0x80)<<8 )+ BLE_armTEMP[2])/4096.f*360.f;//ת360��
//		BLEctrl_Command.Arm2 =(float)(( (BLE_armTEMP [3]-0x80)<<8 )+ BLE_armTEMP[4])/4096.f*360.f;//ת360��
//		
//		BLEctrl_Command.Arm1 = 360.f-BLEctrl_Command.Arm1;
//		BLEctrl_Command.Arm2 = 360.f-BLEctrl_Command.Arm2;
//		
//		BLEctrl_Command.Arm3 =(float)(( (BLE_armTEMP [5]-0x80)<<8 )+ BLE_armTEMP[6])/4096.f*360.f;//ת360��
//		BLEctrl_Command.Arm4 =(float)(( (BLE_armTEMP [7]-0x80)<<8 )+ BLE_armTEMP[8])/4096.f*360.f;//ת360��
//		BLEctrl_Command.Arm5 =BLE_armTEMP[9];
//		
//		BLEarmStatus=1;//����ҡ��ң�����ɹ����ձ�־λ
//		
//		osDelay(100);
  }
}
/**
* @brief ���������յ��Դ�������
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUI_CmpRec */
void StartUI_Cmp(void *argument)
{
	uint8_t Rx_Buffer[32];
	memset(Rx_Buffer,'\0',32);
	uint8_t index=0;//���ն��г���
  for(;;)
  {
//		uint8_t Cmp_Txcmp[]="cmp\r\n";
//		HAL_UART_Transmit(&huart1,Cmp_Txcmp,5,1000);
		/* ��ȡ��Ϣ�����е����ݵ�Rx_Buffer��ֱ����ȡ��\r */	
		do
		{
			if(xQueueReceive(TestRecHandle,&Rx_Buffer[index],portMAX_DELAY) == pdTRUE)
			{ 
				index++;
//				Serial_Printf("cmpGet:%d %d\r\n",index,Rx_Buffer[index-1]);
			}
		}while(Rx_Buffer[index-1] != '\r');
		/* ������� ���� ����Rx_Buffer�У�������һ�������һ��*/
		if(index !=0)
		{
//			Serial_Printf("cmp:%d\r\n",index);
			if(Rx_Buffer[0]=='A'&&Rx_Buffer [1]=='T')
			{
				uint8_t Cmp_TxOK[]="OK\r\n";
				uint8_t Cmp_Txwrong[]="wrong\r\n";
				if(index-1 == 2)
					HAL_UART_Transmit(&huart1,Cmp_TxOK,4,1000);
				if(index-1==4 && Rx_Buffer [2]=='+')
				{	
					HAL_UART_Transmit(&huart1,Cmp_TxOK,4,1000);
					switch(Rx_Buffer [3])
					{
						case 'B'://������������
							UI_CmpPrintfFlag=1;
							break;
						case 'C'://�����˶�״̬
							UI_CmpPrintfFlag=2;
							break;
						case 'P'://ֹͣλ
							UI_CmpPrintfFlag=0;
							HAL_UART_Transmit(&huart1,Cmp_TxOK,4,1000);
							break;
						case 'A':
							UI_CmpPrintfFlag = 3;
						case 'S'://�����˶�ѧ����
							UI_CmpPrintfFlag=4;
							break ;
						case 'D'://debug����λ���������HelloWorld
							UI_CmpPrintfFlag = 0xff; 
							break;
						default :
							HAL_UART_Transmit(&huart1,Cmp_Txwrong,7,1000);
						break;
					}
				}
			}
			
			if(Rx_Buffer[0]=='T')
			{
				//����ģʽ��Ҫ�����ֵΪ��λ�з�����������TAMP=+1000
				if(Rx_Buffer[0]=='T'&&Rx_Buffer [1]=='A'&& Rx_Buffer[2]=='M'&& Rx_Buffer [3]=='P'&& Rx_Buffer [4]=='=')
				{
					C610[1].Target = (Rx_Buffer[6]-'0')*1000+(Rx_Buffer[7]-'0')*100+(Rx_Buffer[8]-'0')*10+(Rx_Buffer[9]-'0');
					if(Rx_Buffer[5] == '-') C610[1].Target *=-1;
				}
				//����ģʽ��Ҫ���ٶ�ֵΪ��λ�з�����������TSPE=+100
				if(Rx_Buffer[0]=='T'&&Rx_Buffer [1]=='S'&& Rx_Buffer[2]=='P'&& Rx_Buffer [3]=='E'&& Rx_Buffer [4]=='=')
				{
					wheel[1].Target = (Rx_Buffer[6]-'0')*100+(Rx_Buffer[7]-'0')*10+(Rx_Buffer[8]-'0');
					if(Rx_Buffer[5] == '-') wheel[1].Target *=-1;
				}
				//����ģʽ��Ҫ��Ƕ�ֵΪ��λ�޷�����������TARM=270
				if(Rx_Buffer[0]=='T'&&Rx_Buffer [1]=='A'&& Rx_Buffer[2]=='R'&& Rx_Buffer [3]=='M'&& Rx_Buffer [4]=='=')
				{
					BLEctrl_Command.Arm1 = (Rx_Buffer[5]-'0')*100+(Rx_Buffer[6]-'0')*10+(Rx_Buffer[7]-'0');
					
				}
				//����ģʽ��Ҫ�����ֵΪһλ�޷�����������TSTE=0
				if(Rx_Buffer[0]=='T'&&Rx_Buffer [1]=='S'&& Rx_Buffer[2]=='T'&& Rx_Buffer [3]=='E'&& Rx_Buffer [4]=='=')
				{
					BLEctrl_Command.Contianer_updown = (Rx_Buffer[5]-'0');
					
				}
				//����ģʽ��Ҫ���ٶ�ֵΪ��λ�з�����������TCHS=+100
				if(Rx_Buffer[0]=='T'&&Rx_Buffer [1]=='C'&& Rx_Buffer[2]=='H'&& Rx_Buffer [3]=='S'&& Rx_Buffer [4]=='=')
				{
					BLEctrl_Command.Chassis_S = (Rx_Buffer[6]-'0')*100+(Rx_Buffer[7]-'0')*10+(Rx_Buffer[8]-'0');
					if(Rx_Buffer[5] == '-') BLEctrl_Command.Chassis_S *=-1;
					
				}
				//����ģʽ��Ҫ����תֵΪ��λ�з�����������TCHR=+100
				if(Rx_Buffer[0]=='T'&&Rx_Buffer [1]=='C'&& Rx_Buffer[2]=='H'&& Rx_Buffer [3]=='R'&& Rx_Buffer [4]=='=')
				{
					BLEctrl_Command.Chassis_R = (Rx_Buffer[6]-'0')*100+(Rx_Buffer[7]-'0')*10+(Rx_Buffer[8]-'0');
					if(Rx_Buffer[5] == '-') BLEctrl_Command.Chassis_R *=-1;
					
				}
				//����ģʽ��Ҫ������ֵΪ��λ�޷�����������TPUM=500��ָ�������õ�PWM�Ƚ�ֵ
				if(Rx_Buffer[0]=='T'&&Rx_Buffer [1]=='P'&& Rx_Buffer[2]=='U'&& Rx_Buffer [3]=='M'&& Rx_Buffer [4]=='=')
				{
					Test_PumpCmp = (Rx_Buffer[5]-'0')*1000+(Rx_Buffer[6]-'0')*100+(Rx_Buffer[7]-'0')*10+(Rx_Buffer[8]-'0');
					Serial_Printf("OK");
				}
				//����ģʽ��Ҫ������ֵΪ��λ�޷�����������TVAL=500��ָ������ͨ����PWM�Ƚ�ֵ
				if(Rx_Buffer[0]=='T'&&Rx_Buffer [1]=='V'&& Rx_Buffer[2]=='A'&& Rx_Buffer [3]=='L'&& Rx_Buffer [4]=='=')
				{
					Test_ValveCmp = (Rx_Buffer[5]-'0')*1000+(Rx_Buffer[6]-'0')*100+(Rx_Buffer[7]-'0')*10+(Rx_Buffer[8]-'0');
				}
			}
			if(index == 6 &&Rx_Buffer [0]==0xaa&&Rx_Buffer [4]==0xee) //ģ������ң�������շ�
			{
				BLEctrl_Command.Chassis_S = (float)(Rx_Buffer [1]-128);
				BLEctrl_Command.Chassis_X = (float)(Rx_Buffer [2]-128);
				BLEctrl_Command.Chassis_R = (float)(Rx_Buffer [3]-128);
			}
			
			if(index == 23 && Rx_Buffer[21]==0x0a && Rx_Buffer[0]=='V' && Rx_Buffer[1]=='X' && Rx_Buffer[16]=='-')	//���Ӿ���������ͨ��
			{
				BLEctrl_Command.Chassis_S = (float)((Rx_Buffer [2]-'0')*100+(Rx_Buffer[4]-'0')*10+(Rx_Buffer[5]-'0'));
				BLEctrl_Command.Chassis_X = (float)((Rx_Buffer [9]-'0')*100+(Rx_Buffer[11]-'0')*10+(Rx_Buffer[12]-'0'));
				BLEctrl_Command.Chassis_R = (float)((Rx_Buffer [17]-'0')*100+(Rx_Buffer[19]-'0')*10+(Rx_Buffer[20]-'0'));
				BLEctrl_Command.Chassis_R *=-1;
			}
			
			if(index == 22 && Rx_Buffer[20]==0x0a && Rx_Buffer[0]=='V' && Rx_Buffer[1]=='X')
			{
				BLEctrl_Command.Chassis_S = (float)((Rx_Buffer [2]-'0')*100+(Rx_Buffer[4]-'0')*10+(Rx_Buffer[5]-'0'));
				BLEctrl_Command.Chassis_X = (float)((Rx_Buffer [9]-'0')*100+(Rx_Buffer[11]-'0')*10+(Rx_Buffer[12]-'0'));
				BLEctrl_Command.Chassis_R = (float)((Rx_Buffer [16]-'0')*100+(Rx_Buffer[18]-'0')*10+(Rx_Buffer[19]-'0'));
			}
			
			if(index == 12 && Rx_Buffer[0]==0xaa&&Rx_Buffer[10]==0xee) //ģ������ң���Զ�������������շ�
			{
				
				
				BLEctrl_Command.Arm1 =(float)(( (Rx_Buffer [1])<<8 )+ Rx_Buffer[2])/4096.f*360.f;//ת360��
				BLEctrl_Command.Arm2 =(float)(( (Rx_Buffer [3])<<8 )+ Rx_Buffer[4])/4096.f*360.f;//ת360��
				
				
				BLEctrl_Command.Arm3 =(float)(( (Rx_Buffer [5])<<8 )+ Rx_Buffer[6])/4096.f*360.f;//ת360��
				BLEctrl_Command.Arm4 =(float)(( (Rx_Buffer [7])<<8 )+ Rx_Buffer[8])/4096.f*360.f;//ת360��
				BLEctrl_Command.Arm5 =Rx_Buffer[9];
				
				BLEarmStatus=1;//����ҡ��ң�����ɹ����ձ�־λ
				
			}
			
			
			memset(Rx_Buffer,'\0',32);
			index = 0;	
		}
		osDelay(5);
  }
}
/**
* @brief ����Ҫ������Է��ʹ�������
* @param argument: Not used
* @retval None
*/
void StartUI_CmpTra(void *argument)
{
	TickType_t previousWakeTime=xTaskGetTickCount ();
	for(;;)
	{
		UI_CmpPrintf();
		vTaskDelayUntil (&previousWakeTime,pdMS_TO_TICKS (1000));
	}
}
