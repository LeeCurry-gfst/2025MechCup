//********
//�˲��ִ�������ִ�е�����زٿأ�����
//*����ң�����ݵĽ�һ�������˶�ѧ���㺯����װ��
//*UI�͵��̲��ֵĳ�ʼ��
//*����MPU6050�����������ǵĽǶȻ�
//*��������ţ�1 3
//               2 4
//********

#include "HardwareCodeInc.h"

struct Chassis_PID wheel[5];
struct Chassis_PID angle;/*CAN���߿���*/
uint32_t Kpid[4];//����ADC��ȡ��Kpidֵ

/**
  * @brief  ����MPU6050��pidλ�ñջ���ÿ100ms����һ������
  * @param  argument: Not used
  * @retval None
  */
void Chassis_anglePID_Calculation(void)
{
	/*��ȡ��ǰĿ��ֵ��ƫ�����ۼӣ�*/
	if(BLEctrl_Command.Chassis_S >= 0) angle .Target += BLEctrl_Command.Chassis_R * angle.dt;
	else angle .Target -= BLEctrl_Command.Chassis_R * angle.dt;//���� ��ת��
	
	/*��ȡ��ǰʵ��ֵ*/
	float yaw
//		,pitch,roll
	;//��λ���Ƕ�
//	MPU_DMP_GetData(&pitch,&roll,&yaw );
	angle .Actual = yaw;
	
	/*��ȡ��ǰ�����PIDֵ����Ϊƽ·���µ�������*/
	float kp,ki,kd;
	if(angle.KpidFlag == Normal ) kp=angle.Kp,ki=angle.Ki ,kd=angle.Kd;
	
	/*����״̬�£�ͨ����ת��λ��ʵʱ����PID��������ȡĿǰADC��õ���PID��ֵ 4095->20ӳ��*/
	if(angle.KpidFlag == Debug  )
	{
		kp = Kpid[0]/4095.0*2;
		ki = Kpid[1]/4095.0*2;
		kd = Kpid[2]/4095.0*2;
		angle.Kp = Kpid[0]/4095.0*2;
		angle.Ki = Kpid[1]/4095.0*2;
		angle.Kd = Kpid[2]/4095.0*2;
	}
	
	/*��ȡ�������ϴ����*/
	angle.ErrorLast =angle.ErrorNow ;
	angle.ErrorNow = angle.Target - (angle.Actual) ;
	
	/*����ۼ�*/
	if(fabs(angle.ErrorNow) <= angle.ErrorLim )//���ַ���
	{
		angle.ErrorSum +=angle.ErrorNow ;
	}
	else
	{
		angle.ErrorSum +=angle.ErrorLim ;
	}
	if(fabs(angle.Ki )<=EPSILON ) angle.ErrorSum =0;
	if(fabs (angle.ErrorSum ) - angle.ErrorSumLim>0 ) 
		angle.ErrorSum =angle.ErrorSumLim ;
	
	/*PID����**  Kp*EN+Ki*ES+Kd*(EN-EL)  ���õ�Ŀ����ٶ�*/
	angle.Out = kp * angle.ErrorNow + ki * angle.ErrorSum + kd * (angle.ErrorNow - angle.ErrorLast); 
	
	/*��ֹ����*/
	if(angle.Out >150 ) angle.Out=150;
	if(angle.Out <-150 ) angle.Out=-150;
}

/**
  * @brief  ң������ִ�����˶�ѧ����
  * @param  ��
  * @retval ��
  */
void Chassis_TeleCommandExecute(void)
{
	float R_rad;
	if(angle.KpidFlag != OFF) R_rad = angle .Out /360.0f*2.0f*PI;//�⻷PID���ƾ�׼ת��
	else R_rad = BLEctrl_Command.Chassis_R /360.0f*2.0f*PI;//ֱ��ʹ��ң�����ݿ���ת����ת�����ƽ��ٶ�
	
	if(BLEctrl_Command.Chassis_S <0) R_rad*=-1;//���� ��ת��
//	
//	/*����ת���˶�ѧ����*/
//	wheel [1].Target = BLEctrl_Command.Chassis_S + R_rad * Chassis_Width / 2;
//	wheel [2].Target = BLEctrl_Command.Chassis_S + R_rad * Chassis_Width / 2;
//	wheel [3].Target = BLEctrl_Command.Chassis_S - R_rad * Chassis_Width / 2;
//	wheel [4].Target = BLEctrl_Command.Chassis_S - R_rad * Chassis_Width / 2;
//	wheel [3].Target*=-1;
//	wheel [4].Target*=-1;
	/*�����˶�ѧ����*/
	
	wheel [1].Target = BLEctrl_Command.Chassis_S - BLEctrl_Command.Chassis_X +R_rad*(Chassis_Width+Chassis_Length)/2.0f;
	wheel [2].Target = BLEctrl_Command.Chassis_S + BLEctrl_Command.Chassis_X +R_rad*(Chassis_Width+Chassis_Length)/2.0f;
	wheel [3].Target = BLEctrl_Command.Chassis_S + BLEctrl_Command.Chassis_X -R_rad*(Chassis_Width+Chassis_Length)/2.0f;
	wheel [4].Target = BLEctrl_Command.Chassis_S - BLEctrl_Command.Chassis_X -R_rad*(Chassis_Width+Chassis_Length)/2.0f;
	wheel [3].Target*=-1;
	wheel [4].Target*=-1;
	
}
/**
  * @brief PID������ʼ��
  * @param  ��ʼ���ĵ��PID������ַ
  * @retval ��
  */
void Chassis_PID_Init(void)
{
	(wheel+1)->Actual =0;
	(wheel+1)->Out =0;
	(wheel+1)->Target =0;
	(wheel+1)->ErrorSum =0;
	(wheel+1)->ErrorLast =0;
	(wheel+1)->ErrorNow =0;
	(wheel+1)->Kp =2390.0;
	(wheel+1)->Ki =60.0;
	(wheel+1)->Kd =50.0;
	(wheel+1)->Kpus =0;
	(wheel+1)->Kius =0;
	(wheel+1)->Kdus =0;
	(wheel+1)->Kpuh =0;
	(wheel+1)->Kiuh =0;
	(wheel+1)->Kduh =0;
	(wheel+1)->ErrorLim =100;
	(wheel+1)->ErrorSumLim =100;
	(wheel+1)->dt = 0.05f;
	(wheel+1)->KpidFlag = Normal;
	
	(wheel+2)->Actual =0;
	(wheel+2)->Out =0;
	(wheel+2)->Target =0;
	(wheel+2)->ErrorSum =0;
	(wheel+2)->ErrorLast =0;
	(wheel+2)->ErrorNow =0;
	(wheel+2)->Kp =2390.0;
	(wheel+2)->Ki =60.0;
	(wheel+2)->Kd =50.0;
	(wheel+2)->ErrorLim =100;
	(wheel+2)->ErrorSumLim =100;
	(wheel+2)->dt = 0.05f;
	(wheel+2)->KpidFlag = Normal;
	
	(wheel+3)->Actual =0;
	(wheel+3)->Out =0;
	(wheel+3)->Target =0;
	(wheel+3)->ErrorSum =0;
	(wheel+3)->ErrorLast =0;
	(wheel+3)->ErrorNow =0;
	(wheel+3)->Kp =2390.0;
	(wheel+3)->Ki =60.0;
	(wheel+3)->Kd =50.0;
	(wheel+3)->ErrorLim =100;
	(wheel+3)->ErrorSumLim =100;
	(wheel+3)->dt = 0.05f;
	(wheel+3)->KpidFlag = Normal;
	
	(wheel+4)->Actual =0;
	(wheel+4)->Out =0;
	(wheel+4)->Target =0;
	(wheel+4)->ErrorSum =0;
	(wheel+4)->ErrorLast =0;
	(wheel+4)->ErrorNow =0;
	(wheel+4)->Kp =2390.0;
	(wheel+4)->Ki =60.0;
	(wheel+4)->Kd =50.0;
	(wheel+4)->ErrorLim =100;
	(wheel+4)->ErrorSumLim =100;
	(wheel+4)->dt = 0.05f;
	(wheel+4)->KpidFlag = Normal;
	
	(&angle)->Actual =0;
	(&angle)->Out =0;
	(&angle)->Target =0;
	(&angle)->ErrorSum =0;
	(&angle)->ErrorLast =0;
	(&angle)->ErrorNow =0;
	(&angle)->Kp =0;
	(&angle)->Ki =0;
	(&angle)->Kd =0;
	(&angle)->ErrorLim =100;
	(&angle)->ErrorSumLim =100;
	(&angle)->dt = 0.1f;
	(&angle)->KpidFlag = OFF;
}
/******************FreeRTOS����*********************/
/**
  * @brief  ����Ӳ��������ĳ�ʼ��
  * @param  argument: Not used
  * @retval None
  */
void StartMy_Init(void *argument)
{
	//�ϵ���ʱ�ȴ��ⲿ�����ϵ����
	HAL_Delay(1000);
	
	
	//��������
	HAL_ADC_Start_DMA (&hadc3 ,Kpid ,4);
	HAL_ADC_Start(&hadc3 );
	
	__HAL_UART_ENABLE_IT(&huartCmp,UART_IT_IDLE);//���������ж�
	__HAL_UART_ENABLE_IT(&huartBLErocker,UART_IT_IDLE);//���������ж�
	__HAL_UART_ENABLE_IT(&huartBLEarm,UART_IT_IDLE);//���������ж�
	USART6->CR1|=USART_CR1_IDLEIE;
	
	HAL_UART_Receive_IT(&huartCmp,&CmpRxDa,1);
	HAL_UART_Receive_IT(&huartBLEarm, &BLEarmRxDa , 1);	
	HAL_UART_Receive_IT(&huartBLErocker, &BLErockerRxDa , 1);	
	
	//��ʼ������ģ��
	BLECtrl_RecDataInit();
	Chassis_PID_Init();
	Others_Init();//����������ģ�飬������е�ۡ���������á���ͨ��
	
	//��ʼ������ģ��
	OLED_Init ();
	OLED_Update();
	C610_Init();
	Serial_Printf ("Start!\r\n");
//	MPU_DMP_Init ();//��ʼ��MPU6050
	
	
	HAL_Delay(1000);
	//�ϵ���ɲ�����Ч
	Buzzer_StartUp();
	
	vTaskDelete(NULL);
}
/**
  * @brief  �ǶȻ��ջ�
  * @param  argument: Not used
  * @retval None
  */
void StartChassis_AngleLoop(void *argument)
{
	TickType_t previousWakeTime=xTaskGetTickCount ();
	for(;;)
	{	
//		Chassis_anglePID_Calculation();
		vTaskDelayUntil (&previousWakeTime,pdMS_TO_TICKS (100));
	}
}
