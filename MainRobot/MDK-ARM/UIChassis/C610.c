//********
//�˲��ִ�������ִ����ˢ�����زٿأ�����
//*CAN�����շ�������װ
//*PID�㷨����M2006���ת�٣����ٶȻ���
//*����C610�������������
//********

#include "HardwareCodeInc.h"

struct M2006_Para C610[5];
/**
* @brief PID�ջ����Ƶ��ת��
* @param argument: Not used
* @retval None
*/
void Chassis_SpeedLoopPID(struct Chassis_PID *Wheel_PID)
{
	
	if(Wheel_PID->KpidFlag == OFF )  {Wheel_PID->Out=0;return ;}
	/*��ȡ��ǰĿ��ֵ���˶�ѧ���㣬�ڽ���ң�����ݵ�ͬʱ����*/
	/*��ȡ��ǰʵ��ֵ��ͨ��CAN������ɻ�ȡ*/
	
	/*��ȡ��ǰ�����PIDֵ����Ϊ��������������*/
	float kp,ki,kd;
	if(Wheel_PID->KpidFlag == Normal ) kp=Wheel_PID->Kp,ki=Wheel_PID->Ki ,kd=Wheel_PID->Kd;
	if(Wheel_PID->KpidFlag == Uphill ) kp=Wheel_PID->Kpuh,ki=Wheel_PID->Kiuh ,kd=Wheel_PID->Kduh;
	if(Wheel_PID->KpidFlag == Upstair ) kp=Wheel_PID->Kpus,ki=Wheel_PID->Kius ,kd=Wheel_PID->Kdus;
	
	/*����״̬�£�ͨ����ת��λ��ʵʱ����PID��������ȡĿǰADC��õ���PID��ֵ 4095->20ӳ��*/
	if(Wheel_PID->KpidFlag == Debug  )
	{
		kp = Kpid[0]/4095.0*100;
		ki = Kpid[1]/4095.0*100;
		kd = Kpid[2]/4095.0*100;
		Wheel_PID->Kp = Kpid[0]/4095.0*3000;
		Wheel_PID->Ki = Kpid[1]/4095.0*500;
		Wheel_PID->Kd = Kpid[2]/4095.0*500;
		
	}
	
	/*��ȡ�������ϴ����*/
	Wheel_PID->ErrorLast =Wheel_PID->ErrorNow ;
	Wheel_PID->ErrorNow = Wheel_PID->Target - (Wheel_PID->Actual) ;
	
	/*����ۼ�*/
	if(fabs(Wheel_PID->ErrorNow) <= Wheel_PID->ErrorLim )//���ַ���
	{
		Wheel_PID->ErrorSum +=Wheel_PID->ErrorNow ;
	}
	else
	{
		Wheel_PID->ErrorSum +=Wheel_PID->ErrorLim ;
	}
	if(fabs(Wheel_PID->Ki )<=EPSILON ) Wheel_PID->ErrorSum =0;
	if(fabs (Wheel_PID->ErrorSum ) - Wheel_PID->ErrorSumLim>0 ) 
		Wheel_PID->ErrorSum =Wheel_PID->ErrorSumLim ;
	
	/*PID����**  Kp*EN+Ki*ES+Kd*(EN-EL)  ���õ�Ŀ����ٶ�*/
	Wheel_PID->Out = kp * Wheel_PID->ErrorNow + ki * Wheel_PID->ErrorSum + kd * (Wheel_PID->ErrorNow - Wheel_PID->ErrorLast); 
	
	/*��ֹ����*/
	if(Wheel_PID->KpidFlag == Upstair)
	{
		if(Wheel_PID->Out >5000 ) Wheel_PID->Out=5000;
		if(Wheel_PID->Out <-5000 ) Wheel_PID->Out=-5000;
	}
	else
	{
		if(Wheel_PID->Out >5000 ) Wheel_PID->Out=5000;
		if(Wheel_PID->Out <-5000 ) Wheel_PID->Out=-5000;
	}
	
}
/**
  * @brief  ����ٶȻ��ջ���CAN��������FreeRTOS����
  * @note   ��ȷ5ms���ڷ��ͣ����þ�����ʱ��֤ʱ��
  */
void StartChassis_SpeedLoop(void *argument)
{
	uint8_t tx_data[8] = {0,0,0,0,0,0,0,0};
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
		/*1.ִ���ٶȻ�PID����*/
		for(uint8_t i=0; i<4; i++)
		{
			Chassis_analysis(i+1);
			Chassis_SpeedLoopPID(wheel+i+1);
			
			/*�������*/
			tx_data [i*2]= (int)(wheel[i+1].Out)>>8;
			tx_data [i*2+1]= (int)(wheel[i+1].Out)&0xff;
			
		}
		/*2.����PID��������*/
		CAN_Send_Message(0x200,tx_data,8);
		/* ��ȷ5ms��ʱ */
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}

/**************************�����������***************************************************/
void Chassis_analysis(uint8_t i)
{
	/*����ǰ�������ݻ���ٶ�ֵ*/
	C610[i].Angle_Last = C610[i].Angle_Now;
	C610[i].Angle_Now = (uint16_t)((C610[i].Rx_Buffer [0]<<8)|C610[i].Rx_Buffer [1]);//0-8191 -> 0-360��
	C610[i].Speed_rpm = ( int16_t)((C610[i].Rx_Buffer [2]<<8)|C610[i].Rx_Buffer [3]) /36.f;
	
	wheel[i].Actual = (C610[i].Speed_rpm/60.f) * Chassis_WheelR ;
	//�������ֵ
	wheel[i].Torque =( (C610[i].Rx_Buffer [4] <<8) |C610[i].Rx_Buffer[5])*5.f/16384.f;
}

/***********************CAN�������շ��������*****************************************/
/**
  * @brief  CAN1��ʼ��
  * @note   ����1Mbps�����ʣ������жϽ���
  */
void C610_Init(void)
{
    /* CAN����������*/
    CAN_FilterTypeDef filter;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // ʹ��FIFO0
    filter.FilterBank = 0;             // ʹ�ù�������0
    filter.FilterMode = CAN_FILTERMODE_IDMASK; // ��ʶ������ģʽ
    filter.FilterScale = CAN_FILTERSCALE_32BIT; // 32λģʽ
    filter.FilterActivation = ENABLE;  // ���ù�����
    filter.SlaveStartFilterBank = 14;  // �ӹ������飨��CAN1��Ч��
	
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        Error_Handler();
    }
    
    /* ����CAN�������ж� */
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, 
        CAN_IT_RX_FIFO0_MSG_PENDING |  // FIFO0��Ϣ�ж�
        CAN_IT_ERROR |                 // �����ж�
        CAN_IT_BUSOFF |                // ���߹ر��ж�
        CAN_IT_LAST_ERROR_CODE         // ����������ж�
    );
}

/**
  * @brief  CAN�����жϻص�����
  * @note   �Զ������ݴ����Ӧ���У����������ݽ���
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
	uint8_t rx_len=CAN_Receive_Message(&rx_header,rx_data);
	
	if(rx_len==8){
		switch(rx_header.StdId)
		{
			case C610_1_ID:
				for(uint8_t i=0; i<rx_len; i++)
					C610[1].Rx_Buffer [i]= rx_data[i];
				break;
			case C610_2_ID:
				for(uint8_t i=0; i<rx_len; i++)
					C610[2].Rx_Buffer [i]= rx_data[i];
				break;
			case C610_3_ID:
				for(uint8_t i=0; i<rx_len; i++)
					C610[3].Rx_Buffer [i]= rx_data[i];
				break;
			case C610_4_ID:
				for(uint8_t i=0; i<rx_len; i++)
					C610[4].Rx_Buffer [i]= rx_data[i];
				break;
			default:  // δ֪ID���ݶ���
				break;
		}
	
	}
    /* ����и������ȼ����񱻻��� */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/**
  * @brief  ��װCAN��������
  * @note   
  */
uint8_t  CAN_Send_Message(uint32_t id,uint8_t *Data,uint8_t Len)
{
	CAN_TxHeaderTypeDef  can1txh;
	uint32_t tx_mail= CAN_TX_MAILBOX0;
	
	can1txh.DLC = Len ;
	can1txh .ExtId = id;
	can1txh .IDE = CAN_ID_STD ;//��׼֡
	can1txh .RTR = CAN_RTR_DATA ;//����֡
	can1txh .StdId = id;
	can1txh .TransmitGlobalTime = DISABLE;
	
	if(HAL_CAN_AddTxMessage (&hcan1 ,&can1txh ,Data, &tx_mail)!=HAL_OK )
		return 1;
	uint8_t free = HAL_CAN_GetTxMailboxesFreeLevel (&hcan1 );
	while(free!=3)
	{
		free = HAL_CAN_GetTxMailboxesFreeLevel (&hcan1 );
	}
	return 0;
	
}
/**
  * @brief  ��ȡCAN��������
  * @note   
  */
uint8_t CAN_Receive_Message(CAN_RxHeaderTypeDef *can1rxh,uint8_t *data)
{
//	CAN_RxHeaderTypeDef can1rxh;
	if(HAL_CAN_GetRxFifoFillLevel (&hcan1,CAN_RX_FIFO0 )==0)
	{
		return 0;
	}
	HAL_CAN_GetRxMessage (&hcan1 ,CAN_RX_FIFO0 ,can1rxh ,data);
	return can1rxh ->DLC ;//�������ݳ���
}


