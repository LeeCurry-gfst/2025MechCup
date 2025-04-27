//********
//此部分代码用于执行无刷电机相关操控，包括
//*CAN数据收发函数封装
//*PID算法调控M2006电机转速，（速度环）
//*控制C610电调的正常工作
//********

#include "HardwareCodeInc.h"

struct M2006_Para C610[5];
/**
* @brief PID闭环控制电机转速
* @param argument: Not used
* @retval None
*/
void Chassis_SpeedLoopPID(struct Chassis_PID *Wheel_PID)
{
	
	if(Wheel_PID->KpidFlag == OFF )  {Wheel_PID->Out=0;return ;}
	/*获取当前目标值：运动学解算，在接收遥控数据的同时进行*/
	/*获取当前实际值：通过CAN接收完成获取*/
	
	/*获取当前所需的PID值，分为正常、调试三种*/
	float kp,ki,kd;
	if(Wheel_PID->KpidFlag == Normal ) kp=Wheel_PID->Kp,ki=Wheel_PID->Ki ,kd=Wheel_PID->Kd;
	if(Wheel_PID->KpidFlag == Uphill ) kp=Wheel_PID->Kpuh,ki=Wheel_PID->Kiuh ,kd=Wheel_PID->Kduh;
	if(Wheel_PID->KpidFlag == Upstair ) kp=Wheel_PID->Kpus,ki=Wheel_PID->Kius ,kd=Wheel_PID->Kdus;
	
	/*调试状态下，通过旋转电位器实时调控PID参数，获取目前ADC测得调制PID的值 4095->20映射*/
	if(Wheel_PID->KpidFlag == Debug  )
	{
		kp = Kpid[0]/4095.0*100;
		ki = Kpid[1]/4095.0*100;
		kd = Kpid[2]/4095.0*100;
		Wheel_PID->Kp = Kpid[0]/4095.0*3000;
		Wheel_PID->Ki = Kpid[1]/4095.0*500;
		Wheel_PID->Kd = Kpid[2]/4095.0*500;
		
	}
	
	/*获取本次与上次误差*/
	Wheel_PID->ErrorLast =Wheel_PID->ErrorNow ;
	Wheel_PID->ErrorNow = Wheel_PID->Target - (Wheel_PID->Actual) ;
	
	/*误差累计*/
	if(fabs(Wheel_PID->ErrorNow) <= Wheel_PID->ErrorLim )//积分分离
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
	
	/*PID计算**  Kp*EN+Ki*ES+Kd*(EN-EL)  ，得到目标角速度*/
	Wheel_PID->Out = kp * Wheel_PID->ErrorNow + ki * Wheel_PID->ErrorSum + kd * (Wheel_PID->ErrorNow - Wheel_PID->ErrorLast); 
	
	/*防止超调*/
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
  * @brief  电机速度环闭环与CAN发送任务（FreeRTOS任务）
  * @note   精确5ms周期发送，采用绝对延时保证时序
  */
void StartChassis_SpeedLoop(void *argument)
{
	uint8_t tx_data[8] = {0,0,0,0,0,0,0,0};
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
		/*1.执行速度环PID控制*/
		for(uint8_t i=0; i<4; i++)
		{
			Chassis_analysis(i+1);
			Chassis_SpeedLoopPID(wheel+i+1);
			
			/*数据填充*/
			tx_data [i*2]= (int)(wheel[i+1].Out)>>8;
			tx_data [i*2+1]= (int)(wheel[i+1].Out)&0xff;
			
		}
		/*2.发送PID解算数据*/
		CAN_Send_Message(0x200,tx_data,8);
		/* 精确5ms延时 */
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}

/**************************解析电调数据***************************************************/
void Chassis_analysis(uint8_t i)
{
	/*基于前两个数据获得速度值*/
	C610[i].Angle_Last = C610[i].Angle_Now;
	C610[i].Angle_Now = (uint16_t)((C610[i].Rx_Buffer [0]<<8)|C610[i].Rx_Buffer [1]);//0-8191 -> 0-360°
	C610[i].Speed_rpm = ( int16_t)((C610[i].Rx_Buffer [2]<<8)|C610[i].Rx_Buffer [3]) /36.f;
	
	wheel[i].Actual = (C610[i].Speed_rpm/60.f) * Chassis_WheelR ;
	//获得力矩值
	wheel[i].Torque =( (C610[i].Rx_Buffer [4] <<8) |C610[i].Rx_Buffer[5])*5.f/16384.f;
}

/***********************CAN配置与收发电调数据*****************************************/
/**
  * @brief  CAN1初始化
  * @note   配置1Mbps波特率，启用中断接收
  */
void C610_Init(void)
{
    /* CAN过滤器配置*/
    CAN_FilterTypeDef filter;
    filter.FilterIdHigh = 0x0000;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 使用FIFO0
    filter.FilterBank = 0;             // 使用过滤器组0
    filter.FilterMode = CAN_FILTERMODE_IDMASK; // 标识符掩码模式
    filter.FilterScale = CAN_FILTERSCALE_32BIT; // 32位模式
    filter.FilterActivation = ENABLE;  // 启用过滤器
    filter.SlaveStartFilterBank = 14;  // 从过滤器组（仅CAN1有效）
	
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        Error_Handler();
    }
    
    /* 启动CAN并激活中断 */
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, 
        CAN_IT_RX_FIFO0_MSG_PENDING |  // FIFO0消息中断
        CAN_IT_ERROR |                 // 错误中断
        CAN_IT_BUSOFF |                // 总线关闭中断
        CAN_IT_LAST_ERROR_CODE         // 最后错误代码中断
    );
}

/**
  * @brief  CAN接收中断回调函数
  * @note   自动将数据存入对应队列，不进行数据解析
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
			default:  // 未知ID数据丢弃
				break;
		}
	
	}
    /* 如果有更高优先级任务被唤醒 */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/**
  * @brief  封装CAN发送任务
  * @note   
  */
uint8_t  CAN_Send_Message(uint32_t id,uint8_t *Data,uint8_t Len)
{
	CAN_TxHeaderTypeDef  can1txh;
	uint32_t tx_mail= CAN_TX_MAILBOX0;
	
	can1txh.DLC = Len ;
	can1txh .ExtId = id;
	can1txh .IDE = CAN_ID_STD ;//标准帧
	can1txh .RTR = CAN_RTR_DATA ;//数据帧
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
  * @brief  获取CAN接收数据
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
	return can1rxh ->DLC ;//返回数据长度
}


