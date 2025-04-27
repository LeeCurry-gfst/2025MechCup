//********
//此部分代码用于执行底盘相关操控，包括
//*蓝牙遥控数据的进一步处理（运动学解算函数封装）
//*UI和底盘部分的初始化
//*基于MPU6050的六轴陀螺仪的角度环
//*定义电机编号：1 3
//               2 4
//********

#include "HardwareCodeInc.h"

struct Chassis_PID wheel[5];
struct Chassis_PID angle;/*CAN总线控制*/
uint32_t Kpid[4];//缓存ADC读取得Kpid值

/**
  * @brief  基于MPU6050的pid位置闭环，每100ms进行一次运算
  * @param  argument: Not used
  * @retval None
  */
void Chassis_anglePID_Calculation(void)
{
	/*获取当前目标值（偏航角累加）*/
	if(BLEctrl_Command.Chassis_S >= 0) angle .Target += BLEctrl_Command.Chassis_R * angle.dt;
	else angle .Target -= BLEctrl_Command.Chassis_R * angle.dt;//修正 后转弯
	
	/*获取当前实际值*/
	float yaw
//		,pitch,roll
	;//单位：角度
//	MPU_DMP_GetData(&pitch,&roll,&yaw );
	angle .Actual = yaw;
	
	/*获取当前所需的PID值，分为平路上坡调试三种*/
	float kp,ki,kd;
	if(angle.KpidFlag == Normal ) kp=angle.Kp,ki=angle.Ki ,kd=angle.Kd;
	
	/*调试状态下，通过旋转电位器实时调控PID参数，获取目前ADC测得调制PID的值 4095->20映射*/
	if(angle.KpidFlag == Debug  )
	{
		kp = Kpid[0]/4095.0*2;
		ki = Kpid[1]/4095.0*2;
		kd = Kpid[2]/4095.0*2;
		angle.Kp = Kpid[0]/4095.0*2;
		angle.Ki = Kpid[1]/4095.0*2;
		angle.Kd = Kpid[2]/4095.0*2;
	}
	
	/*获取本次与上次误差*/
	angle.ErrorLast =angle.ErrorNow ;
	angle.ErrorNow = angle.Target - (angle.Actual) ;
	
	/*误差累计*/
	if(fabs(angle.ErrorNow) <= angle.ErrorLim )//积分分离
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
	
	/*PID计算**  Kp*EN+Ki*ES+Kd*(EN-EL)  ，得到目标角速度*/
	angle.Out = kp * angle.ErrorNow + ki * angle.ErrorSum + kd * (angle.ErrorNow - angle.ErrorLast); 
	
	/*防止超调*/
	if(angle.Out >150 ) angle.Out=150;
	if(angle.Out <-150 ) angle.Out=-150;
}

/**
  * @brief  遥控命令执行与运动学解算
  * @param  无
  * @retval 无
  */
void Chassis_TeleCommandExecute(void)
{
	float R_rad;
	if(angle.KpidFlag != OFF) R_rad = angle .Out /360.0f*2.0f*PI;//外环PID控制精准转动
	else R_rad = BLEctrl_Command.Chassis_R /360.0f*2.0f*PI;//直接使用遥控数据控制转动，转弧度制角速度
	
	if(BLEctrl_Command.Chassis_S <0) R_rad*=-1;//修正 后转弯
//	
//	/*差速转向运动学解算*/
//	wheel [1].Target = BLEctrl_Command.Chassis_S + R_rad * Chassis_Width / 2;
//	wheel [2].Target = BLEctrl_Command.Chassis_S + R_rad * Chassis_Width / 2;
//	wheel [3].Target = BLEctrl_Command.Chassis_S - R_rad * Chassis_Width / 2;
//	wheel [4].Target = BLEctrl_Command.Chassis_S - R_rad * Chassis_Width / 2;
//	wheel [3].Target*=-1;
//	wheel [4].Target*=-1;
	/*麦轮运动学解算*/
	
	wheel [1].Target = BLEctrl_Command.Chassis_S - BLEctrl_Command.Chassis_X +R_rad*(Chassis_Width+Chassis_Length)/2.0f;
	wheel [2].Target = BLEctrl_Command.Chassis_S + BLEctrl_Command.Chassis_X +R_rad*(Chassis_Width+Chassis_Length)/2.0f;
	wheel [3].Target = BLEctrl_Command.Chassis_S + BLEctrl_Command.Chassis_X -R_rad*(Chassis_Width+Chassis_Length)/2.0f;
	wheel [4].Target = BLEctrl_Command.Chassis_S - BLEctrl_Command.Chassis_X -R_rad*(Chassis_Width+Chassis_Length)/2.0f;
	wheel [3].Target*=-1;
	wheel [4].Target*=-1;
	
}
/**
  * @brief PID参数初始化
  * @param  初始化的电机PID参数地址
  * @retval 无
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
/******************FreeRTOS任务*********************/
/**
  * @brief  部分硬件和外设的初始化
  * @param  argument: Not used
  * @retval None
  */
void StartMy_Init(void *argument)
{
	//上电延时等待外部器件上电完成
	HAL_Delay(1000);
	
	
	//开启外设
	HAL_ADC_Start_DMA (&hadc3 ,Kpid ,4);
	HAL_ADC_Start(&hadc3 );
	
	__HAL_UART_ENABLE_IT(&huartCmp,UART_IT_IDLE);//开启空闲中断
	__HAL_UART_ENABLE_IT(&huartBLErocker,UART_IT_IDLE);//开启空闲中断
	__HAL_UART_ENABLE_IT(&huartBLEarm,UART_IT_IDLE);//开启空闲中断
	USART6->CR1|=USART_CR1_IDLEIE;
	
	HAL_UART_Receive_IT(&huartCmp,&CmpRxDa,1);
	HAL_UART_Receive_IT(&huartBLEarm, &BLEarmRxDa , 1);	
	HAL_UART_Receive_IT(&huartBLErocker, &BLErockerRxDa , 1);	
	
	//初始化功能模块
	BLECtrl_RecDataInit();
	Chassis_PID_Init();
	Others_Init();//底盘外所有模块，包括机械臂、风机、气泵、三通阀
	
	//初始化挂载模块
	OLED_Init ();
	OLED_Update();
	C610_Init();
	Serial_Printf ("Start!\r\n");
//	MPU_DMP_Init ();//初始化MPU6050
	
	
	HAL_Delay(1000);
	//上电完成播放音效
	Buzzer_StartUp();
	
	vTaskDelete(NULL);
}
/**
  * @brief  角度环闭环
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
