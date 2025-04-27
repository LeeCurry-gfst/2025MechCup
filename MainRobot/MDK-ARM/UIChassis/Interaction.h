#ifndef __Interaction_h
#define __Interaction_h

void UI_CmpPrintf(void);
void UI_OLEDPrintf(void);
void BLECtrl_RecDataInit(void);



struct BLEctrl{
	float Chassis_S;//遥控目标X轴速度,cm/s
	float Chassis_X;//遥控目标X轴速度,cm/s
	float Chassis_R;//遥控目标旋转速度,°/s
	
	float Arm1;//遥控目标转动角度,°
	float Arm2;//遥控目标转动角度,°
	float Arm3;//遥控目标转动角度,°
	float Arm4;//遥控目标转动角度,°
	uint8_t Arm5;//遥控目标吸取/脱离
	uint8_t Contianer_updown;//遥控控制升降
	uint8_t Pump;//遥控控制气泵开关
	uint8_t Contianer_pourout;//遥控控制倒出（并开盖）
	uint8_t Cleaner;//遥控控制开关吸尘器
};
extern struct BLEctrl BLEctrl_Command;



//==============电脑串口调试相关===============================================

#define huartCmp huart1 //蓝牙串口选用
#define USARTCmp USART1 //蓝牙串口选用

extern uint8_t CmpRxDa;//接收一字节电脑串口数据
extern uint8_t UI_CmpPrintfFlag;//电脑串口打印数据标志位
//===========================================================================


//==============摇杆遥控蓝牙串口================================================
#define huartBLErocker huart2 //蓝牙串口选用
#define USARTBLErocker USART2 //蓝牙串口选用
#define BLErockerLen 9 //串口数据包长度

#define GPIOBLErockerStatus GPIOA//蓝牙串口接收标志位，低电平点灯，0代表没有数据输入
#define GPIO_PIN_BLErockerStatus GPIO_PIN_8//蓝牙串口接收标志位，低电平点灯，0代表没有数据输入
extern uint8_t BLErockerRxDa; //接收一字节蓝牙串口数据
extern uint8_t BLErockerStatus;//蓝牙串口接收标志位，低电平点灯，0代表没有数据输入

//===========================================================================

//==============机械臂自定义控制器蓝牙串口======================================
#define huartBLEarm huart6 //蓝牙串口选用
#define USARTBLEarm USART6 //蓝牙串口选用
#define BLEarmLen 11 //串口数据包长度

#define GPIOBLEarmStatus GPIOA//蓝牙串口接收标志位，低电平点灯，0代表没有数据输入
#define GPIO_PIN_BLEarmStatus GPIO_PIN_8//蓝牙串口接收标志位，低电平点灯，0代表没有数据输入
extern uint8_t BLEarmRxDa; //接收一字节蓝牙串口数据
extern uint8_t BLEarmStatus;//蓝牙串口接收标志位，低电平点灯，0代表没有数据输入


extern uint8_t BLE_armTEMP[16];
//=============================================================================



extern uint16_t Test_PumpCmp;//测试气泵占空比
extern uint16_t Test_ValveCmp;//测试三通阀占空比

#endif
