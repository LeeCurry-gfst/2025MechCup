#ifndef __HardwareCodeInc_h
#define __HardwareCodeInc_h

#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
#include "i2c.h"
#include "can.h"
#include "tim.h"


#include "OLED.h"
#include "Serial.h"
#include "Chassis.h"
#include "Interaction.h"
#include "MPU6050/MPU6050.h"
#include "C610.h"
#include "Buzzer.h"
#include "Monitor.h"
#include "Others.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include <cmath>
#include <cstring>
//宏定义部分数学常数
#define EPSILON 0.000001f
#define PI 3.1415926f

//freertos句柄变量声明

extern osThreadId_t Chassis_CtrlHandle;
extern osThreadId_t My_InitHandle;
extern osThreadId_t UI_CmpHandle;
extern osThreadId_t UI_BLErockerHandle;
extern osThreadId_t UI_BLEarmHandle;
extern osThreadId_t UI_OLEDHandle;
extern osThreadId_t Status_MonitorHandle;
extern osThreadId_t Chassis_SpeedLoopHandle;
extern osThreadId_t UI_CmpTraHandle;
extern osThreadId_t BuzzerHandle;

extern osThreadId_t SteppingMotorHandle;
extern osThreadId_t MechArmHandle;
extern osThreadId_t LaunchHandle;

extern osMessageQueueId_t BLErockerRecHandle;
extern osMessageQueueId_t BLEarmRecHandle;
extern osMessageQueueId_t CmpRecHandle;
extern osMessageQueueId_t TestRecHandle;
extern osMessageQueueId_t TmpArmRecHandle;


#endif
