/****
*本部分代码仅为电脑串口发送数据的底层操作
***/

#include "main.h"      
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include "HardwareCodeInc.h"
uint8_t Serial_RxData;		//定义串口接收的数据变量
uint8_t Serial_RxFlag;		//定义串口接收的标志位变量


/**串口宏定义**/
/*注意在定义处加上第一次中断接收*/
#define Ser_huart huart1
#define Ser_USART USART1


/**
  * 函    数：自己封装的prinf函数
  * 参    数：format 格式化字符串
  * 参    数：... 可变的参数列表
  * 返 回 值：无
  */
void Serial_Printf(char *format, ...)
{
	char String[100];				//定义字符数组
	uint8_t len;
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	len = vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);					//结束变量arg
	
	while(HAL_UART_Transmit_DMA (&Ser_huart,(uint8_t *)String,len)!=HAL_OK){
	Serial_RxFlag=1;
	}
	
	osDelay (20);//适当延时避免DMA传输数据错误
}
