
#ifndef __APP_UART_H__
#define __APP_UART_H__

#include "stdio.h"      //包含需要的头文件
#include "stdarg.h"		//包含需要的头文件 
#include "string.h"     //包含需要的头文件
#include "stdint.h" 

//extern volatile uint8_t u8Rx0Data[50];
//extern uint8_t u8Rx0Cnt;

extern volatile uint8_t u8Rx1Data[50];
extern uint8_t u8Rx1Cnt;

//void Uart0_Init(void);
//void u0_printf(char*,...) ;         //串口0 printf函数

void Uart1_Init(void);
void u1_printf(char*,...) ;         //串口1 printf函数
	
#endif /* __UART_H__ */


