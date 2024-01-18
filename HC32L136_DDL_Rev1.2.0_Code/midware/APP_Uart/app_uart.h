
#ifndef __APP_UART_H__
#define __APP_UART_H__

#include "stdio.h"      //������Ҫ��ͷ�ļ�
#include "stdarg.h"		//������Ҫ��ͷ�ļ� 
#include "string.h"     //������Ҫ��ͷ�ļ�
#include "stdint.h" 

//extern volatile uint8_t u8Rx0Data[50];
//extern uint8_t u8Rx0Cnt;

extern volatile uint8_t u8Rx1Data[50];
extern uint8_t u8Rx1Cnt;

//void Uart0_Init(void);
//void u0_printf(char*,...) ;         //����0 printf����

void Uart1_Init(void);
void u1_printf(char*,...) ;         //����1 printf����
	
#endif /* __UART_H__ */


