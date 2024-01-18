/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "uart.h"
#include "gpio.h"
#include "sysctrl.h"

#include "app_uart.h"

//volatile uint8_t u8Rx0Data[50];
//uint8_t u8Rx0Cnt=0;

volatile uint8_t u8Rx1Data[50];
uint8_t u8Rx1Cnt=0;

//#define u8Tx0Cnt   256    //���崮��0 ���ͻ�������С 256�ֽ�
//__align(8) char u8Tx0Data[u8Tx0Cnt];  

//void u0_printf(char* fmt,...)   //����PA09 �˿�ΪURART0_TX   //����PA10 �˿�ΪURART0_RX
//{  
//	unsigned int i,length;
//	
//	va_list ap;
//	va_start(ap,fmt);
//	vsprintf(u8Tx0Data,fmt,ap);
//	va_end(ap);	
//	
//	length=strlen((const char*)u8Tx0Data);	
//	for(i = 0;i < length;i ++)
//	{			
//    Uart_SendData(UARTCH0, u8Tx0Data[i]); //����UART0���͵�һ���ֽ�
//    delay10us(50);		
//	}	
//}
// 

//void Rx0IntCallback(void)
//{
//    u8Rx0Data[u8Rx0Cnt]=Uart_ReceiveData(UARTCH0);
//    u8Rx0Cnt++;
//}

//void Uart0_PortInit(void)
//{
//    stc_gpio_config_t stcGpioCfg;
//    DDL_ZERO_STRUCT(stcGpioCfg);
//    stcGpioCfg.enDir = GpioDirOut;
//    Gpio_Init(GpioPortA,GpioPin9,&stcGpioCfg);
//    Gpio_SetAfMode(GpioPortA,GpioPin9,GpioAf1);//TX
//    stcGpioCfg.enDir = GpioDirIn;
//    Gpio_Init(GpioPortA,GpioPin10,&stcGpioCfg);
//    Gpio_SetAfMode(GpioPortA,GpioPin10,GpioAf1);//RX
//}

//void Uart0_Init(void)
//{
//    uint16_t u16Scnt = 0;
//    stc_uart_config_t  stcConfig;
//    stc_uart_irq_cb_t stcUartIrqCb;
//    stc_uart_multimode_t stcMulti;
//    stc_uart_baud_t stcBaud;
//    
//    en_uart_mmdorck_t enTb8;

//    DDL_ZERO_STRUCT(stcConfig);
//    DDL_ZERO_STRUCT(stcUartIrqCb);
//    DDL_ZERO_STRUCT(stcMulti);
//    DDL_ZERO_STRUCT(stcBaud);
//    
//    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);//ʹ��GPIOģ��ʱ��
//    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart0,TRUE);//ʹ��uart0ģ��ʱ��
//    Uart0_PortInit();//���ڶ˿ڳ�ʼ��
//    
//    stcUartIrqCb.pfnRxIrqCb   = Rx0IntCallback;//�ж���ڵ�ַ
//    stcConfig.pstcIrqCb = &stcUartIrqCb;
//    stcConfig.bTouchNvic = TRUE;
//  
//		if(TRUE == stcConfig.bTouchNvic)
//		{
//			EnableNvic(UART0_IRQn,IrqLevel3,TRUE);
//		}
//    stcConfig.enRunMode = UartMode3;//ģʽ3
//    stcConfig.enStopBit = Uart1bit;  //1bitֹͣλ

//    stcMulti.enMulti_mode = UartNormal;//��������ģʽ
//	  Uart_SetMultiMode(UARTCH1,&stcMulti);//��������������
//    enTb8 = UartEven;//żУ��
//    Uart_SetMMDOrCk(UARTCH0,enTb8);
//    
//    Uart_Init(UARTCH0, &stcConfig);//���ڳ�ʼ��
//    
//    Uart_SetClkDiv(UARTCH0,Uart8Or16Div);//������Ƶ
//    stcBaud.u32Pclk = Sysctrl_GetPClkFreq();
//    stcBaud.enRunMode = UartMode3;
//    stcBaud.u32Baud = 115200;
//    u16Scnt = Uart_CalScnt(UARTCH0,&stcBaud);//�����ʼ���
//    Uart_SetBaud(UARTCH0,u16Scnt);//����������
//    
//	  Uart_ClrStatus(UARTCH0,UartRC);//���������
//    Uart_EnableIrq(UARTCH0,UartRxIrq);//ʹ�ܴ����ж�  
//    Uart_EnableFunc(UARTCH0,UartRx);//ʹ����
//    Uart_EnableFunc(UARTCH0,UartTx);//ʹ�ܷ�
//}	 
 
/******************************************************************************/
 
/******************************************************************************/
#define u8TxCnt1   256    //���崮��1 ���ͻ�������С 256�ֽ�
__align(8) char u8TxData1[u8TxCnt1];  

void u1_printf(char* fmt,...)   //����PA02 �˿�ΪURART1_TX   //����PA03 �˿�ΪURART1_RX
{  
	unsigned int i,length;
	
	va_list ap;
	va_start(ap,fmt);
	vsprintf(u8TxData1,fmt,ap);
	va_end(ap);	
	
	length=strlen((const char*)u8TxData1);	
	for(i = 0;i < length;i ++)
	{			
    Uart_SendData(UARTCH1, u8TxData1[i]); //����UART1���͵�һ���ֽ�
    delay10us(50);		
	}	
}
 

void Rx1IntCallback(void)
{
    u8Rx1Data[u8Rx1Cnt]=Uart_ReceiveData(UARTCH1);
    u8Rx1Cnt++;
}

void Uart1_PortInit(void)
{
    stc_gpio_config_t stcGpioCfg;
    DDL_ZERO_STRUCT(stcGpioCfg);
    stcGpioCfg.enDir = GpioDirOut;
    Gpio_Init(GpioPortA,GpioPin2,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin2,GpioAf1);//TX
    stcGpioCfg.enDir = GpioDirIn;
    Gpio_Init(GpioPortA,GpioPin3,&stcGpioCfg);
    Gpio_SetAfMode(GpioPortA,GpioPin3,GpioAf1);//RX
}

void Uart1_Init(void)
{
    uint16_t u16Scnt = 0;
    stc_uart_config_t  stcConfig;
    stc_uart_irq_cb_t stcUartIrqCb;
    stc_uart_multimode_t stcMulti;
    stc_uart_baud_t stcBaud;
    
    en_uart_mmdorck_t enTb8;

    DDL_ZERO_STRUCT(stcConfig);
    DDL_ZERO_STRUCT(stcUartIrqCb);
    DDL_ZERO_STRUCT(stcMulti);
    DDL_ZERO_STRUCT(stcBaud);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);//ʹ��GPIOģ��ʱ��
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);//ʹ��uart1ģ��ʱ��
    Uart1_PortInit();//���ڶ˿ڳ�ʼ��
    
    stcUartIrqCb.pfnRxIrqCb   = Rx1IntCallback;//�ж���ڵ�ַ
    stcConfig.pstcIrqCb = &stcUartIrqCb;
    stcConfig.bTouchNvic = TRUE;
  
		if(TRUE == stcConfig.bTouchNvic)
		{
			EnableNvic(UART1_IRQn,IrqLevel3,TRUE);
		}
    stcConfig.enRunMode = UartMode3;//ģʽ3
    stcConfig.enStopBit = Uart1bit;  //1bitֹͣλ

    stcMulti.enMulti_mode = UartNormal;//��������ģʽ
	  Uart_SetMultiMode(UARTCH1,&stcMulti);//��������������
    enTb8 = UartEven;//żУ��
    Uart_SetMMDOrCk(UARTCH1,enTb8);
    
    Uart_Init(UARTCH1, &stcConfig);//���ڳ�ʼ��
    
    Uart_SetClkDiv(UARTCH1,Uart8Or16Div);//������Ƶ
    stcBaud.u32Pclk = Sysctrl_GetPClkFreq();
    stcBaud.enRunMode = UartMode3;
    stcBaud.u32Baud = 115200;
    u16Scnt = Uart_CalScnt(UARTCH1,&stcBaud);//�����ʼ���
    Uart_SetBaud(UARTCH1,u16Scnt);//����������
    
	  Uart_ClrStatus(UARTCH1,UartRC);//���������
    Uart_EnableIrq(UARTCH1,UartRxIrq);//ʹ�ܴ����ж�  
    Uart_EnableFunc(UARTCH1,UartRx);//ʹ����
    Uart_EnableFunc(UARTCH1,UartTx);//ʹ�ܷ�
}	 



