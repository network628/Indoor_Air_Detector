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

//#define u8Tx0Cnt   256    //定义串口0 发送缓冲区大小 256字节
//__align(8) char u8Tx0Data[u8Tx0Cnt];  

//void u0_printf(char* fmt,...)   //配置PA09 端口为URART0_TX   //配置PA10 端口为URART0_RX
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
//    Uart_SendData(UARTCH0, u8Tx0Data[i]); //启动UART0发送第一个字节
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
//    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);//使能GPIO模块时钟
//    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart0,TRUE);//使能uart0模块时钟
//    Uart0_PortInit();//串口端口初始化
//    
//    stcUartIrqCb.pfnRxIrqCb   = Rx0IntCallback;//中断入口地址
//    stcConfig.pstcIrqCb = &stcUartIrqCb;
//    stcConfig.bTouchNvic = TRUE;
//  
//		if(TRUE == stcConfig.bTouchNvic)
//		{
//			EnableNvic(UART0_IRQn,IrqLevel3,TRUE);
//		}
//    stcConfig.enRunMode = UartMode3;//模式3
//    stcConfig.enStopBit = Uart1bit;  //1bit停止位

//    stcMulti.enMulti_mode = UartNormal;//正常工作模式
//	  Uart_SetMultiMode(UARTCH1,&stcMulti);//多主机单独配置
//    enTb8 = UartEven;//偶校验
//    Uart_SetMMDOrCk(UARTCH0,enTb8);
//    
//    Uart_Init(UARTCH0, &stcConfig);//串口初始化
//    
//    Uart_SetClkDiv(UARTCH0,Uart8Or16Div);//采样分频
//    stcBaud.u32Pclk = Sysctrl_GetPClkFreq();
//    stcBaud.enRunMode = UartMode3;
//    stcBaud.u32Baud = 115200;
//    u16Scnt = Uart_CalScnt(UARTCH0,&stcBaud);//波特率计算
//    Uart_SetBaud(UARTCH0,u16Scnt);//波特率设置
//    
//	  Uart_ClrStatus(UARTCH0,UartRC);//清接收请求
//    Uart_EnableIrq(UARTCH0,UartRxIrq);//使能串口中断  
//    Uart_EnableFunc(UARTCH0,UartRx);//使能收
//    Uart_EnableFunc(UARTCH0,UartTx);//使能发
//}	 
 
/******************************************************************************/
 
/******************************************************************************/
#define u8TxCnt1   256    //定义串口1 发送缓冲区大小 256字节
__align(8) char u8TxData1[u8TxCnt1];  

void u1_printf(char* fmt,...)   //配置PA02 端口为URART1_TX   //配置PA03 端口为URART1_RX
{  
	unsigned int i,length;
	
	va_list ap;
	va_start(ap,fmt);
	vsprintf(u8TxData1,fmt,ap);
	va_end(ap);	
	
	length=strlen((const char*)u8TxData1);	
	for(i = 0;i < length;i ++)
	{			
    Uart_SendData(UARTCH1, u8TxData1[i]); //启动UART1发送第一个字节
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
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);//使能GPIO模块时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);//使能uart1模块时钟
    Uart1_PortInit();//串口端口初始化
    
    stcUartIrqCb.pfnRxIrqCb   = Rx1IntCallback;//中断入口地址
    stcConfig.pstcIrqCb = &stcUartIrqCb;
    stcConfig.bTouchNvic = TRUE;
  
		if(TRUE == stcConfig.bTouchNvic)
		{
			EnableNvic(UART1_IRQn,IrqLevel3,TRUE);
		}
    stcConfig.enRunMode = UartMode3;//模式3
    stcConfig.enStopBit = Uart1bit;  //1bit停止位

    stcMulti.enMulti_mode = UartNormal;//正常工作模式
	  Uart_SetMultiMode(UARTCH1,&stcMulti);//多主机单独配置
    enTb8 = UartEven;//偶校验
    Uart_SetMMDOrCk(UARTCH1,enTb8);
    
    Uart_Init(UARTCH1, &stcConfig);//串口初始化
    
    Uart_SetClkDiv(UARTCH1,Uart8Or16Div);//采样分频
    stcBaud.u32Pclk = Sysctrl_GetPClkFreq();
    stcBaud.enRunMode = UartMode3;
    stcBaud.u32Baud = 115200;
    u16Scnt = Uart_CalScnt(UARTCH1,&stcBaud);//波特率计算
    Uart_SetBaud(UARTCH1,u16Scnt);//波特率设置
    
	  Uart_ClrStatus(UARTCH1,UartRC);//清接收请求
    Uart_EnableIrq(UARTCH1,UartRxIrq);//使能串口中断  
    Uart_EnableFunc(UARTCH1,UartRx);//使能收
    Uart_EnableFunc(UARTCH1,UartTx);//使能发
}	 



