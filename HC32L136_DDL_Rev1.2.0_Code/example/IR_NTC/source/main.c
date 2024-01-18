#include "app_uart.h"
#include "stdio.h"
#include "ddl.h"
#include "app_ad.h"
#include "app_gpio.h"
#include "app.h"
#include "tft_lcd.h"
//#include "font.h"
#include "IR_NTC.h"
#include "gpio.h"
#include "GUI_Control.h"
#include "flash.h"
#include "sysctrl.h"
#include "uart.h"
#include "app_timer.h"
#include "lpm.h"
#include "SHT2X.h"
#include "PM1006.h"

uint16_t u16AdcRestult0;     //PA00 ad_air
uint16_t u16AdcRestult1;     //PA01 ad_ntc
//uint16_t u16AdcRestult13;  //BAT_DET 电池电压采集
uint16_t u16AdcRestult10;    //PC00
uint16_t u16AdcRestult12;    //PC12
 
uint16_t KEY_Left  = 1661;  //�? -- 1.2v
uint16_t KEY_Right = 3236;  //�? ++ 2.36v

uint8_t Sleep_flag=0;  //睡眠标志�?  //0  开机状�?  //1  关机状�?
uint16_t Pople_num=0;    //系统计时�?

float Air_T;  

char people = 1;
char TX1Data[10];
 
void APP_Init(void);
void Wakeup_Init(void);
void RCC_Init(void);

	
int32_t main(void)
{
	APP_Init();     
	LCD_LED(1);
    while(1)
    {  
////////////////////////////////////////////////////////////////////////////////////////			/
		  if(Sleep_flag == 1)  //休眠唤醒后初始化全部设�?和IO
			{
				Sleep_flag = 0;  //  就开�? 
				APP_Init();     LCD_LED(1);
				Get_Temp();
				get_ad();     delay1ms(20);
				get_ad();     delay1ms(20);
				while(FALSE == Gpio_GetInputIO(GpioPortD, GpioPin2));
			}
			else if(FALSE == Gpio_GetInputIO(GpioPortD, GpioPin2) && Sleep_flag == 0)   //按下PD2关机
      {
				delay1ms(10);
				if(FALSE == Gpio_GetInputIO(GpioPortD, GpioPin2))   //按下PD2关机
				{
				  deep_sleep();
				}
			}
			 
			
			
			if(timer0count%50>10)
			{
 
				/////////////////////////////充电指示�?///////////////////////////////////////
        if(TRUE == Gpio_GetInputIO(GpioPortC, GpioPin13) && u16AdcRestult13<2550)   //    充电检测口
				{
					bat_shan();
          LED_RED_ON;
					LED_GREEN_OFF;
				}
				else if(TRUE == Gpio_GetInputIO(GpioPortC, GpioPin13) && u16AdcRestult13>2550)
				{ 
					LED_RED_OFF;
					LED_GREEN_ON;
				} 
				else
				{
				  LED_RED_OFF;
					LED_GREEN_OFF;
				}
				
				Is_AQM_D_out(); 
				Get_Temp();
//				Idle_shutdown();   //空闲60S关机
				Low_voltage_alarm_shutdown();   //电池检�? 低压关机	
			}


 
			
			if(timer0count>5) //有人
			{
				people =1;
			}		
			else  //无人
			{
				people =0;
			}	
////////////////////////////////////////////////////////////////////////
      if(u16AdcRestult12<3800) //有手
			{
			  hand=1;
			}	
//      LCD_ShowxNum(120,25,timer0count,4,24,0X80);			
			/************检测PD04电平(USER按键�?��按下(低电�?)) **********/
			get_ad();  delay1ms(10); get_ad();  delay1ms(10);
			if((Sleep_flag == 0)&&(u16AdcRestult12<3900) && (hand==1))  //红�?对射检�? �?��有手
			{ 
				delay1ms(30);  
				if(u16AdcRestult12<3900 && timer0count>30)
				{
//					timer0count =0;
					hand=0;
					Air_T = Test_Temp();
 
//			  u1_printf("\r\nAir_T = %3.2f\r\n",Air_T);
					Blue_Data(); 
//				printf("eCO2=%04dppm Temp_C=%3.2f'C Hum_RH=%3.2f%%RH TVOC=%04dug/m3 HCHO=%04dug/m3\r\n",Is_AQM.eCO2,Is_AQM.Temp_C,Is_AQM.Hum_RH,Is_AQM.TVOC,Is_AQM.HCHO);
				}
			} 
			else
			{
			  Air_T = 0;
			}
			
		if(timer0count%20>10)	
      Disp_Updata(); 
//		printf("u16AdcRestult10 = %04d \r\n",u16AdcRestult10);
//   	printf("timer0count = %06d \r\n",timer0count);
//	  delay1ms(200);
    }
}
 
char voice;

void APP_Init(void)
{
        voice = 1;
	    Air_Tm = 0;
	    RCC_Init();LCD_LED(0);
		GPIO_PortInit();	
        LCD_LED(0);
		IR_NTC_ON;   //红�?供电
	    IR_ON;       //红�?测距-开�?
	    POW_ON;      //3.0V总开�?
		PM2_5_ON;    //PM2.5-开�?
	    Lei_Da_OFF;   //PD06  雷达电源开�? 1开  0�?
        BEEP_On_off;
//    delay1ms(1000);
		Uart1_Init();
		AD_Init();
		LCD_Init();	
		I2C_Configuration(); 
		SHT2X_Init();
		Tim0Init();
		Tim3Init(); //PM1006
	
		///< �?��初�?化�?�?��行后方能进�?FLASH编程操作，FLASH初�?化（�?��函数,编程时间,休眠模式配置�?
		while(Ok != Flash_Init(FlashInt, 12, TRUE)) 
		{
		//		   printf("Flash_Init  error\r\n");
		}
 
    	HomePage();
		u1_printf("AIR_Check_V1.0 2020-6-15 \t\r\n"); 
		
		get_ad();     delay1ms(20); get_ad();     delay1ms(20);  //进入校�?界面
		if((TRUE == Gpio_GetInputIO(GpioPortC, GpioPin13)) && ((KEY_Left-200)<u16AdcRestult10 && u16AdcRestult10<(KEY_Left+200))) 
		{                           //充电口�?�?                   左键
			delay1ms(30);
			if((TRUE == Gpio_GetInputIO(GpioPortC, GpioPin13)) && ((KEY_Left-200)<u16AdcRestult10 && u16AdcRestult10<(KEY_Left+200))) 
			{
				LCD_LED(1);
				BEEP_On_off;
				ADjust_NTC();	
				u1_printf("--->(ADjust_NTC)\r\n");
			}
		}
}

///******************************************************************************
// * Local function prototypes ('static')
// ******************************************************************************/
void Gpio_IRQHandler(uint8_t u8Param)
{
	  stc_lpm_config_t stcConfig;
    if(3 == u8Param)  ///< PORT D
    {
			delay1ms(100);
//			if(TRUE == Gpio_GetIrqStatus(GpioPortD, GpioPin2))
			if((TRUE == Gpio_GetIrqStatus(GpioPortD, GpioPin2))&&(Sleep_flag == 1))
			{	 
//				if(Sleep_flag == 1) //  关机状�?
				{
					
					///< 低功耗模式配�?
					stcConfig.enSEVONPEND   = SevPndDisable;
					stcConfig.enSLEEPDEEP   = SlpDpEnable;
					stcConfig.enSLEEPONEXIT = SlpExtDisable;  //从MAIN运�?
					Lpm_Config(&stcConfig);
					Gpio_ClearIrq(GpioPortD, GpioPin2);	
					Gpio_DisableIrq(GpioPortD, GpioPin2,GpioIrqFalling);
					Recovery_GPIO_State();
 
					IR_NTC_ON;   //红�?供电
					IR_ON;       //红�?测距-开�?
					POW_ON;      //3.0V总开�?
					PM2_5_ON;    //PM2.5-开�?
					Lei_Da_OFF;   //PD06  雷达电源开�? 1开  0�?
				}
			}
    }
} 
/******************************************************************************/
 
/******************************************************************************/
void RCC_Init(void)
{
    stc_sysctrl_clk_config_t stcCfg;

    stc_sysctrl_pll_config_t stcPLLCfg;
 
    ///< 开�?��要使用的各个外�?的时�?
    ///< 开启GPIO外�?时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    ///< 开启FLASH外�?时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralFlash, TRUE);
    
    
    ///< 因将要倍�?的PLL作为系统时钟HCLK会达�?48MHz：所以�?处�?先�?置FLASH 读等待周期为1 cycle(默�?值为0 cycle)
    Flash_WaitCycle(FlashWaitCycle1); 
    

    ///< 时钟初�?化前，优先�?�??使用的时钟源：�?处配置PLL
    Sysctrl_SetRCHTrim(SysctrlRchFreq4MHz);             //PLL使用RCH作为时钟源，因�?需要先设置RCH    
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq4_6MHz;     //RCH 4MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  //PLL 输出48MHz
    stcPLLCfg.enPllClkSrc = SysctrlPllRch;              //输入时钟源选择RCH
    stcPLLCfg.enPllMul    = SysctrlPllMul12;            //4MHz x 12 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg);   

    ///< 选择PLL作为HCLK时钟�?;
    stcCfg.enClkSrc  = SysctrlClkPLL;
    ///< HCLK SYSCLK/2
    stcCfg.enHClkDiv = SysctrlHclkDiv1;
    ///< PCLK 为HCLK/8
    stcCfg.enPClkDiv = SysctrlPclkDiv1;
    ///< 系统时钟初�?�?
    Sysctrl_ClkInit(&stcCfg);
}	
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

 

