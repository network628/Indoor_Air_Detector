/******************************************************************************
 * Include files
 ******************************************************************************/
#include "app.h"

#include "gpio.h"
#include "app_gpio.h"
#include "app_ad.h"
#include "tft_lcd.h"
#include "ddl.h"
#include "GUI_Control.h"
#include "lpm.h"
#include "wdt.h"
#include "uart.h"
#include "adc.h"
#include "flash.h"
#include "IR_NTC.h"
#include "app_timer.h"

uint16_t u16AdcRestult13;  //BAT_DET 电池电压采集

void bat_shan(void) 
{
	Gui_DrawFont_GBK24(160+24*2, 0, WHITE,BLACK, "零",10); delay1ms(100);
	Gui_DrawFont_GBK24(160+24*2, 0, WHITE,BLACK, "壹",10); delay1ms(100);
	Gui_DrawFont_GBK24(160+24*2, 0, WHITE,BLACK, "贰",10); delay1ms(100);
	Gui_DrawFont_GBK24(160+24*2, 0, WHITE,BLACK, "叁",10); delay1ms(100);
}

void Low_voltage_alarm_shutdown(void)   //低压关机
{
 
	uint16_t Bat_adH = 2590-50;  //2590  4.2 V
	uint16_t Bat_adM2 = 2400; //2400  3.9 V
	uint16_t Bat_adM1 = 2240; //2240  3.6 V
	uint16_t Bat_adL = 2200;  //2200  3.5 V
//	uint16_t Bat_adL = 2100;  //2100  3.3 V
	get_ad();  delay1ms(1); get_ad();  delay1ms(1); 
//	LCD_ShowxNum(120,0,u16AdcRestult13,4,24,0X80);
//	printf("--->BAT_AD=%d--->BAT_Value=%f V \r\n",u16AdcRestult13,u16AdcRestult13*3.1/4096 );
  if(u16AdcRestult13 > Bat_adL && u16AdcRestult13 < 2700)//2.0*4096/3.0 
	{
		get_ad();  delay1ms(1); get_ad();  delay1ms(1);
		if(u16AdcRestult13 > Bat_adH)  //-----------------------------------三格电
		{
      Gui_DrawFont_GBK24(160+24*2, 0, WHITE,BLACK, "叁",10);  
		}
		else if(u16AdcRestult13 > Bat_adM2 && u16AdcRestult13 < Bat_adH)  //-二格电  
		{
      Gui_DrawFont_GBK24(160+24*2, 0, WHITE,BLACK, "贰",10);   
		}
		else if(u16AdcRestult13 > Bat_adM1 && u16AdcRestult13 < Bat_adM2)  //一格电  
		{
      Gui_DrawFont_GBK24(160+24*2, 0, WHITE,BLACK, "壹",10);  
		}
		else if(u16AdcRestult13 > Bat_adL && u16AdcRestult13 < Bat_adM1)  //-----电池壳  
		{
      Gui_DrawFont_GBK24(160+24*2, 0, RED,BLACK, "零",10);  
		}
		POW_ON;   //3.0V总开关
		get_ad();
	}	
	else if(u16AdcRestult13 < Bat_adL && u16AdcRestult13 > 0)
	{
		deep_sleep();
	}
	 
}


void Wakeup_Init(void)  //深度睡眠初始化
{
	  stc_lpm_config_t stcConfig;
		stc_gpio_config_t stcGpioCfg;
	
	  Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    ///< 端口方向配置->输入
    stcGpioCfg.enDir = GpioDirIn;
    ///< 端口驱动能力配置->高驱动能力
    stcGpioCfg.enDrv = GpioDrvL;
    ///< 端口上下拉配置->上拉
    stcGpioCfg.enPuPd = GpioPu;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
    ///< 端口输入/输出值寄存器总线控制模式配置->AHB
    stcGpioCfg.enCtrlMode = GpioAHB;
   
    ///< GPIO IO PD04初始化(PD04在STK上外接KEY(USER))
    Gpio_Init(GpioPortD, GpioPin2, &stcGpioCfg); 
	
/***********************************************************************/
	///< 深度休眠模式下响应端口中断
    Gpio_SfIrqModeConfig(GpioSfIrqDpslpMode);
	///< 打开并配置PD04为下降沿中断
    Gpio_EnableIrq(GpioPortD, GpioPin2, GpioIrqFalling);
    ///< 使能端口PORTD系统中断
    EnableNvic(PORTD_IRQn, IrqLevel3, TRUE);
 
    ///< 低功耗模式配置
    stcConfig.enSEVONPEND   = SevPndDisable;
    stcConfig.enSLEEPDEEP   = SlpDpEnable;
//    stcConfig.enSLEEPONEXIT = SlpExtEnable;  //从中断运行
    stcConfig.enSLEEPONEXIT = SlpExtDisable;  //从MAIN运行
    Lpm_Config(&stcConfig);
}

void deep_sleep(void)
{
		timer0count =0;
		Sleep_flag = 1;   //  关机 
		Save_GPIO_State();    //保存GPIO状态
		delay1ms(100);
		POW_OFF;   //3.0V总开关
		IR_NTC_OFF;   //红外供电
	  IR_OFF;       //红外测距-开关
		PM2_5_OFF;    //PM2.5-开关
	  delay1ms(2000);
//		IR_NTC_ON;   //红外供电
//	  IR_ON;       //红外测距-开关
//	  POW_ON;      //3.0V总开关
//		PM2_5_ON;    //PM2.5-开关
//	  Lei_Da_OFF;   //PD06  雷达电源开关 1开  0关
//< 配置Demo板上所有不使用的IO为输出低
	  _LowPowerModeGpioSet();
		Wakeup_Init();
	  delay1ms(100);
		Lpm_GotoLpmMode();//打开休眠 
}

void Idle_shutdown(void)   //空闲关机
{
	 if(timer0count>15000)//60*10s
//   if(timer0count>1500)//60s
//	 if(timer0count>500)//20s
	 {
      deep_sleep();
	 }
	 
	 
}
 
static volatile uint32_t u32FlashTestFlag   = 0;

/*******************************************************************************
 * FLASH 中断服务函数
 ******************************************************************************/
 
 void FlashInt(void)  //FLASH初始化
 {
    if (TRUE == Flash_GetIntFlag(FlashPCInt))
    {
        Flash_ClearIntFlag(FlashPCInt);
        u32FlashTestFlag |= 0x01;
        Flash_DisableIrq(FlashPCInt);
    }
    if (TRUE == Flash_GetIntFlag(FlashSlockInt))
    {
        Flash_ClearIntFlag(FlashSlockInt);
        u32FlashTestFlag |= 0x02;
        Flash_DisableIrq(FlashSlockInt);
    }
      
 }
 
void Write_Flash_Data(uint32_t	Addr, char F_data)  //指定地址写整数
{
	///< FLASH目标扇区擦除
	if(Ok == Flash_SectorErase(Addr))
	{
//   printf("Flash_SectorErase ok\r\n");
	}
///< FLASH 字节写、校验
   Flash_WriteByte(Addr, F_data); 
}
 
char Read_Flash_Data(uint32_t	Addr)   //指定地址读整数
{
	  char  Data;
 
		Data = *((volatile uint8_t*)Addr);
 
	  return Data;
}

void Write_ATCV(uint32_t	Addr, float F_data) //指定地址写小数
{
	  uint8_t Atcv_H;
	  float K;
	  uint8_t Atcv_L;
	  Atcv_H = F_data;
	  K = F_data - Atcv_H;
	  Atcv_L = K*100;

///< FLASH目标扇区擦除
//	if(Ok == Flash_SectorErase(Addr))
//	{
////       printf("Flash_SectorErase ok\r\n");
//	}
	
	///< FLASH 字节写、校验
	 if (Ok == Flash_WriteByte(Addr, Atcv_H))
	 {
		 delay1ms(10);
//		 printf("Ok == Flash_WriteByte(Addr, Atcv_H)\r\n");
	 }
	 if (Ok ==Flash_WriteByte(Addr+1, Atcv_L))
	 {
		 delay1ms(10);
//		 printf("Ok == Flash_WriteByte(Addr, Atcv_L)\r\n");
	 }
}	

float Read_ATCV(uint32_t	Addr)  //指定地址读小数
{
	  uint8_t  R_Data;
	  float DataO;
	  float Data_L;
		R_Data = *((volatile uint8_t*)Addr);
//	printf("R_Data = %d  \t\r\n",R_Data);
	  Data_L = *((volatile uint8_t*)Addr+1);//小数部分
//	printf("Data_L = %3.2f  \t\r\n",Data_L);
		Data_L = Data_L / 100;
	  DataO = R_Data + Data_L;
	  return DataO;
}	

uint32_t	NTC_Addr=0xfff0;
void ADjust_NTC(void)     //手动温度补偿
{
	  char cnt=0;
		float ATCV[2];   // 环境温度校正值 32   42   
    BEEP_On_off;
    cnt =0;
	LCD_Clear(GRAY);
 //size:字体大小 12/16/24、32、40
	LCD_ShowString(40,10,12*21,24,24,"Calibration_UI");
//	In the calibration interface, left click to default and right click to correct
	while(1)
	{
//		timer3count =0;
		get_ad();delay1ms(100);
/////////////////////////////////////////校正保存////////////////////////////////////////////////	
		if((KEY_Right-200)<u16AdcRestult10 && u16AdcRestult10<(KEY_Right+200)) // 右键 
		{
			get_ad();delay1ms(100);
		  if((KEY_Right-200)<u16AdcRestult10 && u16AdcRestult10<(KEY_Right+200)) // 右键 
			{
				 Get_Temp();
			   ATCV[cnt] = Get_AIR(tempvalue);
				 ATCV[cnt] += getAvg(tempvalue);
			   cnt++;
				 if(cnt == 2)
				 {
					 cnt =0;
					 if(Ok == Flash_SectorErase(NTC_Addr))
						{
							for(cnt=0;cnt<2;cnt++)
							{
								Write_ATCV(NTC_Addr+cnt*2, ATCV[cnt]);
							}
						} 
						BEEP_On_off;
						LCD_Clear(BLACK);
						HomePage();
					 break;
				 }
			}
		}	
////////////////////////KEY_Left//------/////////////////////////////////////
		else if((KEY_Left-200)<u16AdcRestult10 && u16AdcRestult10<(KEY_Left+200)) 
		{
			get_ad();delay1ms(100);
		  if((KEY_Left-200)<u16AdcRestult10 && u16AdcRestult10<(KEY_Left+200)) 
			{
			   if(Ok == Flash_SectorErase(NTC_Addr))
					{//  L_Value            H_Value
//						ATCV[0] = 33.8;  ATCV[1] = 38.4;		//新板	
						ATCV[0] = 36.3;  ATCV[1] = 41.6;  //2020.5.14
						for(cnt=0;cnt<2;cnt++)
						{
							Write_ATCV(NTC_Addr+cnt*2, ATCV[cnt]);	
//						  printf("break222;  \r\n");
						}
					}
					BEEP_On_off;
					LCD_Clear(BLACK);
					HomePage();
         break;
			}
		}	
 
		if(!cnt)
		{
			LCD_ShowString(30,280,16*21,32,32,"35");
//			 gstcLcdDisplayCfg.Record	  = 35;
		}else{
			LCD_ShowString(30,280,16*21,32,32,"41");
//			 gstcLcdDisplayCfg.Record	  = 41;
		}
		Get_Temp();  //tempvalue
		ATCV[cnt] = Get_AIR(tempvalue);
		ATCV[cnt] += getAvg(tempvalue);
		
    LCD_ShowxNum(100,240,tempvalue*100,4,32,0X80);
		
	  LCD_ShowxNum(100,280,ATCV[cnt]*100,5,32,0X80);
// 	  printf("u16AdcRestult10 = %04d  \r\n",u16AdcRestult10);
		//			printf("ATCV[%d] = %3.2f  \r\n",cnt, ATCV[cnt]);
		delay1ms(200);
  }
}
 












