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

uint16_t u16AdcRestult13;  //BAT_DET ��ص�ѹ�ɼ�

void bat_shan(void) 
{
	Gui_DrawFont_GBK24(160+24*2, 0, WHITE,BLACK, "��",10); delay1ms(100);
	Gui_DrawFont_GBK24(160+24*2, 0, WHITE,BLACK, "Ҽ",10); delay1ms(100);
	Gui_DrawFont_GBK24(160+24*2, 0, WHITE,BLACK, "��",10); delay1ms(100);
	Gui_DrawFont_GBK24(160+24*2, 0, WHITE,BLACK, "��",10); delay1ms(100);
}

void Low_voltage_alarm_shutdown(void)   //��ѹ�ػ�
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
		if(u16AdcRestult13 > Bat_adH)  //-----------------------------------�����
		{
      Gui_DrawFont_GBK24(160+24*2, 0, WHITE,BLACK, "��",10);  
		}
		else if(u16AdcRestult13 > Bat_adM2 && u16AdcRestult13 < Bat_adH)  //-�����  
		{
      Gui_DrawFont_GBK24(160+24*2, 0, WHITE,BLACK, "��",10);   
		}
		else if(u16AdcRestult13 > Bat_adM1 && u16AdcRestult13 < Bat_adM2)  //һ���  
		{
      Gui_DrawFont_GBK24(160+24*2, 0, WHITE,BLACK, "Ҽ",10);  
		}
		else if(u16AdcRestult13 > Bat_adL && u16AdcRestult13 < Bat_adM1)  //-----��ؿ�  
		{
      Gui_DrawFont_GBK24(160+24*2, 0, RED,BLACK, "��",10);  
		}
		POW_ON;   //3.0V�ܿ���
		get_ad();
	}	
	else if(u16AdcRestult13 < Bat_adL && u16AdcRestult13 > 0)
	{
		deep_sleep();
	}
	 
}


void Wakeup_Init(void)  //���˯�߳�ʼ��
{
	  stc_lpm_config_t stcConfig;
		stc_gpio_config_t stcGpioCfg;
	
	  Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);

    ///< �˿ڷ�������->����
    stcGpioCfg.enDir = GpioDirIn;
    ///< �˿�������������->����������
    stcGpioCfg.enDrv = GpioDrvL;
    ///< �˿�����������->����
    stcGpioCfg.enPuPd = GpioPu;
    ///< �˿ڿ�©�������->��©����ر�
    stcGpioCfg.enOD = GpioOdDisable;
    ///< �˿�����/���ֵ�Ĵ������߿���ģʽ����->AHB
    stcGpioCfg.enCtrlMode = GpioAHB;
   
    ///< GPIO IO PD04��ʼ��(PD04��STK�����KEY(USER))
    Gpio_Init(GpioPortD, GpioPin2, &stcGpioCfg); 
	
/***********************************************************************/
	///< �������ģʽ����Ӧ�˿��ж�
    Gpio_SfIrqModeConfig(GpioSfIrqDpslpMode);
	///< �򿪲�����PD04Ϊ�½����ж�
    Gpio_EnableIrq(GpioPortD, GpioPin2, GpioIrqFalling);
    ///< ʹ�ܶ˿�PORTDϵͳ�ж�
    EnableNvic(PORTD_IRQn, IrqLevel3, TRUE);
 
    ///< �͹���ģʽ����
    stcConfig.enSEVONPEND   = SevPndDisable;
    stcConfig.enSLEEPDEEP   = SlpDpEnable;
//    stcConfig.enSLEEPONEXIT = SlpExtEnable;  //���ж�����
    stcConfig.enSLEEPONEXIT = SlpExtDisable;  //��MAIN����
    Lpm_Config(&stcConfig);
}

void deep_sleep(void)
{
		timer0count =0;
		Sleep_flag = 1;   //  �ػ� 
		Save_GPIO_State();    //����GPIO״̬
		delay1ms(100);
		POW_OFF;   //3.0V�ܿ���
		IR_NTC_OFF;   //���⹩��
	  IR_OFF;       //������-����
		PM2_5_OFF;    //PM2.5-����
	  delay1ms(2000);
//		IR_NTC_ON;   //���⹩��
//	  IR_ON;       //������-����
//	  POW_ON;      //3.0V�ܿ���
//		PM2_5_ON;    //PM2.5-����
//	  Lei_Da_OFF;   //PD06  �״��Դ���� 1��  0��
//< ����Demo�������в�ʹ�õ�IOΪ�����
	  _LowPowerModeGpioSet();
		Wakeup_Init();
	  delay1ms(100);
		Lpm_GotoLpmMode();//������ 
}

void Idle_shutdown(void)   //���йػ�
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
 * FLASH �жϷ�����
 ******************************************************************************/
 
 void FlashInt(void)  //FLASH��ʼ��
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
 
void Write_Flash_Data(uint32_t	Addr, char F_data)  //ָ����ַд����
{
	///< FLASHĿ����������
	if(Ok == Flash_SectorErase(Addr))
	{
//   printf("Flash_SectorErase ok\r\n");
	}
///< FLASH �ֽ�д��У��
   Flash_WriteByte(Addr, F_data); 
}
 
char Read_Flash_Data(uint32_t	Addr)   //ָ����ַ������
{
	  char  Data;
 
		Data = *((volatile uint8_t*)Addr);
 
	  return Data;
}

void Write_ATCV(uint32_t	Addr, float F_data) //ָ����ַдС��
{
	  uint8_t Atcv_H;
	  float K;
	  uint8_t Atcv_L;
	  Atcv_H = F_data;
	  K = F_data - Atcv_H;
	  Atcv_L = K*100;

///< FLASHĿ����������
//	if(Ok == Flash_SectorErase(Addr))
//	{
////       printf("Flash_SectorErase ok\r\n");
//	}
	
	///< FLASH �ֽ�д��У��
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

float Read_ATCV(uint32_t	Addr)  //ָ����ַ��С��
{
	  uint8_t  R_Data;
	  float DataO;
	  float Data_L;
		R_Data = *((volatile uint8_t*)Addr);
//	printf("R_Data = %d  \t\r\n",R_Data);
	  Data_L = *((volatile uint8_t*)Addr+1);//С������
//	printf("Data_L = %3.2f  \t\r\n",Data_L);
		Data_L = Data_L / 100;
	  DataO = R_Data + Data_L;
	  return DataO;
}	

uint32_t	NTC_Addr=0xfff0;
void ADjust_NTC(void)     //�ֶ��¶Ȳ���
{
	  char cnt=0;
		float ATCV[2];   // �����¶�У��ֵ 32   42   
    BEEP_On_off;
    cnt =0;
	LCD_Clear(GRAY);
 //size:�����С 12/16/24��32��40
	LCD_ShowString(40,10,12*21,24,24,"Calibration_UI");
//	In the calibration interface, left click to default and right click to correct
	while(1)
	{
//		timer3count =0;
		get_ad();delay1ms(100);
/////////////////////////////////////////У������////////////////////////////////////////////////	
		if((KEY_Right-200)<u16AdcRestult10 && u16AdcRestult10<(KEY_Right+200)) // �Ҽ� 
		{
			get_ad();delay1ms(100);
		  if((KEY_Right-200)<u16AdcRestult10 && u16AdcRestult10<(KEY_Right+200)) // �Ҽ� 
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
//						ATCV[0] = 33.8;  ATCV[1] = 38.4;		//�°�	
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
 












