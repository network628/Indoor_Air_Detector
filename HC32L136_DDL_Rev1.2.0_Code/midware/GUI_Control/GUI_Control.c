#include "GUI_Control.h"
#include "ddl.h" 
#include "tft_lcd.h"
#include "stdio.h"
#include "SHT2X.h"
#include "IR_NTC.h"
#include "app_gpio.h"
#include "PM1006.h"
#include "icon.h"
#include "app_ad.h"
#include "uart.h"
#include "app_ad.h"
#include "string.h"
#include "app_uart.h"

char hand=1;

uint32_t u32PwcCapValue;  //PM1006

uint8_t dataC_buf[16];
uint8_t data_buf[16];
void Blue_Data(void)
{
	char i;
	float Air_Tb;           //红外温度
	Air_Tb = Air_T;
	Air_Tb*=10;
	Is_AQM.Temp_C*=10;
	Is_AQM.Hum_RH*=10;
	data_buf[0] = 0x80;    
	data_buf[1] = 0x01;
	data_buf[2] = ((int)Is_AQM.Temp_C>>8)&0x00ff;  //Is_AQM.Temp_C
	data_buf[3] = ((int)Is_AQM.Temp_C)&0x00ff;
	data_buf[4] = (Is_AQM.eCO2>>8)&0x00ff;  //Is_AQM.eCO2 
	data_buf[5] = (Is_AQM.eCO2)&0x00ff;
	data_buf[6] = (Is_AQM.TVOC>>8)&0x00ff;   //Is_AQM.TVOC
	data_buf[7] = (Is_AQM.TVOC)&0x00ff;
	data_buf[8] = (Is_AQM.HCHO>>8)&0x00ff;   //Is_AQM.HCHO
	data_buf[9] = (Is_AQM.HCHO)&0x00ff;
	data_buf[10] = ((int)Is_AQM.Hum_RH>>8)&0x00ff;    //Is_AQM.Hum_RH
	data_buf[11] = ((int)Is_AQM.Hum_RH)&0x00ff;
	data_buf[12] = (u32PwcCapValue>>8)&0x00ff;  //pm2.5
	data_buf[13] = (u32PwcCapValue)&0x00ff;
	data_buf[14] = ((int)(Air_Tb)>>8)&0x00ff;  //air_temp
	data_buf[15] = ((int)Air_Tb)&0x00ff;
	 
//	if(strcmp((char*)dataC_buf,(char*)data_buf)==0)  //字符串比较函数strcmp
	{
		for(i=0; i<16; i++)
		{
			Uart_SendData(UARTCH1, data_buf[i]);
			delay10us(50);
		}
	  
	}
//	else {
//	  strcpy((char*)dataC_buf,(char*)data_buf);   //字符串复制
//	}
		


}

  u16 Max_PM2_5;
  u16 Max_eCO2;
	float Max_Temp_C;
	float Max_Hum_RH;
	u16 Max_TVOC;
	u16 Max_HCHO;
  float Max_Air_T;

void HomePage(void)
{
		LCD_Clear(BLACK); //背景

    POINT_COLOR = WHITE;
    BACK_COLOR  = BLACK;
	
	  Max_PM2_5 = 0;
		Max_eCO2 = 0;
		Max_Temp_C = 0;
		Max_Hum_RH = 0;
		Max_TVOC = 0;
		Max_HCHO = 0;
	  Max_Air_T = 0;
//	Gui_DrawFont_GBK32(10, 5, CYAN, BLACK, "最大值", 10);
	//   零壹贰
//	  Gui_DrawFont_GBK24(160, 0, WHITE,BLACK, "肆叁伍",10);
	  Gui_DrawFont_GBK24(160+24, 0, WHITE,BLACK, "伍",10);  //蓝牙图标
 		Gui_DrawFont_GBK24(156,0,WHITE,BLACK, "柒",10);//显示雷达图标	
    
	  POINT_COLOR = DeepGray; LCD_ShowString(20,50,16*5,16,16,"PM2.5"); POINT_COLOR = WHITE;	

    POINT_COLOR = DeepGray; LCD_ShowString(75,100,16*5,16,16,"ug/m3"); POINT_COLOR = WHITE;

	  Gui_DrawFont_GBK16(130, 50, DeepGray, BLACK,"人体温度",4);
    LCD_ShowString(130,80,210,40,40,"000");
	  Gui_DrawFont_GBK16(200, 100, DeepGray, BLACK,"°",6); //°C
	  POINT_COLOR = DeepGray; LCD_ShowString(200+7,100,16*5,16,16,"C"); POINT_COLOR = WHITE;
	 
	  Gui_DrawFont_GBK16(20, 140, DeepGray, BLACK,"温度",10);

	  Gui_DrawFont_GBK16(90, 185, DeepGray, BLACK,"°",6); //°
	  POINT_COLOR = DeepGray; LCD_ShowString(97,185,16*5,16,16,"C"); POINT_COLOR = WHITE;
		
	  Gui_DrawFont_GBK16(130, 140, DeepGray, BLACK,"湿度",10);

		POINT_COLOR = DeepGray; LCD_ShowString(197,185,16*5,16,16,"%RH"); POINT_COLOR = WHITE;
//////////////////////////////////////////////////////////////////////////////	
	  Gui_DrawFont_GBK16(20, 230, WHITE, BLACK, "有害气体", 20);
    POINT_COLOR = MAGENTA;
    LCD_ShowString(5,260,16*5,16,16,"eCO2"); 
    LCD_ShowString(75,260,16*5,16,16,"TVOC");		
		LCD_ShowString(160,260,16*5,16,16,"HCHO"); POINT_COLOR = WHITE;

		POINT_COLOR = DeepGray;LCD_ShowString(40,265,16*5,16,16,"ppm"); POINT_COLOR = WHITE;

		POINT_COLOR = DeepGray;LCD_ShowString(110,265,16*5,16,16,"ug/m3"); POINT_COLOR = WHITE;

		POINT_COLOR = DeepGray; LCD_ShowString(195,265,16*5,16,16,"ug/m3");  POINT_COLOR = WHITE;

}
 

uint16_t timer0count;    //系统计时器
float Air_Tm;           //红外温度
void Disp_Updata(void) 
{
  Air_Tm = Air_T;
  Is_AQM_D_out();
	if(u32PwcCapValue>Max_PM2_5)     //PM2.5最大值获取
	{
		Max_PM2_5 = u32PwcCapValue;
	}
	if(Is_AQM.Temp_C > Max_Temp_C) //温度最大值获取
	{
	  Max_Temp_C = Is_AQM.Temp_C;
	}
	if(Is_AQM.Hum_RH > Max_Hum_RH) //湿度最大值获取
	{
	   Max_Hum_RH = Is_AQM.Hum_RH;
	}
	if(Is_AQM.eCO2 > Max_eCO2 )  //eCO2最大值获取
	{
	   Max_eCO2 = Is_AQM.eCO2;
	}
	if(Is_AQM.TVOC > Max_TVOC)  //TVOC最大值获取
	{
	   Max_TVOC = Is_AQM.TVOC;
	}
	if(Is_AQM.HCHO > Max_HCHO)  //HCHO最大值获取
	{
	   Max_HCHO = Is_AQM.HCHO;
	}
	if(Air_Tm > Max_Air_T)  //HCHO最大值获取
	{
	   Max_Air_T = Air_Tm;
//		Max_Air_T = 42.1;
	}
	
 
	if(u32PwcCapValue<=75) {POINT_COLOR = GREEN; }  //PM2.5
	else if(u32PwcCapValue>75 && u32PwcCapValue<=250) {POINT_COLOR = YELLOW; }  //PM2.5
	else if(u32PwcCapValue>250) {POINT_COLOR = RED; }  //PM2.5
	LCD_ShowxNum(10,80,u32PwcCapValue,3,40,0X80);
	
   POINT_COLOR = WHITE;
	 LCD_ShowNum(20,170,Is_AQM.Temp_C,2,32);	//显示正数部分 
 	 LCD_ShowString(20+40,170,210,32,32,".");
	 LCD_ShowNum(20+50,170,Is_AQM.Temp_C*10,1,32);	//显示小数部分
 
	LCD_ShowNum(130,170,Is_AQM.Hum_RH,2,32);	//显示正数部分 
	LCD_ShowString(130+40,170,210,32,32,".");
	LCD_ShowNum(130+50,170,Is_AQM.Hum_RH*10,1,32);	//显示小数部分
	
	if(Is_AQM.eCO2<=1000) {POINT_COLOR = GREEN; }   
	else if(Is_AQM.eCO2>1000 && Is_AQM.eCO2<=1500) {POINT_COLOR = YELLOW; }  
	else if(Is_AQM.eCO2>1500) {POINT_COLOR = RED; }  
	LCD_ShowxNum(5,285,Is_AQM.eCO2,4,24,0X80);	////二氧化碳
	
	if(Is_AQM.TVOC<=600) {POINT_COLOR = GREEN; }   
	else if(Is_AQM.TVOC>600) {POINT_COLOR = RED; } 
	LCD_ShowxNum(75,285,Is_AQM.TVOC,4,24,0X80);	//有机气态物质 挥发性有机物
	
	if(Is_AQM.HCHO<=100) {POINT_COLOR = GREEN; }   
	else if(Is_AQM.HCHO>100) {POINT_COLOR = RED; } 
	LCD_ShowxNum(160,285,Is_AQM.HCHO,4,24,0X80);	//甲醛
	
	POINT_COLOR = WHITE;
 

			
//			if(timer0count%100>70 && (timer0count%100<80))
			{
				Blue_Data();
			}	
			
      get_ad();     delay1ms(20); get_ad();     delay1ms(20);
		  if((KEY_Right-200)<u16AdcRestult10 && u16AdcRestult10<(KEY_Right+200))
			{
				delay1ms(30);
				if((KEY_Right-200)<u16AdcRestult10 && u16AdcRestult10<(KEY_Right+200))
				{
					BEEP_On_off; 
					Disp_Max();
				}
			}	
			else if((KEY_Left-200)<u16AdcRestult10 && u16AdcRestult10<(KEY_Left+200))
			{
				delay1ms(30);
				if((KEY_Left-200)<u16AdcRestult10 && u16AdcRestult10<(KEY_Left+200))
				{ 	//-----------------关声音-----------------------
					if(voice == 0)
					{
					  voice = 1;
//					  BEEP_On_off; 
//				    Lei_Da_ON;   //PD06  雷达电源开关 1开  0关
				    Gui_DrawFont_GBK24(156,0,WHITE,BLACK, "柒",10);//显示雷达图标	
					}
					else if(voice == 1)
					{
					  voice = 0;
//					  BEEP_On_off; 
//				    Lei_Da_OFF;   //PD06  雷达电源开关 1开  0关
						LCD_ShowString(156,0,24*2,24,24,"  ");//关闭雷达图标
					}
				}
			}	
}

/********************************************************************************/

 
void Disp_Max(void) 
{
	POINT_COLOR = MAGENTA; 
//	Gui_DrawFont_GBK32(10, 5, MAGENTA, BLACK, "最大值", 10);
	LCD_ShowString(100,0,210,24,24,"MAX");
 
	LCD_ShowxNum(10,80,Max_PM2_5,3,40,0X80);  //PM2.5

	LCD_ShowNum(20,170,Max_Temp_C,2,32);	//显示正数部分 
	LCD_ShowString(20+40,170,210,32,32,".");
	LCD_ShowNum(20+50,170,Max_Temp_C*10,1,32);	//显示小数部分

	LCD_ShowNum(130,170,Max_Hum_RH,2,32);	//显示正数部分 
	LCD_ShowString(130+40,170,210,32,32,".");
	LCD_ShowNum(130+50,170,Max_Hum_RH*10,1,32);	//显示小数部分

	LCD_ShowxNum(5,285,Max_eCO2,4,24,0X80);	////二氧化碳

	LCD_ShowxNum(75,285,Max_TVOC,4,24,0X80);	//有机气态物质 挥发性有机物

	LCD_ShowxNum(160,285,Max_HCHO,4,24,0X80);	//甲醛
	
	LCD_ShowNum(130,80,Max_Air_T,2,40);	//显示正数部分 
	LCD_ShowString(173,80,210,40,40,".");
	LCD_ShowNum(178,80,Max_Air_T*10,1,40);	//显示小数部分
	
	delay1ms(3000);
	LCD_ShowString(100,0,210,24,24,"   ");
//  LCD_ShowString(10, 5, 150,32,32,"      ");
	POINT_COLOR = WHITE;
}
/********************************************************************************/

float Test_Temp(void)
{
  float Air_Temp,disp_temp;
	Get_Temp();
	Air_Temp = Actual_Read_Temp(Biao_Read_Temp());
	disp_temp = Air_Temp;
 
	if(Air_Temp<35)          //LI
	{
//		printf("LO \r\n");
		POINT_COLOR = GREEN; LCD_ShowString(130,80,210,40,40,"Low  "); POINT_COLOR = WHITE;
//		Temp_Sound(3);  // 3  体温低了 
//		BEEP_On_off;
		delay1ms(200);
    return 0; 
	}
	else if(Air_Temp>42.9)	         //HI
	{
//		printf("HI \r\n");
		POINT_COLOR = RED;   LCD_ShowString(130,80,210,40,40,"High "); POINT_COLOR = WHITE;
//		Temp_Sound(2); // 2  体温高了 
//		BEEP_On_off;
		delay1ms(200);
		return 0; 
	}
	else if((Air_Temp>=35)&&(Air_Temp<42))
	{
		if((Air_Temp>=35)&&(Air_Temp<36.1))
		{
//			Temp_Sound(1); // 1  体温高了 
//			Temp_Sound(2); //2 体温低了
			Temp_Sound(0);   // 0  体温正常 
			POINT_COLOR = GREEN;
//			BEEP_On_off;
		}
		else if((Air_Temp>=36.1)&&(Air_Temp<37.2))
		{
      Temp_Sound(0);   // 0  体温正常 
			POINT_COLOR = YELLOW;
//			BEEP_On_off;
		}
		else if((Air_Temp>=37.2)&&(Air_Temp<42))
		{
  		Temp_Sound(1); // 1  体温高了 
			POINT_COLOR = RED;
//			BEEP_On_off;
		}
		 LCD_ShowString(130,80,210,40,40,"       ");
		 LCD_ShowNum(130,80,disp_temp,2,40);	//显示正数部分 
		 LCD_ShowString(173,80,210,40,40,".");
		 LCD_ShowNum(178,80,disp_temp*10,1,40);	//显示小数部分
		 POINT_COLOR = WHITE;
		 Gui_DrawFont_GBK16(200, 100, DeepGray, BLACK,"°",10); //°C
		 POINT_COLOR = DeepGray;	LCD_ShowString(200+7,100,16*5,16,16,"C"); POINT_COLOR = WHITE;
//	   Air_Temp/=10;
	}
	return Air_Temp;
}
 

