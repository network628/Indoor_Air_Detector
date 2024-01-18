
#ifndef __APP_H__
#define __APP_H__
#include "stdint.h" 
//#include "app_lcd.h"

 
extern uint8_t Sleep_flag;
//extern volatile stc_lcd_display_cfg_t gstcLcdDisplayCfg;
extern uint32_t	NTC_Addr;
extern uint16_t timer0count;    //系统计时器

//extern uint16_t KEY_Up  ;   //上
//extern uint16_t KEY_Down ;  //下
extern uint16_t KEY_Left ;  //左
extern uint16_t KEY_Right ;  //右
 
 
void LPM_Init(void);
 
void FlashInt(void);
void ADjust_NTC(void);
void Auto_ADjust37_NTC(void);     //37温度补偿
void Wakeup_Init(void);
 
char Read_Flash_Data(uint32_t	Addr);
float Read_ATCV(uint32_t	Addr);
void Write_ATCV(uint32_t	Addr, float F_data);

void deep_sleep(void);
void Idle_shutdown(void);   //空闲关机
void Low_voltage_alarm_shutdown(void);
void bat_shan(void);

#endif /* __UART_H__ */


