
#ifndef __APP_AD_H__
#define __APP_AD_H__

#include "stdint.h" 

extern uint16_t u16AdcRestult0;   //PA00 ad_air
extern uint16_t u16AdcRestult1;   //PA01 ad_ntc
extern uint16_t u16AdcRestult13;  //PC03 BAT_DEL µÁ≥ÿµÁ—πºÏ≤‚
extern uint16_t u16AdcRestult10;  //PC00
extern uint16_t u16AdcRestult12;  //PC02	∫ÏÕ‚≤‚æ‡-AD


uint8_t AD_Init(void);
void get_ad(void);
#endif /* __UART_H__ */


