
#ifndef __APP_GPIO_H__
#define __APP_GPIO_H__
#include "ddl.h"

#define POW_ON   Gpio_SetIO(GpioPortA, GpioPin7)  //3.0V�ܿ���
#define POW_OFF  Gpio_ClrIO(GpioPortA, GpioPin7)

#define	LED_RED_ON	 Gpio_SetIO(GpioPortC, GpioPin6);  //���
#define LED_RED_OFF   Gpio_ClrIO(GpioPortC, GpioPin6);
		
#define	LED_GREEN_ON	Gpio_SetIO(GpioPortC, GpioPin7);  //�̵�
#define	LED_GREEN_OFF	Gpio_ClrIO(GpioPortC, GpioPin7);
		
#define	IR_ON	  Gpio_SetIO(GpioPortC, GpioPin5);        //������-����
#define	IR_OFF	Gpio_ClrIO(GpioPortC, GpioPin5);      //������-����

#define	PM2_5_ON	Gpio_SetIO(GpioPortC, GpioPin4);    //PM2_5-����
#define	PM2_5_OFF	Gpio_ClrIO(GpioPortC, GpioPin4);    //PM2_5-����

#define	IR_NTC_ON	  Gpio_SetIO(GpioPortA, GpioPin6);    //���⹩��
#define	IR_NTC_OFF	Gpio_ClrIO(GpioPortA, GpioPin6);

#define	Lei_Da_ON	  Gpio_SetIO(GpioPortD, GpioPin6);    
#define	Lei_Da_OFF	Gpio_ClrIO(GpioPortD, GpioPin6);      //PD06  �״��Դ���� 1��  0��
#define	IR_Lei_ON	Gpio_GetInputIO(GpioPortB, GpioPin11)    // �״���IO��

#define BEEP_On_off  {if(voice){ Gpio_SetIO(GpioPortA, GpioPin5); delay1ms(300); Gpio_ClrIO(GpioPortA, GpioPin5);delay1ms(300);}}  //��������
 
extern char voice;

void GPIO_PortInit(void);
void Temp_Sound(char n);
	
void Recovery_GPIO_State(void);
void Save_GPIO_State(void);  
void _LowPowerModeGpioSet(void);
#endif /* __UART_H__ */


