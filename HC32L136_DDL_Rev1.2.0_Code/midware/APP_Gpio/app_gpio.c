/******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "gpio.h"
#include "sysctrl.h"
#include "app_gpio.h"
#include "board_skhc32l13x.h"
#include "app_timer.h"

void GPIO_PortInit(void)
{
 		stc_gpio_config_t stcGpioCfg;
	  stc_gpio_config_t pstcGpioCfg;
    
    ///< ��GPIO����ʱ���ſ�
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    ///< �˿ڷ�������->���
    pstcGpioCfg.enDir = GpioDirOut;
    ///< �˿�������������->����������
    pstcGpioCfg.enDrv = GpioDrvH;
    ///< �˿�����������->��������
    pstcGpioCfg.enPuPd = GpioNoPuPd;
    ///< �˿ڿ�©�������->��©����ر�
    pstcGpioCfg.enOD = GpioOdDisable;
    ///< �˿�����/���ֵ�Ĵ������߿���ģʽ����->AHB
    pstcGpioCfg.enCtrlMode = GpioAHB;
    
    Gpio_Init(GpioPortA, GpioPin6, &pstcGpioCfg);  //���⹩��
    Gpio_Init(GpioPortA, GpioPin7, &pstcGpioCfg);  //POW  �ܿ���
    Gpio_Init(GpioPortA, GpioPin5, &pstcGpioCfg);  //BEEP  ������
	  Gpio_ClrIO(GpioPortA, GpioPin5);

		
		Gpio_Init(GpioPortD, GpioPin7, &pstcGpioCfg);  //LCD����
    Gpio_ClrIO(GpioPortD, GpioPin7);

		Gpio_Init(GpioPortC, GpioPin6, &pstcGpioCfg);  //���
		Gpio_Init(GpioPortC, GpioPin7, &pstcGpioCfg);  //�̵�
    Gpio_Init(GpioPortC, GpioPin5, &pstcGpioCfg);  //������-����
		Gpio_Init(GpioPortC, GpioPin4, &pstcGpioCfg);  //PM2.5-����
		Gpio_Init(GpioPortD, GpioPin6, &pstcGpioCfg);  //PD06  �״��Դ���� 1��  0��
		

		Gpio_Init(GpioPortB, GpioPin14, &pstcGpioCfg); //TG1 ����
		Gpio_Init(GpioPortB, GpioPin15, &pstcGpioCfg); //TG2 ��λ
		Gpio_ClrIO(GpioPortB, GpioPin14);   //TG1 ����
		Gpio_ClrIO(GpioPortB, GpioPin15);   //TG2 ��λ	


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
		
//	  Gpio_ClearIrq(GpioPortD, GpioPin2);
//    ///< �򿪲�����PD04Ϊ�½����ж�
//    Gpio_EnableIrq(GpioPortD, GpioPin2, GpioIrqFalling);
//    ///< ʹ�ܶ˿�PORTDϵͳ�ж�
//    EnableNvic(PORTD_IRQn, IrqLevel3, TRUE);
		
		///< �˿�����������->����
    stcGpioCfg.enPuPd = GpioPd;
		Gpio_Init(GpioPortB, GpioPin11, &stcGpioCfg);   // �״���IO��
}
 
 
void Temp_Sound(char n)
{
	// 1  ��������  
	// 2  ���¸��� 
	// 3  ���µ��� 
	// 4  bodytempture nomo
	// 5  bodytempture hi 
	// 6  bodytempture low
	if(voice && timer0count>30)
	{
		Gpio_SetIO(GpioPortB, GpioPin15);  //TG2 ��λ
		delay10us(30);  //100uS
		Gpio_ClrIO(GpioPortB, GpioPin15);
		delay1ms(5);
		
		while(n--)  
		{
			Gpio_SetIO(GpioPortB, GpioPin14);  //TG1 ����
			delay10us(11);  //100uS
			Gpio_ClrIO(GpioPortB, GpioPin14);  //TG1 ����
			delay10us(11);  //100uS
		}
	//	Gpio_ClrIO(GpioPortB, GpioPin14);  //TG1 ����
		Gpio_SetIO(GpioPortB, GpioPin14);
		delay1ms(2000);
	}
		Gpio_ClrIO(GpioPortB, GpioPin14);   //TG1 ����
		Gpio_ClrIO(GpioPortB, GpioPin15);   //TG2 ��λ
}
uint32_t gpio_reg[24] ;

void Save_GPIO_State(void)   //����GPIO״̬
{
	
//		Gpio_ClrIO(GpioPortB, GpioPin14);   //TG1 ����
//		Gpio_ClrIO(GpioPortB, GpioPin15);   //TG2 ��λ
//M0P_GPIO->PBADS = 0x0000;
//M0P_GPIO->PBPU = 0x0000;
//M0P_GPIO->PBPD = 0xFFFF;
//M0P_GPIO->PBOD = 0x0000;
//M0P_GPIO->PBOUT = 0x0000;
//M0P_GPIO->PBDIR = 0x3FFF;

Gpio_ClrIO(GpioPortD, GpioPin7);
	
	
		gpio_reg[0] = M0P_GPIO->PAADS; 
		gpio_reg[1] = M0P_GPIO->PAPU ;
		gpio_reg[2] = M0P_GPIO->PAPD ;
		gpio_reg[3] = M0P_GPIO->PAOD ;
		gpio_reg[4] = M0P_GPIO->PAOUT; 
		gpio_reg[5] = M0P_GPIO->PADIR; 

		gpio_reg[6] = M0P_GPIO->PBADS; 
		gpio_reg[7] = M0P_GPIO->PBPU ;
		gpio_reg[8] = M0P_GPIO->PBPD ;
		gpio_reg[9] = M0P_GPIO->PBOD ;
		gpio_reg[10] = M0P_GPIO->PBOUT;
		gpio_reg[11] = M0P_GPIO->PBDIR;

		gpio_reg[12] = M0P_GPIO->PCADS;
		gpio_reg[13] = M0P_GPIO->PCPU ;
		gpio_reg[14] = M0P_GPIO->PCPD ;
		gpio_reg[15] = M0P_GPIO->PCOD ;
		gpio_reg[16] = M0P_GPIO->PCOUT;
		gpio_reg[17] = M0P_GPIO->PCDIR;

		gpio_reg[18] = M0P_GPIO->PDADS;
		gpio_reg[19] = M0P_GPIO->PDPU ;
		gpio_reg[20] = M0P_GPIO->PDPD ;
		gpio_reg[21] = M0P_GPIO->PDOD ;
		gpio_reg[22] = M0P_GPIO->PDOUT;
		gpio_reg[23] = M0P_GPIO->PDDIR;



}

void Recovery_GPIO_State(void)    //�ָ�GPIO״̬
{
// ���� ......
		char i;

		i=0;
		M0P_GPIO->PAADS = gpio_reg[i++];
		M0P_GPIO->PAPU  = gpio_reg[i++];
		M0P_GPIO->PAPD  = gpio_reg[i++];
		M0P_GPIO->PAOD  = gpio_reg[i++];
		M0P_GPIO->PAOUT = gpio_reg[i++];
		M0P_GPIO->PADIR = gpio_reg[i++];
					 
		M0P_GPIO->PBADS = gpio_reg[i++];
		M0P_GPIO->PBPU  = gpio_reg[i++];
		M0P_GPIO->PBPD  = gpio_reg[i++];
		M0P_GPIO->PBOD  = gpio_reg[i++];
		M0P_GPIO->PBOUT = gpio_reg[i++];
		M0P_GPIO->PBDIR = gpio_reg[i++];
					 
		M0P_GPIO->PCADS = gpio_reg[i++];
		M0P_GPIO->PCPU  = gpio_reg[i++];
		M0P_GPIO->PCPD  = gpio_reg[i++];
		M0P_GPIO->PCOD  = gpio_reg[i++];
		M0P_GPIO->PCOUT = gpio_reg[i++];
		M0P_GPIO->PCDIR = gpio_reg[i++];
					 
		M0P_GPIO->PDADS = gpio_reg[i++];
		M0P_GPIO->PDPU  = gpio_reg[i++];
		M0P_GPIO->PDPD  = gpio_reg[i++];
		M0P_GPIO->PDOD  = gpio_reg[i++];
		M0P_GPIO->PDOUT = gpio_reg[i++];
		M0P_GPIO->PDDIR = gpio_reg[i++];
}


void _LowPowerModeGpioSet(void)
{
 
		M0P_GPIO->PAADS = 0x0000;
		M0P_GPIO->PAPU = 0x6000;
		M0P_GPIO->PAPD = 0x9FFF;
		M0P_GPIO->PAOD = 0x0000;
		M0P_GPIO->PAOUT = 0x0000;
		M0P_GPIO->PADIR = 0xFFFF;

		M0P_GPIO->PBADS = 0x0000;
		M0P_GPIO->PBPU = 0x0000;
		M0P_GPIO->PBPD = 0xFFFF;
		M0P_GPIO->PBOD = 0x0000;
		M0P_GPIO->PBOUT = 0x0000;
		M0P_GPIO->PBDIR = 0xFFFF;




		M0P_GPIO->PCADS = 0x0000;
		M0P_GPIO->PCPU = 0x0008;
		M0P_GPIO->PCPD = 0xFFF7;
		M0P_GPIO->PCOD = 0x0000;
		M0P_GPIO->PCOUT = 0x0008;
		M0P_GPIO->PCDIR = 0xFFFF;


		M0P_GPIO->PDADS = 0x0000;
		M0P_GPIO->PDPU = 0x0004;
		M0P_GPIO->PDPD = 0x00E3;
		M0P_GPIO->PDOD = 0x0000;
		M0P_GPIO->PDOUT = 0x0000;
		M0P_GPIO->PDDIR = 0x00FF;

}


