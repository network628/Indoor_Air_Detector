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
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    ///< 端口方向配置->输出
    pstcGpioCfg.enDir = GpioDirOut;
    ///< 端口驱动能力配置->高驱动能力
    pstcGpioCfg.enDrv = GpioDrvH;
    ///< 端口上下拉配置->无上下拉
    pstcGpioCfg.enPuPd = GpioNoPuPd;
    ///< 端口开漏输出配置->开漏输出关闭
    pstcGpioCfg.enOD = GpioOdDisable;
    ///< 端口输入/输出值寄存器总线控制模式配置->AHB
    pstcGpioCfg.enCtrlMode = GpioAHB;
    
    Gpio_Init(GpioPortA, GpioPin6, &pstcGpioCfg);  //红外供电
    Gpio_Init(GpioPortA, GpioPin7, &pstcGpioCfg);  //POW  总开关
    Gpio_Init(GpioPortA, GpioPin5, &pstcGpioCfg);  //BEEP  蜂鸣器
	  Gpio_ClrIO(GpioPortA, GpioPin5);

		
		Gpio_Init(GpioPortD, GpioPin7, &pstcGpioCfg);  //LCD背光
    Gpio_ClrIO(GpioPortD, GpioPin7);

		Gpio_Init(GpioPortC, GpioPin6, &pstcGpioCfg);  //红灯
		Gpio_Init(GpioPortC, GpioPin7, &pstcGpioCfg);  //绿灯
    Gpio_Init(GpioPortC, GpioPin5, &pstcGpioCfg);  //红外测距-开关
		Gpio_Init(GpioPortC, GpioPin4, &pstcGpioCfg);  //PM2.5-开关
		Gpio_Init(GpioPortD, GpioPin6, &pstcGpioCfg);  //PD06  雷达电源开关 1开  0关
		

		Gpio_Init(GpioPortB, GpioPin14, &pstcGpioCfg); //TG1 脉冲
		Gpio_Init(GpioPortB, GpioPin15, &pstcGpioCfg); //TG2 复位
		Gpio_ClrIO(GpioPortB, GpioPin14);   //TG1 脉冲
		Gpio_ClrIO(GpioPortB, GpioPin15);   //TG2 复位	


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
		
//	  Gpio_ClearIrq(GpioPortD, GpioPin2);
//    ///< 打开并配置PD04为下降沿中断
//    Gpio_EnableIrq(GpioPortD, GpioPin2, GpioIrqFalling);
//    ///< 使能端口PORTD系统中断
//    EnableNvic(PORTD_IRQn, IrqLevel3, TRUE);
		
		///< 端口上下拉配置->下拉
    stcGpioCfg.enPuPd = GpioPd;
		Gpio_Init(GpioPortB, GpioPin11, &stcGpioCfg);   // 雷达检测IO口
}
 
 
void Temp_Sound(char n)
{
	// 1  体温正常  
	// 2  体温高了 
	// 3  体温低了 
	// 4  bodytempture nomo
	// 5  bodytempture hi 
	// 6  bodytempture low
	if(voice && timer0count>30)
	{
		Gpio_SetIO(GpioPortB, GpioPin15);  //TG2 复位
		delay10us(30);  //100uS
		Gpio_ClrIO(GpioPortB, GpioPin15);
		delay1ms(5);
		
		while(n--)  
		{
			Gpio_SetIO(GpioPortB, GpioPin14);  //TG1 脉冲
			delay10us(11);  //100uS
			Gpio_ClrIO(GpioPortB, GpioPin14);  //TG1 脉冲
			delay10us(11);  //100uS
		}
	//	Gpio_ClrIO(GpioPortB, GpioPin14);  //TG1 脉冲
		Gpio_SetIO(GpioPortB, GpioPin14);
		delay1ms(2000);
	}
		Gpio_ClrIO(GpioPortB, GpioPin14);   //TG1 脉冲
		Gpio_ClrIO(GpioPortB, GpioPin15);   //TG2 复位
}
uint32_t gpio_reg[24] ;

void Save_GPIO_State(void)   //保存GPIO状态
{
	
//		Gpio_ClrIO(GpioPortB, GpioPin14);   //TG1 脉冲
//		Gpio_ClrIO(GpioPortB, GpioPin15);   //TG2 复位
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

void Recovery_GPIO_State(void)    //恢复GPIO状态
{
// 唤醒 ......
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


