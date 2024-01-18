#include "PM1006.h"
#include "base_types.h"
#include "timer3.h"
#include "bt.h"
#include "app.h"
#include "gpio.h"

uint16_t u16TIM3_CntValue;

/*******************************************************************************
 * BT3中断服务函数
 ******************************************************************************/
void Tim3Int(void)
{
    static uint16_t u16TIM3_OverFlowCnt;
    
    static uint16_t u16TIM3_CapValue;
     
    if(TRUE == Tim3_GetIntFlag(Tim3UevIrq))
    {
        u16TIM3_OverFlowCnt++;
        Tim3_ClearIntFlag(Tim3UevIrq);
    }

    if(TRUE == Tim3_GetIntFlag(Tim3CA0Irq))
    {
        u16TIM3_CapValue = Tim3_M1_PWC_CapValueGet();
        u32PwcCapValue = u16TIM3_OverFlowCnt*0x10000 + u16TIM3_CapValue;
        u32PwcCapValue/=100000;
//			  printf("-->u32PwcCapValue=%05d  \r\n",u32PwcCapValue);
        u16TIM3_OverFlowCnt = 0;
        Tim3_ClearIntFlag(Tim3CA0Irq);
    }
}

void Tim3Init(void)
{
    uint16_t                     u16CntValue;
    stc_tim3_mode1_config_t      stcTim3BaseCfg;
    stc_tim3_pwc_input_config_t  stcTim3PwcInCfg;
    stc_gpio_config_t            stcTIM3A0Port;
    DDL_ZERO_STRUCT(stcTim3BaseCfg);
    DDL_ZERO_STRUCT(stcTIM3A0Port);
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);   //端口外设时钟使能
    Sysctrl_SetPeripheralGate(SysctrlPeripheralTim3, TRUE);   //Timer3 外设时钟使能
    //PA08设置为TIM3_CH0A
    stcTIM3A0Port.enDir  = GpioDirIn;
    Gpio_Init(GpioPortA, GpioPin8, &stcTIM3A0Port);
    Gpio_SetAfMode(GpioPortA,GpioPin8,GpioAf2);

    stcTim3BaseCfg.enWorkMode = Tim3WorkMode1;                //定时器模式
    stcTim3BaseCfg.enCT       = Tim3Timer;                    //定时器功能，计数时钟为内部PCLK
    stcTim3BaseCfg.enPRS      = Tim3PCLKDiv1;                 //PCLK
    stcTim3BaseCfg.enOneShot  = Tim3PwcCycleDetect;         //PWC循环检测

    stcTim3BaseCfg.pfnTim3Cb  = Tim3Int;                      //中断函数入口
    
    Tim3_Mode1_Init(&stcTim3BaseCfg);                         //TIM3 的模式1功能初始化
    
    stcTim3PwcInCfg.enTsSel  = Tim3Ts6IAFP;                   //PWC输入选择 CHA
    stcTim3PwcInCfg.enIA0Sel = Tim3IA0Input;                  //CHA选择IA0
    stcTim3PwcInCfg.enFltIA0 = Tim3FltPCLKDiv16Cnt3;          //PCLK/16 3个连续有效

    Tim3_M1_Input_Config(&stcTim3PwcInCfg);                   //PWC输入设置
    
    Tim3_M1_PWC_Edge_Sel(Tim3PwcFallToRise);                  //上升沿到上升沿捕获
    
    u16CntValue = 0;
    Tim3_M1_Cnt16Set(u16CntValue);                            //设置计数初值  
    
    Tim3_ClearIntFlag(Tim3UevIrq);                            //清Uev中断标志
    Tim3_ClearIntFlag(Tim3CA0Irq);                            //清捕捉中断标志
    
    EnableNvic(TIM3_IRQn, IrqLevel3, TRUE);                   //TIM3中断使能
    
    Tim3_Mode1_EnableIrq(Tim3UevIrq);                         //使能TIM3溢出中断
    Tim3_Mode1_EnableIrq(Tim3CA0Irq);                         //使能TIM3捕获中断
    
    Tim3_M1_Run();    //TIM3 运行
}  
 



