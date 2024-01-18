#include "PM1006.h"
#include "base_types.h"
#include "timer3.h"
#include "bt.h"
#include "app.h"
#include "gpio.h"

uint16_t u16TIM3_CntValue;

/*******************************************************************************
 * BT3�жϷ�����
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
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);   //�˿�����ʱ��ʹ��
    Sysctrl_SetPeripheralGate(SysctrlPeripheralTim3, TRUE);   //Timer3 ����ʱ��ʹ��
    //PA08����ΪTIM3_CH0A
    stcTIM3A0Port.enDir  = GpioDirIn;
    Gpio_Init(GpioPortA, GpioPin8, &stcTIM3A0Port);
    Gpio_SetAfMode(GpioPortA,GpioPin8,GpioAf2);

    stcTim3BaseCfg.enWorkMode = Tim3WorkMode1;                //��ʱ��ģʽ
    stcTim3BaseCfg.enCT       = Tim3Timer;                    //��ʱ�����ܣ�����ʱ��Ϊ�ڲ�PCLK
    stcTim3BaseCfg.enPRS      = Tim3PCLKDiv1;                 //PCLK
    stcTim3BaseCfg.enOneShot  = Tim3PwcCycleDetect;         //PWCѭ�����

    stcTim3BaseCfg.pfnTim3Cb  = Tim3Int;                      //�жϺ������
    
    Tim3_Mode1_Init(&stcTim3BaseCfg);                         //TIM3 ��ģʽ1���ܳ�ʼ��
    
    stcTim3PwcInCfg.enTsSel  = Tim3Ts6IAFP;                   //PWC����ѡ�� CHA
    stcTim3PwcInCfg.enIA0Sel = Tim3IA0Input;                  //CHAѡ��IA0
    stcTim3PwcInCfg.enFltIA0 = Tim3FltPCLKDiv16Cnt3;          //PCLK/16 3��������Ч

    Tim3_M1_Input_Config(&stcTim3PwcInCfg);                   //PWC��������
    
    Tim3_M1_PWC_Edge_Sel(Tim3PwcFallToRise);                  //�����ص������ز���
    
    u16CntValue = 0;
    Tim3_M1_Cnt16Set(u16CntValue);                            //���ü�����ֵ  
    
    Tim3_ClearIntFlag(Tim3UevIrq);                            //��Uev�жϱ�־
    Tim3_ClearIntFlag(Tim3CA0Irq);                            //�岶׽�жϱ�־
    
    EnableNvic(TIM3_IRQn, IrqLevel3, TRUE);                   //TIM3�ж�ʹ��
    
    Tim3_Mode1_EnableIrq(Tim3UevIrq);                         //ʹ��TIM3����ж�
    Tim3_Mode1_EnableIrq(Tim3CA0Irq);                         //ʹ��TIM3�����ж�
    
    Tim3_M1_Run();    //TIM3 ����
}  
 



