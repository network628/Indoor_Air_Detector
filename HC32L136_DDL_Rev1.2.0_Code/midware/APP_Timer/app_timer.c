#include "app_timer.h"
#include "base_types.h"
#include "app_ad.h"
#include "bt.h"
#include "app.h"
#include "tft_lcd.h"

/*******************************************************************************
 * BT1??????
 ******************************************************************************/
void Tim0Int(void)
{
 
    if(TRUE == Bt_GetIntFlag(TIM0, BtUevIrq))
    {
        timer0count++;
//			LCD_ShowxNum(120,25,timer0count,4,24,0X80);	
        if(timer0count>65530)
				{
				   timer0count = 0;
				}
        Bt_ClearIntFlag(TIM0,BtUevIrq);
		

//			 if(timer0count >50)
//			 {
//				 timer0count = 0;
//			   get_ad();
//			 printf("u16AdcRestult10 = %d  \r\n",u16AdcRestult10 );
////				 delay1ms(1000);
		//	 }
				
    }
}

void Tim0Init(void)
{
    uint16_t                  u16ArrValue;
    uint16_t                  u16CntValue;
    stc_bt_mode0_config_t     stcBtBaseCfg;
 

    
    DDL_ZERO_STRUCT(stcBtBaseCfg);
 
    
 
    Sysctrl_SetPeripheralGate(SysctrlPeripheralBTim, TRUE); //Base Timer??????
    
 
    
    stcBtBaseCfg.enWorkMode = BtWorkMode0;                  //?????
    stcBtBaseCfg.enCT       = BtTimer;                      //?????,???????PCLK
    stcBtBaseCfg.enPRS      = BtPCLKDiv64;                  //PCLK/64
    stcBtBaseCfg.enCntMode  = Bt16bitArrMode;               //????16????/???
    stcBtBaseCfg.bEnTog     = FALSE;
    stcBtBaseCfg.bEnGate    = FALSE;
    stcBtBaseCfg.enGateP    = BtGatePositive;
    
    stcBtBaseCfg.pfnTim0Cb  = Tim0Int;                      //??????
    
    Bt_Mode0_Init(TIM0, &stcBtBaseCfg);                     //TIM0 ???0?????
    
    u16ArrValue = 0xFFFF - SystemCoreClock / 16 / 100;  //100us
    
    Bt_M0_ARRSet(TIM0, u16ArrValue);                        //?????(?? = 0x10000 - ARR)
    
    u16CntValue = 0xFFFF - SystemCoreClock / 16 / 100;
    
    Bt_M0_Cnt16Set(TIM0, u16CntValue);                      //??????
    
    Bt_ClearIntFlag(TIM0,BtUevIrq);                         //?????   
    Bt_Mode0_EnableIrq(TIM0);                               //??TIM0??(??0???????)
    EnableNvic(TIM0_IRQn, IrqLevel3, TRUE);                 //TIM0????
    
    Bt_M0_Run(TIM0);

}

 

