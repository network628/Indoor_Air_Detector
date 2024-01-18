/******************************************************************************
 * Include files
 ******************************************************************************/
#include "app_ad.h"
#include "ddl.h"
#include "gpio.h"
#include "sysctrl.h"

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "adc.h"
#include "gpio.h"
/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
#include "app_uart.h"
#include "stdio.h"
 

// uint32_t u16AdcRestultAcc;
/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/
static stc_adc_irq_t stcAdcIrqFlag;

 
 /*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

void AdcContIrqCallback(void)
{    
    Adc_GetSqrResult(&u16AdcRestult0, CH0MUX);  //PA00 ad_air
    Adc_GetSqrResult(&u16AdcRestult1, CH1MUX);  //PA01 ad_ntc
    Adc_GetSqrResult(&u16AdcRestult13,CH2MUX);  //PC03 BAT_DEL 电池电压检测
	  Adc_GetSqrResult(&u16AdcRestult10,CH3MUX);  //PC00	KEY
    Adc_GetSqrResult(&u16AdcRestult12,CH4MUX);  //PC02	红外测距-AD
    stcAdcIrqFlag.bAdcSQRIrq = TRUE;
} 

 
/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/
uint8_t AD_Init(void)
{
	  
    uint8_t                    u8AdcScanCnt;
    stc_adc_cfg_t              stcAdcCfg;
    stc_adc_irq_t              stcAdcIrq;
    stc_adc_irq_calbakfn_pt_t  stcAdcIrqCalbaks;
    stc_gpio_config_t          stcAdcAN0Port;
    stc_gpio_config_t          stcAdcAN2Port;
    stc_gpio_config_t          stcAdcAN13Port;
    stc_gpio_config_t          stcAdcAN10Port;
    stc_gpio_config_t          stcAdcAN12Port;
	
    DDL_ZERO_STRUCT(stcAdcCfg);
    DDL_ZERO_STRUCT(stcAdcIrq);
    DDL_ZERO_STRUCT(stcAdcIrqCalbaks);
    DDL_ZERO_STRUCT(stcAdcIrqFlag);
    DDL_ZERO_STRUCT(stcAdcAN0Port);
    DDL_ZERO_STRUCT(stcAdcAN2Port);
    DDL_ZERO_STRUCT(stcAdcAN13Port);
    DDL_ZERO_STRUCT(stcAdcAN10Port);
    DDL_ZERO_STRUCT(stcAdcAN12Port);    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
    
    Gpio_SetAnalogMode(GpioPortA, GpioPin0);        //PA00 (AIN0)
    Gpio_SetAnalogMode(GpioPortA, GpioPin1);        //PA01 (AIN2)    
    Gpio_SetAnalogMode(GpioPortC, GpioPin3);        //PC03 (AIN13)    
    Gpio_SetAnalogMode(GpioPortC, GpioPin0);        //PC00 (AIN10)
		Gpio_SetAnalogMode(GpioPortC, GpioPin2);        //PC02 (AIN12)
    if (Ok != Sysctrl_SetPeripheralGate(SysctrlPeripheralAdcBgr, TRUE))
    {
        return 1;
    }    
    
    //ADC??
    Adc_Enable();
    M0P_BGR->CR_f.BGR_EN = 0x1u;                 //BGR????
    M0P_BGR->CR_f.TS_EN = 0x0u;
    delay100us(1);
    
    stcAdcCfg.enAdcOpMode = AdcSCanMode;         //????
    stcAdcCfg.enAdcClkDiv = AdcClkSysTDiv1;
    stcAdcCfg.enAdcSampTimeSel = AdcSampTime8Clk;
    stcAdcCfg.enAdcRefVolSel   = RefVolSelAVDD;  /*!<AVDD*/
    stcAdcCfg.bAdcInBufEn      = FALSE;
    
    Adc_Init(&stcAdcCfg);
    
    Adc_ConfigSqrChannel(CH0MUX, AdcExInputCH0);
    Adc_ConfigSqrChannel(CH1MUX, AdcExInputCH1);
    Adc_ConfigSqrChannel(CH2MUX, AdcExInputCH13); //PC03  
    Adc_ConfigSqrChannel(CH3MUX, AdcExInputCH10); //PC00  
		Adc_ConfigSqrChannel(CH4MUX, AdcExInputCH12);/*!<使用通道2输入PC02*/
		
    EnableNvic(ADC_IRQn, IrqLevel3, TRUE);
    
    Adc_EnableIrq();
    
    stcAdcIrq.bAdcSQRIrq = TRUE;
    stcAdcIrqCalbaks.pfnAdcSQRIrq = AdcContIrqCallback;
    
    Adc_ConfigIrq(&stcAdcIrq, &stcAdcIrqCalbaks);
    
    u8AdcScanCnt = 5;
    
    Adc_ConfigSqrMode(&stcAdcCfg, u8AdcScanCnt, TRUE);//??ADC????????
    
    Adc_SQR_Start();
    

    return 0;
}
 
void get_ad(void)
{
	while(FALSE == stcAdcIrqFlag.bAdcSQRIrq);
	Adc_ClrAccResult();
	stcAdcIrqFlag.bAdcSQRIrq = FALSE;
	Adc_SQR_Start();    
}


 



