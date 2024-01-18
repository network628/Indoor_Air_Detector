 
/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "ddl.h"
#include "uart.h"
#include "app_uart.h"
#include "interrupts_hc32l136.h"
__WEAKDEF void Gpio_IRQHandler(uint8_t u8Param);
__WEAKDEF void Dma_IRQHandler(uint8_t u8Param);
__WEAKDEF void Uart_IRQHandler(uint8_t u8Param);
__WEAKDEF void LpUart_IRQHandler(uint8_t u8Param);
__WEAKDEF void Spi_IRQHandler(uint8_t u8Param);
__WEAKDEF void I2c_IRQHandler(uint8_t u8Param);
__WEAKDEF void Tim_IRQHandler(uint8_t u8Param);
__WEAKDEF void Tim3_IRQHandler(uint8_t u8Param);
__WEAKDEF void Adt_IRQHandler(uint8_t u8Param);
__WEAKDEF void LpTim_IRQHandler(uint8_t u8Param);
__WEAKDEF void Pca_IRQHandler(uint8_t u8Param);
__WEAKDEF void Wdt_IRQHandler(uint8_t u8Param);
__WEAKDEF void Vc_IRQHandler(uint8_t u8Param);
__WEAKDEF void Rtc_IRQHandler(uint8_t u8Param);
__WEAKDEF void Adc_IRQHandler(uint8_t u8Param);
__WEAKDEF void Pcnt_IRQHandler(uint8_t u8Param);
__WEAKDEF void Lvd_IRQHandler(uint8_t u8Param);
__WEAKDEF void Lcd_IRQHandler(uint8_t u8Param);
__WEAKDEF void EfRam_IRQHandler(uint8_t u8Param);
__WEAKDEF void ClkTrim_IRQHandler(uint8_t u8Param);

/**
 *******************************************************************************
 ** \brief NVIC 中断使能
 **
 ** \param [in]  enIrq          中断号枚举类型
 ** \param [in]  enLevel        中断优先级枚举类型
 ** \param [in]  bEn            中断开关
 ** \retval    Ok       设置成功
 **            其他值   设置失败
 ******************************************************************************/
void EnableNvic(IRQn_Type enIrq, en_irq_level_t enLevel, boolean_t bEn)
{
    NVIC_ClearPendingIRQ(enIrq);
    NVIC_SetPriority(enIrq, enLevel);
    if (TRUE == bEn)
    {
        NVIC_EnableIRQ(enIrq);
    }
    else
    {
        NVIC_DisableIRQ(enIrq);
    }
}

/**
 *******************************************************************************
 ** \brief NVIC hardware fault 中断实现
 **        用于单步调试功能
 **
 ** \retval
 ******************************************************************************/
void HardFault_Handler(void)
{
    volatile int a = 0;

    while( 0 == a)
    {
        ;
    }
}

/**
 *******************************************************************************
 ** \brief GPIO PortA 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void PORTA_IRQHandler(void)
{
    Gpio_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief GPIO PortB 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void PORTB_IRQHandler(void)
{
    Gpio_IRQHandler(1);
}

/**
 *******************************************************************************
 ** \brief GPIO PortC 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void PORTC_IRQHandler(void)
{
    Gpio_IRQHandler(2);
}

/**
 *******************************************************************************
 ** \brief GPIO PortD 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void PORTD_IRQHandler(void)
{
    Gpio_IRQHandler(3);
}

/**
 *******************************************************************************
 ** \brief DMAC  中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void DMAC_IRQHandler(void)
{
    Dma_IRQHandler(0);
}


/**
 *******************************************************************************
 ** \brief UART0 串口0 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void UART0_IRQHandler(void)
{
    Uart_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief UART1 串口1 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
//uint8_t u8RxCnt;
void UART1_IRQHandler(void)
{
    Uart_IRQHandler(1);
 
}

/**
 *******************************************************************************
 ** \brief LPUART0 低功耗串口0 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void LPUART0_IRQHandler(void)
{
    LpUart_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief LPUART1 低功耗串口1 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void LPUART1_IRQHandler(void)
{
    LpUart_IRQHandler(1);
}

/**
 *******************************************************************************
 ** \brief SPI0 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void SPI0_IRQHandler(void)
{
    Spi_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief SPI1 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void SPI1_IRQHandler(void)
{
    Spi_IRQHandler(1);
}

/**
 *******************************************************************************
 ** \brief I2C0 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void I2C0_IRQHandler(void)
{
    I2c_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief I2C1 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void I2C1_IRQHandler(void)
{
    I2c_IRQHandler(1);
}

/**
 *******************************************************************************
 ** \brief TIM0 基础时钟0 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void TIM0_IRQHandler(void)
{
    Tim_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief TIM1 基础时钟1 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void TIM1_IRQHandler(void)
{
    Tim_IRQHandler(1);
}

/**
 *******************************************************************************
 ** \brief TIM2 基础时钟2 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void TIM2_IRQHandler(void)
{
    Tim_IRQHandler(2);
}

/**
 *******************************************************************************
 ** \brief TIM3 基础时钟3 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void TIM3_IRQHandler(void)
{
    Tim3_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief LPTIM 低功耗时钟 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void LPTIM_IRQHandler(void)
{
    LpTim_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief TIM4 高级时钟4 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void TIM4_IRQHandler(void)
{
    Adt_IRQHandler(4);
}

/**
 *******************************************************************************
 ** \brief TIM5 高级时钟5 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void TIM5_IRQHandler(void)
{
    Adt_IRQHandler(5);
}

/**
 *******************************************************************************
 ** \brief TIM6 高级时钟6 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void TIM6_IRQHandler(void)
{
    Adt_IRQHandler(6);
}

/**
 *******************************************************************************
 ** \brief PCA 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void PCA_IRQHandler(void)
{
    Pca_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief WDT 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void WDT_IRQHandler(void)
{
    Wdt_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief RTC 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void RTC_IRQHandler(void)
{
    Rtc_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief ADC 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void ADC_IRQHandler(void)
{
    Adc_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief PCNT 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void PCNT_IRQHandler(void)
{
    Pcnt_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief 电压比较0 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void VC0_IRQHandler(void)
{
    Vc_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief 电压比较1 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void VC1_IRQHandler(void)
{
    Vc_IRQHandler(1);
}

/**
 *******************************************************************************
 ** \brief 低电压检测 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void LVD_IRQHandler(void)
{
    Lvd_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief LCD 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void LCD_IRQHandler(void)
{
    Lcd_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief RAM 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void EF_RAM_IRQHandler(void)
{
    EfRam_IRQHandler(0);
}

/**
 *******************************************************************************
 ** \brief 时钟校准 中断处理函数
 ** 
 ** \retval
 ******************************************************************************/
void CLKTRIM_IRQHandler(void)
{
    ClkTrim_IRQHandler(0);
}



/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
