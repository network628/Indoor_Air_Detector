#ifndef __SHT2X_H
#define __SHT2X_H			 

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "gpio.h"

#define CHECKSUM_ERROR 0x04


typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))


#define EEPROM_I2C_SCL_PIN      					 	GpioPin8   
#define EEPROM_I2C_SCL_GPIO_PORT   	       	GpioPortB
//#define EEPROM_I2C_SCL_GPIO_RCC      			  RCC_APB2Periph_GPIOB  
 
#define EEPROM_I2C_SDA_PIN      					 	GpioPin9   
#define EEPROM_I2C_SDA_GPIO_PORT   	       	GpioPortB  
//#define EEPROM_I2C_SDA_GPIO_RCC      			  RCC_APB2Periph_GPIOB  
//Gpio_SetIO(en_gpio_port_t enPort, en_gpio_pin_t enPin);
//Gpio_ClrIO(en_gpio_port_t enPort, en_gpio_pin_t enPin);
#define SCL_H         	 Gpio_SetIO(EEPROM_I2C_SCL_GPIO_PORT , EEPROM_I2C_SCL_PIN)   /*GPIOB->BSRR = GPIO_Pin_6*/
#define SCL_L            Gpio_ClrIO(EEPROM_I2C_SCL_GPIO_PORT , EEPROM_I2C_SCL_PIN) /*GPIOB->BRR  = GPIO_Pin_6 */
   
#define SDA_H         	 Gpio_SetIO(EEPROM_I2C_SDA_GPIO_PORT , EEPROM_I2C_SDA_PIN)   /*GPIOB->BSRR = GPIO_Pin_7*/
#define SDA_L         	 Gpio_ClrIO(EEPROM_I2C_SDA_GPIO_PORT , EEPROM_I2C_SDA_PIN) /*GPIOB->BRR  = GPIO_Pin_7*/
//Gpio_GetInputIO(en_gpio_port_t enPort, en_gpio_pin_t enPin);
#define SCL_read       	Gpio_GetInputIO(EEPROM_I2C_SCL_GPIO_PORT , EEPROM_I2C_SCL_PIN)/* GPIOB->IDR  & GPIO_Pin_6   */
#define SDA_read       	Gpio_GetInputIO(EEPROM_I2C_SDA_GPIO_PORT , EEPROM_I2C_SDA_PIN)/*GPIOB->IDR  & GPIO_Pin_7	  */

#define I2C_PageSize  8  

#define ADDR_24C08		0xA0

/* Private function prototypes -----------------------------------------------*/
void I2C_delay(void);
FunctionalState I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NoAck(void);
FunctionalState I2C_WaitAck(void) ;
void I2C_SendByte(uint8_t SendByte) ;
uint8_t I2C_ReceiveByte(void)  ;
void I2C_Configuration(void);

typedef enum{
  TRIG_T_MEASUREMENT_HM    = 0xE3, // command trig. temp meas. hold master
  TRIG_RH_MEASUREMENT_HM   = 0xE5, // command trig. humidity meas. hold master
  TRIG_T_MEASUREMENT_POLL  = 0xF3, // command trig. temp meas. no hold master
  TRIG_RH_MEASUREMENT_POLL = 0xF5, // command trig. humidity meas. no hold master
  USER_REG_W               = 0xE6, // command writing user register
  USER_REG_R               = 0xE7, // command reading user register
  SOFT_RESET               = 0xFE  // command soft reset
}SHT2xCommand;

typedef enum {
  SHT2x_RES_12_14BIT       = 0x00, // RH=12bit, T=14bit
  SHT2x_RES_8_12BIT        = 0x01, // RH= 8bit, T=12bit
  SHT2x_RES_10_13BIT       = 0x80, // RH=10bit, T=13bit
  SHT2x_RES_11_11BIT       = 0x81, // RH=11bit, T=11bit
  SHT2x_RES_MASK           = 0x81  // Mask for res. bits (7,0) in user reg.
} SHT2xResolution;

typedef enum {
  SHT2x_EOB_ON             = 0x40, // end of battery
  SHT2x_EOB_MASK           = 0x40, // Mask for EOB bit(6) in user reg.
} SHT2xEob;

typedef enum {
  SHT2x_HEATER_ON          = 0x04, // heater on
  SHT2x_HEATER_OFF         = 0x00, // heater off
  SHT2x_HEATER_MASK        = 0x04, // Mask for Heater bit(2) in user reg.
} SHT2xHeater;


typedef enum{
  HUMIDITY,
  TEMP
}SHT2xMeasureType;

typedef enum{
  I2C_ADR_W                = 128,   // sensor I2C address + write bit
  I2C_ADR_R                = 129    // sensor I2C address + read bit
}I2cHeader;

 
typedef struct
{
  u16 eCO2;
	float Temp_C;
	float Hum_RH;
	u16 TVOC;
	u16 HCHO;
}Is_AQM_Dat;

extern Is_AQM_Dat Is_AQM;

FunctionalState SHT2X_IIC_WriteByte(uint8_t WriteAddress,uint8_t SendByte);
FunctionalState SHT2X_IIC_ReadByte( uint8_t ReadAddress, uint16_t length  ,uint8_t* pBuffer);
FunctionalState SHT2x_CheckCrc(u8 data[],u8 startBytes,u8 number, u8 checksum);
u8 SHT2x_ReadUserRegister(void);
FunctionalState SHT2x_WriteUserRegister(u8 userdata);
FunctionalState SHT2x_SoftReset(void);
FunctionalState SHT2x_Calc_T(void);
FunctionalState SHT2x_Calc_RH(void);
u8 SHT2x_GetSerialNumber(u8 *pBuffer1,u8 *pBuffer2);
void SHT2X_TEST(void);
char SHT2X_Init(void);
FunctionalState Is_AQM_D(void);
void Is_AQM_D_out(void);
#endif 
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
