/******************** (C) COPYRIGHT  源地工作室 ********************************
 * 文件名  ：SHT2X.c
 * 描述    ：初始化SHT20及一些基本的操作 
 * 作者    ：zhuoyingxingyu
 * 淘宝    ：源地工作室http://vcc-gnd.taobao.com/
 * 论坛地址：极客园地-嵌入式开发论坛http://vcc-gnd.com/
 * 版本更新: 2015-10-20
 * 硬件连接: PB6-I2C1_SCL、PB7-I2C1_SDA
 * 调试方式：J-Link-OB
**********************************************************************************/
//头文件
#include "SHT2X.h"
#include "app_uart.h"
#include "math.h"
#include "i2c.h"
#include "gpio.h"
#include "ddl.h"
#include "uart.h"

 float temperatureC;
 float humidityRH;
 
 u8 sndata1[8];
 u8 sndata2[6];
 u32 SN1; 
 u32 SN2; 

/**
  * @file   I2C_Configuration
  * @brief  EEPROM管脚配置
  * @param  无
  * @retval 无
  */
void I2C_Configuration(void)
{
//  GPIO_InitTypeDef  GPIO_InitStructure; 
//  /* Configure I2C2 pins: PB6->SCL and PB7->SDA */
//  RCC_APB2PeriphClockCmd(EEPROM_I2C_SCL_GPIO_RCC|EEPROM_I2C_SDA_GPIO_RCC, ENABLE);
//	
//	GPIO_InitStructure.GPIO_Pin =  EEPROM_I2C_SCL_PIN ;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
//	GPIO_Init(EEPROM_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Pin =  EEPROM_I2C_SDA_PIN;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;  
//	GPIO_Init(EEPROM_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
	 
    stc_gpio_config_t stcGpioCfg;
	  DDL_ZERO_STRUCT(stcGpioCfg);
	  Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    stcGpioCfg.enDir = GpioDirOut;
    stcGpioCfg.enOD = GpioOdEnable;
    stcGpioCfg.enPuPd = GpioPu;
    Gpio_Init(GpioPortB, GpioPin8,&stcGpioCfg); //scl//端口初始化
    Gpio_Init(GpioPortB, GpioPin9,&stcGpioCfg); //sda
}

 /**
  * @file   I2C_delay
  * @brief  延迟时间
  * @param  无
  * @retval 无
  */
void I2C_delay(void)
{	
   uint8_t i=50; /* 这里可以优化速度,经测试最低到5还能写入 */
   while(i) 
   { 
     i--; 
   } 
}

 /**
  * @file   I2C_Start
  * @brief  起始信号
  * @param  无
  * @retval 无
  */
FunctionalState I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)return DISABLE;	/* SDA线为低电平则总线忙,退出 */
	SDA_L;
	I2C_delay();
	if(SDA_read) return DISABLE;	/* SDA线为高电平则总线出错,退出 */
	SDA_L;
	I2C_delay();
	return ENABLE;
}

 /**
  * @file   I2C_Stop
  * @brief  停止信号
  * @param  无
  * @retval 无
  */
void I2C_Stop(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
}

 /**
  * @file   I2C_Ack
  * @brief  应答信号
  * @param  无
  * @retval 无
  */
void I2C_Ack(void)
{	
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}

 /**
  * @file   I2C_NoAck
  * @brief  无应答信号
  * @param  无
  * @retval 无
  */
void I2C_NoAck(void)
{	
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}

 /**
  * @file   I2C_WaitAck
  * @brief  等待Ack
  * @param  无
  * @retval 返回为:=1有ACK,=0无ACK
  */
FunctionalState I2C_WaitAck(void) 	
{
	SCL_L;
	I2C_delay();
	SDA_H;			
	I2C_delay();
	SCL_H;
	I2C_delay();
	if(SDA_read)
	{
      SCL_L;
      return DISABLE;
	}
	SCL_L;
	return ENABLE;
}

 /**
  * @file   I2C_SendByte
  * @brief  数据从高位到低位
  * @param  - SendByte: 发送的数据
  * @retval 无
  */
void I2C_SendByte(uint8_t SendByte) 
{
    uint8_t i=8;
    while(i--)
    {
			SCL_L;
			I2C_delay();
			if(SendByte&0x80)
			SDA_H;  
			else 
			SDA_L;   
			SendByte<<=1;
			I2C_delay();
			SCL_H;
			I2C_delay();
    }
    SCL_L;
}


 /**
  * @file   I2C_ReceiveByte
  * @brief  数据从高位到低位
  * @param  无
  * @retval I2C总线返回的数据
  */
uint8_t I2C_ReceiveByte(void)  
{ 
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2C_delay();
	    SCL_H;
      I2C_delay();	
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;
    return ReceiveByte;
} 
   
 
/**
  * @file   SHT2X_IIC_WriteByte
  * @brief  向SHT20写一字节数据
  * @param  
	*          - SendByte: 待写入数据
	*          - WriteAddress: 待写入地址
  * @retval 返回为:=1成功写入,=0失败
  */
FunctionalState SHT2X_IIC_WriteByte(uint8_t WriteAddress,uint8_t SendByte)
{		
    if(!I2C_Start())return DISABLE;
    I2C_SendByte(0x80); 
    if(!I2C_WaitAck()){I2C_Stop(); return DISABLE;}
    I2C_SendByte(WriteAddress);   /* 设置低起始地址 */      
    I2C_WaitAck();	
    I2C_SendByte(SendByte);
    I2C_WaitAck();   
    I2C_Stop(); 
    return ENABLE;
}	
 /**
  * @file   SHT2X_IIC_ReadByte
  * @brief  从SHT20读取一串数据
  * @param  
	*					- pBuffer: 存放读出数据
	*     	  - length: 待读出长度
	*         - ReadAddress: 待读出地址
  * @retval 返回为:=1成功读入,=0失败
  */
FunctionalState SHT2X_IIC_ReadByte( uint8_t ReadAddress, uint16_t length  ,uint8_t* pBuffer)
{		
	  if(!I2C_Start())return DISABLE;
    I2C_SendByte(0x80); /* 设置高起始地址+器件地址 */ 
    if(!I2C_WaitAck())
		{I2C_Stop(); return DISABLE;}
    I2C_SendByte(ReadAddress);   /* 设置低起始地址 */      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(0x81);
    I2C_WaitAck();
    while(length)
    {
			*pBuffer = I2C_ReceiveByte();
			if(length == 1)
			I2C_NoAck();
			else I2C_Ack(); 
			pBuffer++;
			length--;
    }
    I2C_Stop();
    return ENABLE;
} 
 /**
  * @file   SHT2x_CheckCrc
  * @brief  calculates checksum for n bytes of data and compares it with expected
  * @param  
	*					- data[]: 等待检验的数据
	*     	  - startByte: 开始数组的标号
	*         - nbrOfBytes: 检验个数
	*         - checksum: 接收到的CRC数据
  * @retval 返回为:=1检测成功,=0检测失败
  */
FunctionalState SHT2x_CheckCrc(u8 data[],u8 startBytes,u8 number, u8 checksum)
{
	u8 bit=0;
	u8 crc = 0;	
  u8 byteCtr;
  //calculates 8-Bit checksum with given polynomial
  for (byteCtr = startBytes; byteCtr < startBytes+number; ++byteCtr)
  { crc ^= (data[byteCtr]);
    for (bit = 8; bit > 0; --bit)
    { if (crc & 0x80) crc = (crc << 1) ^ 0x131;
      else crc = (crc << 1);
    }
  }
  if (crc ==checksum) 
	return ENABLE;
  else
	return DISABLE;
}
 /**
  * @file   SHT2x_ReadUserRegister
  * @brief  reads the SHT2x user register 
  * @param  no
  * @retval 返回读取到的参数值
  */
u8 SHT2x_ReadUserRegister(void)
{
	u8 data[1]={0};
  SHT2X_IIC_ReadByte( USER_REG_R, 1 ,data);
  return data[0];	
}
 /**
  * @file   SHT2x_WriteUserRegister
  * @brief  writes the SHT2x user register (8bit)
  * @param  userdata：要写的参数
  * @retval ENABLE：成功，DISABLE：失败
  */
FunctionalState SHT2x_WriteUserRegister(u8 userdata)
{
	SHT2X_IIC_WriteByte(USER_REG_W ,userdata);
	delay1ms(30);
	if(userdata==SHT2x_ReadUserRegister())
	return ENABLE;
	else 
	return DISABLE;
}

 /**
  * @file   SHT2x_Calc_T
  * @brief  计算温度
  * @param  NO
  * @retval 返回温度值
  */
FunctionalState SHT2x_Calc_T(void)
{
	 u8 length=0;
	 u8 Tdata[3]={0};
	 if(!I2C_Start())return DISABLE;
    I2C_SendByte(I2C_ADR_W); 
    if(!I2C_WaitAck())
		{I2C_Stop(); return DISABLE;}
    I2C_SendByte(TRIG_T_MEASUREMENT_POLL);      
    I2C_WaitAck();
		delay10us(20);
	  I2C_Stop();
		do
		{
		I2C_Start();
    I2C_SendByte(I2C_ADR_R);
		}
		while(!I2C_WaitAck());
		for(length=0;length<=3;length++)
		{
			Tdata[length]=I2C_ReceiveByte();
			I2C_Ack(); 		
		};
		I2C_NoAck();		
		I2C_Stop();		
		if(((Tdata[0]+Tdata[1]+Tdata[2])>0)&&SHT2x_CheckCrc(Tdata,0,2,Tdata[2]))	
    temperatureC= (-46.85 + (175.72/65536 )*((float)((((u16)Tdata[0]<<8)+(u16)Tdata[1])&0xfffc)));
		else
		return DISABLE;
	  return ENABLE;
}


 /**
  * @file   SHT2x_Calc_RH
  * @brief  计算湿度
  * @param  NO
  * @retval 返回湿度值
  */
FunctionalState SHT2x_Calc_RH(void)
{	
   u8 length=0;
	 u8 RHdata[3]={0};
	 if(!I2C_Start())return DISABLE;
    I2C_SendByte(I2C_ADR_W); 
    if(!I2C_WaitAck())
		{I2C_Stop(); return DISABLE;}
    I2C_SendByte(TRIG_RH_MEASUREMENT_POLL);      
    I2C_WaitAck();
		delay10us(20);
	  I2C_Stop();
		do
		{
		I2C_Start();
    I2C_SendByte(I2C_ADR_R);
		}
		while(!I2C_WaitAck());
		for(length=0;length<=3;length++)
		{
			RHdata[length]=I2C_ReceiveByte();
			I2C_Ack(); 		
		};
		I2C_NoAck();		
		I2C_Stop();
		if(((RHdata[0]+RHdata[1]+RHdata[2])>0)&&SHT2x_CheckCrc(RHdata,0,2,RHdata[2]))		
	  humidityRH = -6.0 + 125.0/65536 * ((float)((((u16)RHdata[0]<<8)+(u16)RHdata[1])&0xfff0)); 
		else
		return DISABLE;
	  return ENABLE;
}
 /**
  * @file   SHT2x_SoftReset
  * @brief  软件复位
  * @param  NO
  * @retval NO
  */
FunctionalState SHT2x_SoftReset(void)
{
	 if(!I2C_Start())return DISABLE;
    I2C_SendByte(0x80); 
    if(!I2C_WaitAck()){I2C_Stop(); return DISABLE;}
    I2C_SendByte(SOFT_RESET);       
    I2C_WaitAck();	  
    I2C_Stop(); 
  	delay1ms(1500);
		return ENABLE;	
}

 /**
  * @file   SHT2x_GetSerialNumber
  * @brief  得到SHT20的序列号
  * @param  
  * @retval 
  */
u8 SHT2x_GetSerialNumber(u8 *pBuffer1,u8 *pBuffer2)
{   u8 length=8;
	  if(!I2C_Start())return DISABLE;
    I2C_SendByte(0x80); 
    if(!I2C_WaitAck()){I2C_Stop(); return DISABLE;}
    I2C_SendByte(0xfa);       
    I2C_WaitAck();	
    I2C_SendByte(0x0f);
    I2C_WaitAck();   
		if(!I2C_Start())return DISABLE;
    I2C_SendByte(0x81); 
    if(!I2C_WaitAck()){I2C_Stop(); return DISABLE;}
    while(length)
    {
      *pBuffer1 = I2C_ReceiveByte();
      if(length == 1)
				I2C_NoAck();
      else I2C_Ack(); 
      pBuffer1++;
      length--;
    }
     I2C_Stop();
		length=6;
	  if(!I2C_Start())return DISABLE;
    I2C_SendByte(0x80); 
    if(!I2C_WaitAck()){I2C_Stop(); return DISABLE;}
    I2C_SendByte(0xfc);     
    I2C_WaitAck();	
    I2C_SendByte(0xc9);
    I2C_WaitAck();   
		if(!I2C_Start())return DISABLE;
    I2C_SendByte(0x81); 
    if(!I2C_WaitAck()){I2C_Stop(); return DISABLE;}
    while(length)
    {
      *pBuffer2 = I2C_ReceiveByte();
      if(length == 1)
				I2C_NoAck();
      else I2C_Ack(); 
      pBuffer2++;
      length--;
    }
     I2C_Stop();	
    return ENABLE;
}
 /**
  * @file   SHT2X_Init
  * @brief  SHT20初始化
  * @param  NO
  * @retval NO
  */
char SHT2X_Init(void)
{
	  char state;
		SHT2x_GetSerialNumber(sndata1,sndata2);
		if((sndata1[0]+sndata1[1]+sndata1[3]+sndata1[4]+sndata1[5]+sndata1[6]+sndata1[7])>0)
		{		
		if(
		SHT2x_CheckCrc(sndata1,0,1,sndata1[1])&&
		SHT2x_CheckCrc(sndata1,2,1,sndata1[3])&&
		SHT2x_CheckCrc(sndata1,4,1,sndata1[5])&&
		SHT2x_CheckCrc(sndata1,6,1,sndata1[7])&&
		SHT2x_CheckCrc(sndata2,0,2,sndata2[2])&&
		SHT2x_CheckCrc(sndata2,3,2,sndata2[5])
		)
		{
//		printf("SHT2X CRC ok\r\n");
		SN1=((sndata2[3]<<24)+(sndata2[4]<<16)+(sndata1[0]<<8)+sndata1[2]);
		SN2=((sndata1[4]<<24)+(sndata1[6]<<16)+(sndata2[0]<<8)+sndata2[1]);
//		printf("SHT2X SN:0x%x%x\r\n",SN1,SN2);
		}
//		else
//		printf("SHT2X CRC error\r\n");	
		}
		else
		{
		SHT2x_GetSerialNumber(sndata1,sndata2);
//		printf("SHT2X通信错误\r\n");
		};
    delay1ms(100);
		if(SHT2x_WriteUserRegister(0x3a))
		{
			state = 0;
//		  printf("设置成功\r\n");
		}
		else 
		{
			state =1;
//		  printf("设置错误\r\n");
		}
		return state;
}
 /**
  * @file   SHT2X_TEST
  * @brief  SHT20测试
  * @param  NO
  * @retval NO
  */
//void SHT2X_TEST(void)
//{
//		if(SHT2x_Calc_T())
//		{ 
//			 printf("温度：%f\r\n",temperatureC);
//		   //OLED_shuji(38,17,(s8)(temperatureC));
//		}
//    /*湿度测试*/
//		if(SHT2x_Calc_RH())
//		{
//			printf("湿度：%f%%\r\n",humidityRH);
//		}
//	

//} 

//FunctionalState Set_AQM(void)
//{
//	 if(!I2C_Start())return DISABLE;
//    I2C_SendByte(0xa2); 
//    if(!I2C_WaitAck())
//		{I2C_Stop(); return DISABLE;}
//    I2C_SendByte(0x51);      
//    I2C_WaitAck();
//		delay10us(20);
//	  I2C_Stop();
//}

static u8 data[13];
FunctionalState Is_AQM_D(void)
{
	 u8 length=0;
//	  char i;
	 if(!I2C_Start())return DISABLE;
    I2C_SendByte(0xa2); 
    if(!I2C_WaitAck())
		{I2C_Stop(); return DISABLE;}
    I2C_SendByte(0x51);      
    I2C_WaitAck();
		delay10us(20);
	  I2C_Stop();
		do
		{
		I2C_Start();
    I2C_SendByte(0xa3);
		}
		while(!I2C_WaitAck());
		for(length=0;length<=12;length++)
		{
			data[length]=I2C_ReceiveByte();
			I2C_Ack(); 		
		};
		I2C_NoAck();		
		I2C_Stop();		
 
// 	  for(i=0;i<13;i++)
//    Uart_SendDataPoll(M0P_UART1,data[i]);
		
	  return ENABLE;
}

Is_AQM_Dat Is_AQM;

void Is_AQM_D_out(void)
{
//  u8 buf[13];
	Is_AQM_D();
	if((data[0]==0xff) && (data[3]==0x00))
	{
		Is_AQM.eCO2 = (data[1]<<8)|data[2];       //二氧化碳
//		Is_AQM.Temp_C = (buf[4]*8-669)/10;
//		Is_AQM.Hum_RH = (buf[5]*8-125)/10;
		Is_AQM.TVOC = (data[8]<<8)|data[9];       //有机气态物质 挥发性有机物
		Is_AQM.HCHO = (data[10]<<8)|data[11];     //甲醛
	}
	
  if(SHT2x_Calc_T())
	{ 
		 Is_AQM.Temp_C = temperatureC; 
	}
	/*湿度测试*/
	if(SHT2x_Calc_RH())
	{
		Is_AQM.Hum_RH = humidityRH;
	}
// 	printf("--->Is_AQM.eCO2=%04dppm  Is_AQM.Temp_C=%3.2f'C  Is_AQM.Hum_RH=%3.2f%%RH  Is_AQM.TVOC=%04dug/m3  Is_AQM.HCHO=%04dug/m3\r\n",Is_AQM.eCO2,Is_AQM.Temp_C,Is_AQM.Hum_RH,Is_AQM.TVOC,Is_AQM.HCHO);	
}
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
