/******************** (C) COPYRIGHT  Դ�ع����� ********************************
 * �ļ���  ��SHT2X.c
 * ����    ����ʼ��SHT20��һЩ�����Ĳ��� 
 * ����    ��zhuoyingxingyu
 * �Ա�    ��Դ�ع�����http://vcc-gnd.taobao.com/
 * ��̳��ַ������԰��-Ƕ��ʽ������̳http://vcc-gnd.com/
 * �汾����: 2015-10-20
 * Ӳ������: PB6-I2C1_SCL��PB7-I2C1_SDA
 * ���Է�ʽ��J-Link-OB
**********************************************************************************/
//ͷ�ļ�
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
  * @brief  EEPROM�ܽ�����
  * @param  ��
  * @retval ��
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
    Gpio_Init(GpioPortB, GpioPin8,&stcGpioCfg); //scl//�˿ڳ�ʼ��
    Gpio_Init(GpioPortB, GpioPin9,&stcGpioCfg); //sda
}

 /**
  * @file   I2C_delay
  * @brief  �ӳ�ʱ��
  * @param  ��
  * @retval ��
  */
void I2C_delay(void)
{	
   uint8_t i=50; /* ��������Ż��ٶ�,��������͵�5����д�� */
   while(i) 
   { 
     i--; 
   } 
}

 /**
  * @file   I2C_Start
  * @brief  ��ʼ�ź�
  * @param  ��
  * @retval ��
  */
FunctionalState I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)return DISABLE;	/* SDA��Ϊ�͵�ƽ������æ,�˳� */
	SDA_L;
	I2C_delay();
	if(SDA_read) return DISABLE;	/* SDA��Ϊ�ߵ�ƽ�����߳���,�˳� */
	SDA_L;
	I2C_delay();
	return ENABLE;
}

 /**
  * @file   I2C_Stop
  * @brief  ֹͣ�ź�
  * @param  ��
  * @retval ��
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
  * @brief  Ӧ���ź�
  * @param  ��
  * @retval ��
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
  * @brief  ��Ӧ���ź�
  * @param  ��
  * @retval ��
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
  * @brief  �ȴ�Ack
  * @param  ��
  * @retval ����Ϊ:=1��ACK,=0��ACK
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
  * @brief  ���ݴӸ�λ����λ
  * @param  - SendByte: ���͵�����
  * @retval ��
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
  * @brief  ���ݴӸ�λ����λ
  * @param  ��
  * @retval I2C���߷��ص�����
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
  * @brief  ��SHT20дһ�ֽ�����
  * @param  
	*          - SendByte: ��д������
	*          - WriteAddress: ��д���ַ
  * @retval ����Ϊ:=1�ɹ�д��,=0ʧ��
  */
FunctionalState SHT2X_IIC_WriteByte(uint8_t WriteAddress,uint8_t SendByte)
{		
    if(!I2C_Start())return DISABLE;
    I2C_SendByte(0x80); 
    if(!I2C_WaitAck()){I2C_Stop(); return DISABLE;}
    I2C_SendByte(WriteAddress);   /* ���õ���ʼ��ַ */      
    I2C_WaitAck();	
    I2C_SendByte(SendByte);
    I2C_WaitAck();   
    I2C_Stop(); 
    return ENABLE;
}	
 /**
  * @file   SHT2X_IIC_ReadByte
  * @brief  ��SHT20��ȡһ������
  * @param  
	*					- pBuffer: ��Ŷ�������
	*     	  - length: ����������
	*         - ReadAddress: ��������ַ
  * @retval ����Ϊ:=1�ɹ�����,=0ʧ��
  */
FunctionalState SHT2X_IIC_ReadByte( uint8_t ReadAddress, uint16_t length  ,uint8_t* pBuffer)
{		
	  if(!I2C_Start())return DISABLE;
    I2C_SendByte(0x80); /* ���ø���ʼ��ַ+������ַ */ 
    if(!I2C_WaitAck())
		{I2C_Stop(); return DISABLE;}
    I2C_SendByte(ReadAddress);   /* ���õ���ʼ��ַ */      
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
	*					- data[]: �ȴ����������
	*     	  - startByte: ��ʼ����ı��
	*         - nbrOfBytes: �������
	*         - checksum: ���յ���CRC����
  * @retval ����Ϊ:=1���ɹ�,=0���ʧ��
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
  * @retval ���ض�ȡ���Ĳ���ֵ
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
  * @param  userdata��Ҫд�Ĳ���
  * @retval ENABLE���ɹ���DISABLE��ʧ��
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
  * @brief  �����¶�
  * @param  NO
  * @retval �����¶�ֵ
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
  * @brief  ����ʪ��
  * @param  NO
  * @retval ����ʪ��ֵ
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
  * @brief  �����λ
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
  * @brief  �õ�SHT20�����к�
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
  * @brief  SHT20��ʼ��
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
//		printf("SHT2Xͨ�Ŵ���\r\n");
		};
    delay1ms(100);
		if(SHT2x_WriteUserRegister(0x3a))
		{
			state = 0;
//		  printf("���óɹ�\r\n");
		}
		else 
		{
			state =1;
//		  printf("���ô���\r\n");
		}
		return state;
}
 /**
  * @file   SHT2X_TEST
  * @brief  SHT20����
  * @param  NO
  * @retval NO
  */
//void SHT2X_TEST(void)
//{
//		if(SHT2x_Calc_T())
//		{ 
//			 printf("�¶ȣ�%f\r\n",temperatureC);
//		   //OLED_shuji(38,17,(s8)(temperatureC));
//		}
//    /*ʪ�Ȳ���*/
//		if(SHT2x_Calc_RH())
//		{
//			printf("ʪ�ȣ�%f%%\r\n",humidityRH);
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
		Is_AQM.eCO2 = (data[1]<<8)|data[2];       //������̼
//		Is_AQM.Temp_C = (buf[4]*8-669)/10;
//		Is_AQM.Hum_RH = (buf[5]*8-125)/10;
		Is_AQM.TVOC = (data[8]<<8)|data[9];       //�л���̬���� �ӷ����л���
		Is_AQM.HCHO = (data[10]<<8)|data[11];     //��ȩ
	}
	
  if(SHT2x_Calc_T())
	{ 
		 Is_AQM.Temp_C = temperatureC; 
	}
	/*ʪ�Ȳ���*/
	if(SHT2x_Calc_RH())
	{
		Is_AQM.Hum_RH = humidityRH;
	}
// 	printf("--->Is_AQM.eCO2=%04dppm  Is_AQM.Temp_C=%3.2f'C  Is_AQM.Hum_RH=%3.2f%%RH  Is_AQM.TVOC=%04dug/m3  Is_AQM.HCHO=%04dug/m3\r\n",Is_AQM.eCO2,Is_AQM.Temp_C,Is_AQM.Hum_RH,Is_AQM.TVOC,Is_AQM.HCHO);	
}
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
