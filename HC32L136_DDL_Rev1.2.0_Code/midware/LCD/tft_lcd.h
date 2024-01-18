#ifndef __LCD_H
#define __LCD_H		
#include "stdint.h"
#include "stdlib.h"
#include "gpio.h"

//////////////////////////////////////////////////////////////////////////////////	 
//////////////////////////////////////////////////////////////////////////////////
 
  
//LCD��Ҫ������
typedef struct  
{										    
	u16 width;			//LCD ���
	u16 height;			//LCD �߶�
	u16 id;				//LCD ID
	u8  dir;			//���������������ƣ�0��������1��������	
	u8	wramcmd;		//��ʼдgramָ��
	u8  setxcmd;		//����x����ָ��
	u8  setycmd;		//����y����ָ��	 
}_lcd_dev; 	  

//LCD����
extern _lcd_dev lcddev;	//����LCD��Ҫ����
//LCD�Ļ�����ɫ�ͱ���ɫ	   
extern u16  POINT_COLOR;//Ĭ�Ϻ�ɫ    
extern u16  BACK_COLOR; //������ɫ.Ĭ��Ϊ��ɫ

#define  LCD_SIZE_X		 240		 
#define  LCD_SIZE_Y		 320	


//////////////////////////////////////////////////////////////////////////////////	 
//-----------------LCD�˿ڶ���---------------- 
 
//ɨ�跽����
#define L2R_U2D  0 //������,���ϵ���
#define L2R_D2U  1 //������,���µ���
#define R2L_U2D  2 //���ҵ���,���ϵ���
#define R2L_D2U  3 //���ҵ���,���µ���

#define U2D_L2R  4 //���ϵ���,������
#define U2D_R2L  5 //���ϵ���,���ҵ���
#define D2U_L2R  6 //���µ���,������
#define D2U_R2L  7 //���µ���,���ҵ���	 

extern u8 DFT_SCAN_DIR;


//#define LCD_RST_PIN    GPIO_Pin_12	 //RST ��Ӧ��Ƭ�����Ŷ��� P3^3 / ��ӦSTM32�� PA11
#define LCD_RD_PIN     GpioPin10	 //RD  ��Ӧ��Ƭ�����Ŷ��� P2^6 / ��ӦSTM32�� PB1
#define LCD_WR_PIN     GpioPin11	 //WR  ��Ӧ��Ƭ�����Ŷ��� P2^5 / ��ӦSTM32�� PB2
#define LCD_RS_PIN     GpioPin12   //RS  ��Ӧ��Ƭ�����Ŷ��� P3^2 / ��ӦSTM32�� PA8
#define LCD_CS_PIN     GpioPin15	 //CS  ��Ӧ��Ƭ�����Ŷ��� P2^7 / ��ӦSTM32�� PB0
#define LCD_LED_PIN    GpioPin7   //PD07   OLD_PB10  LED

//#define    LCD_RST(x)  x ? GPIO_SetBits(GPIOA, LCD_RST_PIN):   GPIO_ResetBits(GPIOA, LCD_RST_PIN) //�Լ�����λ��������
#define    LCD_RD(x)    x ? Gpio_SetIO(GpioPortC, LCD_RD_PIN) :   Gpio_ClrIO(GpioPortC, LCD_RD_PIN)
#define    LCD_WR(x)    x ? Gpio_SetIO(GpioPortC, LCD_WR_PIN) :   Gpio_ClrIO(GpioPortC, LCD_WR_PIN)
#define    LCD_RS(x)    x ? Gpio_SetIO(GpioPortC, LCD_RS_PIN) :   Gpio_ClrIO(GpioPortC, LCD_RS_PIN)
#define    LCD_CS(x)    x ? Gpio_SetIO(GpioPortA, LCD_CS_PIN) :   Gpio_ClrIO(GpioPortA, LCD_CS_PIN)
#define    LCD_LED(x)   x ? Gpio_SetIO(GpioPortD, LCD_LED_PIN):   Gpio_ClrIO(GpioPortD, LCD_LED_PIN)
//PC0~15,��Ϊ���ݿ�
#define DATAOUT(x) Gpio_SetClrPort(GpioPortB, ((x )<<16)|~(x )); //GPIOB->ODR=x; //�������
#define DATAIN     (Gpio_GetInputData(GpioPortB)); //GPIOB->IDR;   //��������

//������ɫ
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	   0x001F  
#define BRED             0XF81F
#define GRED 			       0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40 //��ɫ
#define BRRED 			     0XFC07 //�غ�ɫ
#define GRAY  			     0X8430 //��ɫ
#define DeepGray         0xad55 //���ɫ
//GUI��ɫ
#define DeepColor        0x18e6 //��ɫ����
#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 0X5458 //����ɫ
//������ɫΪPANEL����ɫ 
 
#define LIGHTGREEN     	 0X841F //ǳ��ɫ
#define LIGHTGRAY        0XEF5B //ǳ��ɫ(PANNEL)
#define LGRAY 			     0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ

#define LGRAYBLUE        0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE           0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)

#define GUI_Green			 0x1eb6
#define GUI_Yellow		 0xfe85
#define GUI_Red				 0xe330
#define GUI_ReadColor	 0x4e1f
#define GUI_WriteColor 0x771a
	    															  
void LCD_Init(void);													   	//��ʼ��
void LCD_DisplayOn(void);													//����ʾ
void LCD_DisplayOff(void);													//����ʾ
void LCD_Clear(u16 Color);	 												//����
void LCD_SetCursor(u16 Xpos, u16 Ypos);										//���ù��
 
void LCD_DrawPoint(u16 x,u16 y,u16 color);//����
void LCD_Fast_DrawPoint(u16 x,u16 y,u16 color);								//���ٻ���
u16  LCD_ReadPoint(u16 x,u16 y); 											//���� 
void LCD_Draw_Circle(u16 x0,u16 y0,u8 r,u16 color);									//��Բ
void LCD_Draw_Circle1(int xc, int yc,u16 c,int r, int fill);
 						//����
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2,u16 color);
 	   				//������
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color);
//x y ����  ��ϸ  ��ɫ  ������
void Paint_brush(u16 x,u16 y,u16 l,u8 t,u16 color,u8 flag);
void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color);		   				//��䵥ɫ
void LCD_Color_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 *color);				//���ָ����ɫ
 
void LCD_ShowNum(u16 x,u16 y,u32 num,u8 len,u8 size);  						//��ʾһ������
void LCD_ShowxNum(u16 x,u16 y,u32 num,u8 len,u8 size,u8 mode);				//��ʾ ����
 
void LCD_Set_Window(u16 sx,u16 sy,u16 width,u16 height);
void IO_init(void);

void opt_delay(u8 i);
void LCD_WR_REG(u16);
void LCD_WR_DATA(u16);
void LCD_WriteReg(u8 LCD_Reg, u16 LCD_RegValue);
u16 LCD_ReadReg(u8 LCD_Reg);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAM(u16 RGB_Code);		  
void LCD_Scan_Dir(u8 dir);							//������ɨ�跽��
void LCD_Display_Dir(u8 dir);						//������Ļ��ʾ����

//��ָ��λ����ʾһ���ַ�
//void LCD_ShowChar(u16 x,u16 y,u16 For_color,u16 Bk_color, char ch);
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 size,u8 mode);
void LCD_ShowString(u16 x,u16 y,u16 width,u16 height,u8 size,u8 *p);
//void LCD_ShowChar6X12(u16 x,u16 y,u16 For_color,u16 Bk_color, char ch);
//void LCD_ShowString_n(u16  x,u16 y,u16 For_color,u16 Bk_color,char *p,u8 num);
//void LCD_ShowString6X12(u16  x,u16 y,u16 For_color,u16 Bk_color,char *p);
//��ʾ�ַ���
//void LCD_ShowString(u16  x,u16 y,u16 For_color,u16 Bk_color,char *p);
void progress_bar(u16 Xpos, u16 Ypos, u16 Width, u16 Height,u16 color_f,u16 color_b, u16 bar_value);
void LCD_WriteBMP(u8 Xpos, u16 Ypos, u16 Width, u8 Height, u8 *bitmap);
void Gui_DrawFont_GBK12(u16 x, u16 y, u16 fc, u16 bc, u8 *s, u16 hz16_num);
void Gui_DrawFont_GBK16(u16 x, u16 y, u16 fc, u16 bc, u8 *s, u16 hz16_num);
void Gui_DrawFont_GBK20(u16 x, u16 y, u16 fc, u16 bc, u8 *s, u16 hz20_num);
void Gui_DrawFont_GBK24(u16 x, u16 y, u16 fc, u16 bc, u8 *s, u16 hz24_num);
void Gui_DrawFont_GBK32(u16 x, u16 y, u16 fc, u16 bc, u8 *s, u16 hz32_num);
void Gui_DrawFont_GBK1632(u16 x, u16 y, u16 fc, u16 bc, u8 *s, u16 hz1632_num);


//9320/9325 LCD�Ĵ���  
#define R0             0x00
#define R1             0x01
#define R2             0x02
#define R3             0x03
#define R4             0x04
#define R5             0x05
#define R6             0x06
#define R7             0x07
#define R8             0x08
#define R9             0x09
#define R10            0x0A
#define R12            0x0C
#define R13            0x0D
#define R14            0x0E
#define R15            0x0F
#define R16            0x10
#define R17            0x11
#define R18            0x12
#define R19            0x13
#define R20            0x14
#define R21            0x15
#define R22            0x16
#define R23            0x17
#define R24            0x18
#define R25            0x19
#define R26            0x1A
#define R27            0x1B
#define R28            0x1C
#define R29            0x1D
#define R30            0x1E
#define R31            0x1F
#define R32            0x20
#define R33            0x21
#define R34            0x22
#define R36            0x24
#define R37            0x25
#define R40            0x28
#define R41            0x29
#define R43            0x2B
#define R45            0x2D
#define R48            0x30
#define R49            0x31
#define R50            0x32
#define R51            0x33
#define R52            0x34
#define R53            0x35
#define R54            0x36
#define R55            0x37
#define R56            0x38
#define R57            0x39
#define R59            0x3B
#define R60            0x3C
#define R61            0x3D
#define R62            0x3E
#define R63            0x3F
#define R64            0x40
#define R65            0x41
#define R66            0x42
#define R67            0x43
#define R68            0x44
#define R69            0x45
#define R70            0x46
#define R71            0x47
#define R72            0x48
#define R73            0x49
#define R74            0x4A
#define R75            0x4B
#define R76            0x4C
#define R77            0x4D
#define R78            0x4E
#define R79            0x4F
#define R80            0x50
#define R81            0x51
#define R82            0x52
#define R83            0x53
#define R96            0x60
#define R97            0x61
#define R106           0x6A
#define R118           0x76
#define R128           0x80
#define R129           0x81
#define R130           0x82
#define R131           0x83
#define R132           0x84
#define R133           0x85
#define R134           0x86
#define R135           0x87
#define R136           0x88
#define R137           0x89
#define R139           0x8B
#define R140           0x8C
#define R141           0x8D
#define R143           0x8F
#define R144           0x90
#define R145           0x91
#define R146           0x92
#define R147           0x93
#define R148           0x94
#define R149           0x95
#define R150           0x96
#define R151           0x97
#define R152           0x98
#define R153           0x99
#define R154           0x9A
#define R157           0x9D
#define R192           0xC0
#define R193           0xC1
#define R229           0xE5							  		 
#endif  
	 
	 



