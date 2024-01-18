#ifndef __DIGITAL_LED_H
#define __DIGITAL_LED_H

#include "tft_lcd.h"

typedef struct
{
//  unsigned char (*IF_DrawLine)(unsigned short xs,unsigned short ys,unsigned short xe,unsigned short ye);		//IO��ʼ������
	void (*IF_DrawLine)(unsigned short xs,unsigned short ys,unsigned short xe,unsigned short ye);		//IO��ʼ������
	void (*Digital_Draw_Line)(unsigned short xs,unsigned short ys,unsigned short xe,unsigned short ye,unsigned short color);
}_Draw_Str;

extern  unsigned short DIGITAL_COLOR_FRONT;
extern  unsigned short DIGITAL_COLOR_FRONT_EMP;
extern unsigned short DIGITAL_COLOR_BACK;
extern _Draw_Str draw_str;
extern unsigned short DIGITAL_POINT_COLOR;
 
/**
	*��ʼ����XY������ܶ��߿���ʾ��С��ߣ���ʾ������
  *mode ��bit7 �Ƿ��������ʾ�Σ�bit6 �Ƿ���丱��ʾ�Σ�bit5 �Ƿ���ʾ����ʾ��
	����
	Digital_Draw_num(20,20,75,440,760,6,0x90);
	��20��20λ����ʾ���Ϊ75��440��760�ߵ����������6
	*/
void Digital_Draw_num(unsigned short x,unsigned short y,unsigned char lw,unsigned short w,unsigned short h,unsigned char num,unsigned char mode);

void Draw_Line(unsigned short xs,unsigned short ys,unsigned short xe,unsigned short ye,unsigned short color)
{
	LCD_DrawLine(xs,ys,xe,ye,color);
}


void Draw_Digital_Num(unsigned short x,unsigned short y,unsigned short width,unsigned short height,unsigned int NUM)
{
	int ge,shi,bai,qian;
	static int ge2,shi2,bai2,qian2;
	qian = NUM/1000;
	if(qian!=qian2)
	{
		LCD_Fill(x,y, x +width,  y+height,LGRAY);
		qian2 = NUM/1000;
		Digital_Draw_num(x   ,y,12,width,height,  qian2 ,0x80);
	}
	
	bai = NUM%1000/100;
	if(bai!=bai2)
	{
		LCD_Fill(x+10+width*1,y, x+10+width*1 +width,  y+height,LGRAY);
		bai2 = NUM%1000/100;
		Digital_Draw_num(x+10+width*1,y,12,width,height,bai2,0x80);
	}
//	else
//	{
//		LCD_Fill(x+10+width*1,y, x+10+width*1 +width,  y+height,LGRAY);
//		Digital_Draw_num(x+10+width*1,y,12,width,height,bai,0x80);
//	}
	
	shi = NUM%1000%100/10;
	if(shi!=shi2)
	{
		LCD_Fill(x+20+width*2,y, x+20+width*2 +width,  y+height,LIGHTGRAY);
		shi2 = NUM%1000%100/10;
		Digital_Draw_num(x+20+width*2,y,12,width,height,shi2 ,0x80);
	}
//	else
//	{
//		LCD_Fill(x+20+width*2,y, x+20+width*2 +width,  y+height,LIGHTGRAY);
//		Digital_Draw_num(x+20+width*2,y,12,width,height,shi ,0x80);
//	}
	
  ge = NUM%1000%100%10;
	if(ge!=ge2)
	{
		LCD_Fill(x+30+width*3,y, x+30+width*3 +width,  y+height,LIGHTGRAY);
		ge2 = NUM%1000%100%10;
		Digital_Draw_num(x+30+width*3,y,12,width,height,ge2 ,0x80);	
	}
}


#endif
