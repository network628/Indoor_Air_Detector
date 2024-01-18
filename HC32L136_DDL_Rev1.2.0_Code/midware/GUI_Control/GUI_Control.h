#ifndef __GUI_CONTROL_H
#define __GUI_CONTROL_H
#include "stdint.h" 
#include "stdlib.h"

extern uint16_t KEY_Left;  //�� -- 1.2v
extern uint16_t KEY_Right;  //�� ++ 2.36v
extern char Radar;
extern uint8_t Sleep_flag;
extern float Air_T;           //�����¶�
extern float Air_Tm;           //�����¶�

void HomePage(void);
void Disp_Updata(void);
float Test_Temp(void);
void Disp_Max(void);
void Blue_Data(void);

#endif


