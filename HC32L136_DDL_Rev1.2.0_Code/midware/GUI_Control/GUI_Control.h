#ifndef __GUI_CONTROL_H
#define __GUI_CONTROL_H
#include "stdint.h" 
#include "stdlib.h"

extern uint16_t KEY_Left;  //左 -- 1.2v
extern uint16_t KEY_Right;  //右 ++ 2.36v
extern char Radar;
extern uint8_t Sleep_flag;
extern float Air_T;           //红外温度
extern float Air_Tm;           //红外温度

void HomePage(void);
void Disp_Updata(void);
float Test_Temp(void);
void Disp_Max(void);
void Blue_Data(void);

#endif


