

#ifndef     __IR_NTC_H__
#define     __IR_NTC_H__

//extern unsigned int Tem_Value,tempvalue;
extern float NTC_TEMP;
extern float H_Value,L_Value;// 环境温度校正值 
extern float tempvalue;

void Get_Temp(void);
float Get_AIR(float ntc_Tamb);
float getAvg(float temp);
float Actual_Read_Temp(float Forehead_Temp);  //根据额头温度得到实际温度
float Biao_Read_Temp(void);  //表面温度
void sort(int a[],int n);
void sort_f(float a[],float n);
#endif // end of __CONFIG_H__











