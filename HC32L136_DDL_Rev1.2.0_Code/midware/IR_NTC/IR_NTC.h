

#ifndef     __IR_NTC_H__
#define     __IR_NTC_H__

//extern unsigned int Tem_Value,tempvalue;
extern float NTC_TEMP;
extern float H_Value,L_Value;// �����¶�У��ֵ 
extern float tempvalue;

void Get_Temp(void);
float Get_AIR(float ntc_Tamb);
float getAvg(float temp);
float Actual_Read_Temp(float Forehead_Temp);  //���ݶ�ͷ�¶ȵõ�ʵ���¶�
float Biao_Read_Temp(void);  //�����¶�
void sort(int a[],int n);
void sort_f(float a[],float n);
#endif // end of __CONFIG_H__











