 
#include "IR_NTC.h"
#include "app_ad.h"
#include "app.h"
#include "app_uart.h" 
#include "stdio.h"
#include "ddl.h"

#include <math.h>
 
//#define TPS334  
#define WTTS001
 

float Forehead_Actual_code[]={//��ͷ�¶�
//	27.3  , 33.4,
//	27.4  ,	33.6,
//	27.5  ,	33.8,
//	27.8  ,	34.4,
////	27.7  ,	34.6,
//	27.9	,	34.8,
////	27.9  ,	34.9,
//	28.0  ,	34.9,
	
	28.1  ,	35.0,
	28.2  , 35.1,
	28.3  , 35.1,
	28.4  , 35.2,
	28.5  , 35.2,
	28.6  , 35.3,
	28.7  , 35.3,
	28.8  , 35.4,
	28.9  , 35.4,
	29.0  , 35.5,
	29.1  , 35.5,
	29.2  , 35.6,
  29.3  , 35.6,
	29.4  , 35.7,
	29.5  , 35.7,
	29.6  , 35.8,
	29.7  , 35.8,
	29.8  , 35.9,
	29.9  , 35.9,
	////////////////////////////////////////
	30.0  , 36.0,
	30.1  , 36.0,
	30.2  , 36.0,
	30.3  , 36.0,
	
	30.4  , 36.1,
	30.5  , 36.1,
	30.6  , 36.1,
	30.7  , 36.1,
	30.8  , 36.1,
	
	30.9  , 36.2,
	31.0  , 36.2,
	31.1  , 36.2,
	31.2	,	36.2,
	31.3	,	36.2,
	
	31.4	,	36.3,
	31.5	,	36.3,
	31.6	,	36.3,
	31.7	,	36.3,
	31.8	,	36.3,
	
	31.9	,	36.4,
	32.0	,	36.4,
	32.1	,	36.4,
	32.2	,	36.4,
	32.3	,	36.4,
	
	32.4	,	36.5,
	32.5	,	36.5,
	32.6	,	36.5,
	32.7	,	36.5,
	32.8	,	36.5,
	
	32.9	,	36.6,
	33.0	,	36.6,
	33.1	,	36.6,
	33.2	,	36.6,
	33.3	,	36.6,
	
	33.4	,	36.7,
	33.5	,	36.7,
	33.6	,	36.7,
	33.7	,	36.7,
	33.8	,	36.7,
	
	33.9	,	36.8,
	34		,	36.8,
	34.1	,	36.8,
	34.2	,	36.8,
	34.3	,	36.8,
	
	34.4	,	36.9,
	34.5	,	36.9,
	34.6	,	36.9,
	34.7	,	36.9,
	34.8	,	36.9,
	
	34.9	,	37.0,
	35.0	,	37.0,
	
	35.1	,	37.1,
	35.2	,	37.1,
	
	35.3	,	37.2,
	35.4	,	37.3,
	35.5	,	37.4,
	35.6	,	37.5,
	35.7	,	37.6,
	35.75	,	37.7,
	35.8	,	37.8,
	36		,	37.9,
	36.1	,	38.0,
	36.2	,	38.1,
	36.3	,	38.2,
	36.4	,	38.3,
	36.5	,	38.4,
	36.6	,	38.5,
	36.7	,	38.6,
	36.8	,	38.7,
	36.9	,	38.9,
	37		,	39.0,
	37.1	,	39.1,
	37.2	,	39.2,
	37.3	,	39.3,
	37.4	,	39.4,
	37.5	,	39.5,
	37.6	,	39.6,
	37.7	,	39.7,
	37.8	,	39.8,
	37.9	,	39.9,
	38		,	40.0,
	38.1	,	40.1,
	38.2	,	40.2,
	38.3	,	40.3,
	38.4	,	40.4,
	38.5	,	40.5,
	38.6	,	40.6,
	38.7	,	40.7,
	38.8	,	40.8,
	38.9	,	40.9,
	39		,	41.0,
	39.1	,	41.1,
	39.2	,	41.2,
	39.3	,	41.3,
	39.4	,	41.4,
	39.5	,	41.5,
	39.6	,	41.6,
	39.7	,	41.7,
	39.8	,	41.8,
	39.9	,	41.9,
	40		,	42.5,
	41    ,	42.9,
	42    , 43.5,
//	42.9  , 42.3,
};

 

/****************************************************************/ 
#ifdef  GT7750
//�����¶�0��C~45��C��Ӧ�ĵ�ѹֵ��v�� 31��C
float arr_VT[] = {
		2303,   //0
		2236,   //1
		2172,   //2
		2108,   //3
		2044,   //4
		1980,   //5
		1912,	 	//6
		1844,   //7
		1776,   //8
		1708,   //9
		1640,   //10
		1570,		//11
		1499,		//12
		1427,		//13
		1354,		//14
		1281,		//15
		1207,		//16
		1132,		//17
		1056,		//18
		980,		//19
		903,		//20
		825,		//21
		746,		//22
		666,		//23
		586,		//24
		505,		//25
		423,		//26
		340,		//27
		256,		//28
		172,		//29
		86,			//30
		0,			//31
		-87,		//32
		-175,		//33
		-264,		//34
		-354,		//35
		-444,		//36
		-536,		//37
		-628,		//38
		-721,		//39
		-815,		//40
		-912,		//41
		-1009,	//42
		-1106,	//43
		-1203,	//44
		-1300,	//45
};
		
float NTCRcode[] = {
/*����ֵ****�¶�ֵ****TS11***/
 
	198.8560,	// 10.00		 
	189.5862,	// 11.00		 
	180.8000,	// 12.00		 
	172.4696,	// 13.00		 
	164.5689,	// 14.00		 
	157.0735,	// 15.00		 
	149.9605,	// 16.00		 
	143.2084,	// 17.00		 
	136.7970,	// 18.00	 
	130.7073,	// 19.00		 
	124.9216,	// 20.00		 
	119.4230,	// 21.00		 
	114.1960,	// 22.00		 
	109.2256,	// 23.00		 
	104.4979,	// 24.00	 
	100.0000,	// 25.00	 
	95.7194,	// 26.00		 
	91.6446,	// 27.00		 
	87.7646,	// 28.00		 
	84.0692,	// 29.00		 
	80.5486,	// 30.00		 
	77.1937,	// 31.00		 
	73.9959,	// 32.00		 
	70.9470,	// 33.00	 
	68.0395,	// 34.00	
	65.2660,	// 35.00	 
	62.6197,	// 36.00		 
	60.0941,	// 37.00		 
	57.6832,	// 38.00		 
	55.3812,	// 39.00		 
	53.1827,	// 40.00		 
	51.0825,	// 41.00		 
	49.0757,	// 42.00		 
	47.1578,	// 43.00		 
	45.3244,	// 44.00		 
	43.5713,	// 45.00	 
	41.8947,	// 46.00		 
	40.2908,	// 47.00		 
	38.7563,	// 48.00	 
	37.2876,	// 49.00		 
	35.8818,	// 50.00		 
};
#endif


#ifdef  WTTS001  
//�����¶�0��C~45��C��Ӧ�ĵ�ѹֵ��v�� 
float arr_VT[] = {
		2659.242000,
		2591.693120,
		2519.130240,
		2456.360160,
		2398.458480,
		2326.646400,
		2252.772720,
		2191.682880,
		2126.560560,
		2054.425920,
		1977.250400,
		1913.876640,
		1835.539280,
		1759.098880,
		1675.614240,
		1595.017600,
		1513.334160,
		1437.530880,
		1364.629600,
		1287.658880,
		1211.598000,
		1125.473280,
		1044.242720,
		960.922560,
		880.506640,
		798.000000,
		712.398720,
		624.698880,
		540.907200,
		453.012480,
		352.985600,
		254.853440,
		175.686000,
		89.398720,
		0.000000,
		-94.526400,
		-177.084160,
		-265.774080,
		-353.555280,
		-451.512320,
		-540.502400,
		-633.626880,
		-734.930560,
		-834.326080,
		-936.863280,
		-1023.334400
		};

float NTCRcode[] = {
/*����ֵ****�¶�ֵ****TS11***/
 	325.806,	// 0.00	
	309.674 ,	// 1.00	
	294.436 ,	// 2.00	
	280.035 ,	// 3.00	
	266.422 ,	// 4.00	
	253.549 ,	// 5.00	
	241.372 ,	// 6.00	
	229.849 ,	// 7.00	
	218.942 ,	// 8.00	
	208.614 ,	// 9.00	
	198.832 ,	// 10.00
	189.563 ,	// 11.00
	180.778 ,	// 12.00
	172.449 ,	// 13.00
	164.55  ,	// 14.00
	157.056 ,	// 15.00
	149.945 ,	// 16.00
	143.195 ,	// 17.00
	136.785 ,	// 18.00
	130.698 ,	// 19.00
	124.914 ,	// 20.00
	119.417 ,	// 21.00
	114.191 ,	// 22.00
	109.223 ,	// 23.00
	104.497 ,	// 24.00
	100     , // 25.00	
	95.721  ,	// 26.00
	91.647  ,	// 27.00
	87.768  ,	// 28.00
	84.073  ,	// 29.00
	80.554  ,	// 30.00
	77.199  ,	// 31.00
	74.002  ,	// 32.00
	70.954  ,	// 33.00
	68.046  ,	// 34.00
	65.273  ,	// 35.00
	62.627  ,	// 36.00
	60.101  ,	// 37.00
	57.69   ,	// 38.00
	55.388  ,	// 39.00
	53.189  ,	// 40.00
	51.089  ,	// 41.00
	49.082  ,	// 42.00
	47.163  ,	// 43.00
	45.329  ,	// 44.00
	43.575  ,	// 45.00
	41.898  ,	// 46.00
	40.293  ,	// 47.00
	38.758  ,	// 48.00
	37.289  ,	// 49.00
	35.882  ,	// 50.00	 
};

#endif
 
/****************************************************************/
#ifdef  SGXV02
//�����¶�0��C~45��C��Ӧ�ĵ�ѹֵ��v�� 31��C
float arr_VT[] = {
		1876.1,
		1823.8,
		1771,
		1717.6,
		1663.6,
		1609.1,
		1554,
		1498.5,
		1442.4,
		1385.9,
		1328.8,
		1271.3,
		1213.3,
		1154.9,
		1096.1,
		1036.8,
		977.1,
		917,
		856.5,
		795.7,
		734.4,
		672.9,
		611,
		548.7,
		486.1,
		423.3,
		360.1,
		296.6,
		232.9,
		168.9,
		104.7,
		0,
		-024.5,
		-089.4,
		-154.5,
		-219.9,
		-285.3,
		-351,
		-416.8,
		-482.8,
		-548.8,
		-615.1,
		-681.4,
		-747.8,
		-814.3,
		-880.9
		};

float NTCRcode[] = {
/*����ֵ****�¶�ֵ****TS11***/
 
	195.29,	// 10.00		 
	186.484,	// 11.00		 
	178.117,	// 12.00		 
	170.165,	// 13.00		 
	162.605,	// 14.00		 
	155.416,	// 15.00		 
	148.577,	// 16.00		 
	142.068,	// 17.00		 
	135.874,	// 18.00	 
	129.976,	// 19.00		 
	124.358,	// 20.00		 
	119.008,	// 21.00		 
	113.909,	// 22.00		 
	109.049,	// 23.00		 
	104.417,	// 24.00	 
	100,	// 25.00	 
	95.786,	// 26.00		 
	91.767,	// 27.00		 
	87.932,	// 28.00		 
	84.272,	// 29.00		 
	80.779,	// 30.00		 
	77.443,	// 31.00		 
	74.258,	// 32.00		 
	71.215,	// 33.00	 
	68.309,	// 34.00	
	65.532,	// 35.00	 
	62.878,	// 36.00		 
	60.341,	// 37.00		 
	57.916,	// 38.00		 
	55.597,	// 39.00		 
	53.38,	// 40.00		 
	51.259,	// 41.00		 
	49.23,	// 42.00		 
	47.289,	// 43.00		 
	45.432,	// 44.00		 
	43.654,	// 45.00	 
	41.952,	// 46.00		 
	40.323,	// 47.00		 
	38.764,	// 48.00	 
	37.27,	// 49.00		 
	35.84,	// 50.00		 
};

#endif
/**********************************************************************************/

#ifdef  TPS334
//�����¶�0��C~45��C��Ӧ�ĵ�ѹֵ��v�� 
float arr_VT[] = {
		2659.242000,
		2591.693120,
		2519.130240,
		2456.360160,
		2398.458480,
		2326.646400,
		2252.772720,
		2191.682880,
		2126.560560,
		2054.425920,
		1977.250400,
		1913.876640,
		1835.539280,
		1759.098880,
		1675.614240,
		1595.017600,
		1513.334160,
		1437.530880,
		1364.629600,
		1287.658880,
		1211.598000,
		1125.473280,
		1044.242720,
		960.922560,
		880.506640,
		798.000000,
		712.398720,
		624.698880,
		540.907200,
		453.012480,
		352.985600,
		254.853440,
		175.686000,
		89.398720,
		0.000000,
		-94.526400,
		-177.084160,
		-265.774080,
		-353.555280,
		-451.512320,
		-540.502400,
		-633.626880,
		-734.930560,
		-834.326080,
		-936.863280,
		-1023.334400
		};

float NTCRcode[] = {
/*����ֵ****�¶�ֵ****TS11***/
	95.490  ,	// 0.00	
	91.338  ,	// 1.00	
	87.186  ,	// 2.00	
	83.034  ,	// 3.00	
	78.882  ,	// 4.00	
	74.730  ,	// 5.00	
	71.562  ,	// 6.00	
	68.394  ,	// 7.00	
	65.226  ,	// 8.00	
	62.058  ,	// 9.00	
	58.890  ,	// 10.00
	56.454  ,	// 11.00
	54.018  ,	// 12.00
	51.582  ,	// 13.00
	49.146  ,	// 14.00
	46.710  ,	// 15.00
	44.832  ,	// 16.00
	42.954  ,	// 17.00
	41.076  ,	// 18.00
	39.198  ,	// 19.00
	37.320  ,	// 20.00
	35.856  ,	// 21.00
	34.392  ,	// 22.00
	32.928  ,	// 23.00
	31.464  ,	// 24.00
	30.000  ,	// 25.00
	28.8498 ,	// 26.00
	27.6996 ,	// 27.00
	26.5494 ,	// 28.00
	25.3992 ,	// 29.00
	24.249  ,	// 30.00
	23.3424 ,	// 31.00
	22.4358 ,	// 32.00
	21.5292 ,	// 33.00
	20.6226 ,	// 34.00
	19.716  ,	// 35.00
	18.9966 ,	// 36.00
	18.2772 ,	// 37.00
	17.5578 ,	// 38.00
	16.8384 ,	// 39.00
	16.119  ,	// 40.00
	15.546  ,	// 41.00
	14.973  ,	// 42.00
	14.400  ,	// 43.00
	13.827  ,	// 44.00
	13.254  ,	// 45.00
	12.7932 ,	// 46.00
	12.3324 ,	// 47.00
	11.8716 ,	// 48.00
	11.4108 ,	// 49.00
	10.950  ,	// 50.00
 	 
};

#endif
/**********************************************************************************/

uint32_t NTC_cnt;   //PA01 ad_ntc
float F_ATCV;  // �����¶�У��ֵ
float NTC_TEMP;
float tempvalue;
double E= 2.718281828459045;
/******************************
����˵������ȡ�¶�ֵ
��ڲ�������

���ڲ�������
******************************/
void Get_Temp(void)
{
  float R_T,VDD;
	char xx;
	char AD_cnt=100;

	VDD = 3.0;
	get_ad();
	delay1ms(20);    
	while(AD_cnt--)
	{
		get_ad();
		NTC_cnt += u16AdcRestult1;  //pa01
		delay10us(1); 
	}
	NTC_cnt/=100;               //NTC����100K��2.5V
	R_T = (2.5*100/(2.5-(VDD * NTC_cnt/4096)))-100;  //��駵�ǰADֵ�����ǰ��ֵ
	
	xx=0;
	while(!(R_T > NTCRcode[xx]) && AD_cnt--)
	{
		xx++;
	}
	tempvalue = (xx-1) +  ( NTCRcode[xx-1] - R_T)/( NTCRcode[xx-1] - NTCRcode[xx]);
  tempvalue = tempvalue - 4;
 
}

double AIR_LMD(float ntc_Tamb)  //S������
{
	double S,S1,S2;
	double Tt,Ta;
	
 
	Tt   = 34;     //�������� //�����¶�Ϊ��ֵ 34
	Ta   = ntc_Tamb;     //�����¶�25�� //ʵ�ʵĻ����¶�

		//ʵ���¶�ȡ��
		int temp1 = (int)Ta;
		//ʵ���¶�ȡ����Ӧ��V
		double Tobj1 = arr_VT[temp1];
		 Tobj1/=1000000;
	
		//ʵ���¶�ȡ�����һ
		int temp2 = (int)Ta+1;
		//ʵ���¶�ȡ����Ӧ��V
		double Tobj2 = arr_VT[temp2];
	  Tobj2/=1000000;
	
		double b2 = 5*2.718281828459045;
		double b3_temp1 = 10*(1+2*2.718281828459045-3*temp1);
		double b3_temp2 = 10*(1+2*2.718281828459045-3*temp2);
		//sϵ��1
		S1 = Tobj1/(pow(Tt+273.15,4)-pow(temp1+273.15,4))/(b2-b3_temp1);
		//sϵ��2
		S2 = Tobj2/(pow(Tt+273.15,4)-pow(temp2+273.15,4))/(b2-b3_temp2);
		//�ֳ�10�ȷ�
		double num_avg = fabs((S2-S1))/10;
		//ȡС�����ּ���ȷ�
		double point = (Ta-temp1)*10;
  
		//����sֵ
		if(S1>S2) {
			 S  =  S1-num_avg*point;
		}else {
			 S  =  S1+num_avg*point;
		}
		
	return S;
}

void sort(int a[],int n)
{
	int i,j,temp;
	for(i=0;i<n-1;i++)                   //�Ƚ�n-1�Σ���һ��ѭ����ʾ������
	{   
		for(j=0;j<n-i-1;j++)              // ���һ�αȽ�a[n-i-1]��a[n-i-2]   ���ڶ���ѭ����ʾ�Ƚϴ��� ��
		{   
			if(a[j]<a[j+1])
			{   
				temp = a[j+1];
				a[j+1] = a[j];
				a[j] = temp;
			}
		}
	}
}

void sort_f(float a[],float n)
{
	int i,j;
	float	temp;
	for(i=0;i<n-1;i++)                   //�Ƚ�n-1�Σ���һ��ѭ����ʾ������
	{   
		for(j=0;j<n-i-1;j++)              // ���һ�αȽ�a[n-i-1]��a[n-i-2]   ���ڶ���ѭ����ʾ�Ƚϴ��� ��
		{   
			if(a[j]<a[j+1])
			{   
				temp = a[j+1];
				a[j+1] = a[j];
				a[j] = temp;
			}
		}
	}
}

//int bubble_sort(int *p) 
//{
//	int i,j,temp;
//	for(i=0;i<10;i++)
//	{   
//		for(j=0;j<9-i;j++)
//		{   
//			if(*(p+j)<*(p+j+1))
//			{   
//				temp = *(p+j);
//				*(p+j) = *(p+j+1);
//				*(p+j+1) = temp;
//			}   
//		}       
//	}   
//}


#define bats  470
 
int AIR_ADD = 0;
//�����¶Ȼ�ȡ   VOUT��VT���ĵ�ѹ  NTC�¶�
float Get_AIR(float ntc_Tamb)
{
	  int Temp[100];
		double Tobj;
		double S;
		double Vout;
		double AVDD=3.0;
		uint8_t AD_cnt=100;


		delay1ms(10);    
		while(AD_cnt--)
		{
			get_ad();
			Temp[AD_cnt] = u16AdcRestult0;  //pa00
		  delay10us(1);
		}
		sort(Temp,100);
		
//		for(AD_cnt=25;AD_cnt<75;AD_cnt++)
  	for(AD_cnt=0;AD_cnt<50;AD_cnt++)
		{
			AIR_ADD+=Temp[AD_cnt];
//			printf("Temp[%d] = %d  \r\n",AD_cnt,Temp[AD_cnt]);
//			delay10us(5); 
		}
		AIR_ADD /= 50;
		
//    AIR_ADD-=1680;  //old
		AIR_ADD-=1558;    //new

//		printf("-------->>AIR_ADD = %d \r\n",AIR_ADD);
		Vout = AVDD * AIR_ADD/4096;	 
		Vout /= bats;
//		printf("-------->>Vout = %f \t\r\n",Vout  ); 
		S = AIR_LMD(ntc_Tamb);  //  S ������
		Tobj= pow(Vout/(S*(5*E-(10*(1+2*E-3*ntc_Tamb))))+ pow(ntc_Tamb+273.15,4),0.25)-273.15;
//		printf("-------->>ntc_Tamb = %f \t\r\n",ntc_Tamb ); 
//		printf("-------->>Tobj = %3.2f \r\n",Tobj);
 
		return Tobj;
}




float H_Value,L_Value;
float H_Va=41,L_Va=35;
float Biao_Read_Temp(void)  //�����¶�   32---42
{
	  float Biao_Temp;
    Get_Temp();
	  Biao_Temp =  Get_AIR(tempvalue);
  	Biao_Temp += getAvg(tempvalue);
//	  printf("-----------Biao_Temp 1= %3.2f------------\r\n",Biao_Temp);
		L_Value = Read_ATCV(NTC_Addr);     //30
//		M_Value = Read_ATCV(NTC_Addr+2);
		H_Value = Read_ATCV(NTC_Addr+2);   //39
 
		if(tempvalue > 35)  //�������
		{
//			if(tempvalue > 37) ////�����¶�С��NTC�¶�
			{
//				printf("-------->>Biao_Temp=%3.2f  \r\n",Biao_Temp);
//				return Biao_Temp;
			}
		}
	  else if(tempvalue < 35)  //�����¶ȴ���NTC�¶�
		{
			  
        if(Biao_Temp>=L_Value && Biao_Temp<=H_Value) //35----41
				{
		//			printf("Biao_Temp 3= %3.2f  \r\n",Biao_Temp);
					Biao_Temp = L_Va + (Biao_Temp-L_Value)*((H_Va-L_Va)/(H_Value-L_Value)); 
				}
				else  if(Biao_Temp>tempvalue && Biao_Temp<L_Value) //NTC---30
				{
		//			printf("Biao_Temp 2= %3.2f  \r\n",Biao_Temp);  30    25          29.3    25
					Biao_Temp = tempvalue + (Biao_Temp-tempvalue)*((L_Va-tempvalue)/(L_Value-tempvalue));
				}
				else if(Biao_Temp>H_Value && Biao_Temp<=50) //41----50
				{
		//			printf("Biao_Temp 4= %3.2f  \r\n",Biao_Temp);
					//          41                  flash H    41   35     flash h  flashL
					Biao_Temp = H_Va + (Biao_Temp-H_Value)*((H_Va-L_Va)/(H_Value-L_Value)); 
//					Biao_Temp = H_Va + (Biao_Temp-H_Value)*6;
				} 
	  } 		
    Biao_Temp+=0.1; 
//		printf("--->>Biao_Temp=%3.2f--->L_Value=%4.3f--->H_Value=%4.3f  \r\n",Biao_Temp,L_Value,H_Value);
		return Biao_Temp;
}
 

 


float getAvg(float temp) 
{
	float temp_m;
  float tempH =  25;  
  //35�ȶ�Ӧ�ı����¶�33
  float tempH_v = 30.25 ;   
	
//  float tempH = 29.41; //40 c
//  //40�ȶ�Ӧ�ı����¶�36.7
//  float tempH_v = 29.84;  //

  float tempL = 10.05;    //  10 c
  //5�ȶ�Ӧ�ı����¶�36.7
  float tempL_v = 11.65;
  
  float avg = (tempH_v-tempL_v)/(tempH-tempL);  // 18.19/19.36  =  0.9395661157024793
	float num = 0.0;
  
	num = tempH- temp;
  
  temp_m = num*avg;
//	temp_m = num*0.2335;  //SC001 2020-5-6
	temp_m = 0.05;
  if(tempvalue>=20&&tempvalue<=30)
		temp_m = num*(0.05);
//	printf("-------->>temp_m=%3.2f\r\n",temp_m);
  return temp_m;
 }

 
float Actual_Read_Temp(float biao_Temp)  //���ݶ�ͷ�¶ȵõ�ʵ���¶�
{
	  float Actual_Temp;
	  char cnt;
	  cnt=0;
// 		biao_Temp = biao_Temp-0.1 ;
	  while(! (biao_Temp < Forehead_Actual_code[cnt*2]))
		{cnt++;}

		Actual_Temp = Forehead_Actual_code[cnt*2-1];
		Actual_Temp -= 0.2;
		return Actual_Temp;
}
/************************************************************************/




