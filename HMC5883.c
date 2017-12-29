#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
#include "r_cg_serial.h"
#define SALVE_ADDR 0x3c
#define  FILL_NUM  10
#define FILTER_NUM 10
typedef struct HMC5583LData
{
	uint8_t XMSB;
	uint8_t XLSB;
	uint8_t YMSB;
	uint8_t YLSB;
	uint8_t ZMSB;
	uint8_t ZLSB;
}HSD;
//uint8_t Data[6]={0};
int16_t  x,y,z;
float X_HMC, Y_HMC, Z_HMC, MAG_AVGx, MAG_AVGy, MAG_AVGz;
float MAG_X_BUF[FILTER_NUM],MAG_Y_BUF[FILTER_NUM],MAG_Z_BUF[FILTER_NUM];





void delayLittle(uint16_t Tim)
{
	uint16_t delayNum=2000;
	while(--Tim)
		while(--delayNum)
			NOP();
}

uint8_t readRegisterValue(uint8_t registerNum)
{
	uint16_t cnt=0;
	uint8_t registerValue=0;
	
	R_IIC10_Master_Send(SALVE_ADDR, &registerNum, 1);
	delayLittle(2);
	R_IIC10_Master_Receive(SALVE_ADDR+1, &registerValue, 1);
	delayLittle(2);
	return registerValue; 	
}

void writeRegister(uint8_t registerNum, uint8_t registerValue)
{
	uint8_t val[2]={0};
	uint16_t cnt=0;
	val[0]=registerNum;
	val[1]=registerValue;
	
	R_IIC10_Master_Send(SALVE_ADDR, val, 2);
	delayLittle(2);
}
void initMMC5883L()
{
	writeRegister(0x02, 0x00);
}

void Multiple_Read_HMC5883L(void)
{	
	uint8_t i;
	uint8_t registerNum = 0x03;
	//HSD Data;
	uint8_t ch[8] = {0};
	static uint8_t filter_cnt1=0;
	float temp1=0,temp2=0,temp3=0;
	
	R_IIC10_Master_Send(SALVE_ADDR, &registerNum, 1);
	delayLittle(2);
	R_IIC10_Master_Receive(SALVE_ADDR+1, ch, 6);
	delayLittle(2);
	
	/*Data.XMSB = ch[0]; 
	Data.XLSB = ch[1];
	Data.YMSB = ch[2];
	Data.YLSB = ch[3];
	Data.ZMSB = ch[4];
	Data.ZLSB = ch[5];*/
	
	x = ch[0] << 8 | ch[1];
	y = ch[4] << 8 | ch[5];
	z = ch[2] << 8 | ch[3];
	
	X_HMC = (float)(x-258) ;
	Y_HMC = (float)y*1.005256+78.912596;
	Z_HMC = (float)z*1.094421+1.641632 ;
	
	temp1 = temp2 = temp3 = 0;	
	MAG_X_BUF[filter_cnt1] = X_HMC;
	MAG_Y_BUF[filter_cnt1] = Y_HMC;
	MAG_Z_BUF[filter_cnt1] = Z_HMC;
	
	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += MAG_X_BUF[i];
		temp2 += MAG_Y_BUF[i];
		temp3 += MAG_Z_BUF[i];
	}
	MAG_AVGx = temp1 / FILTER_NUM;
	MAG_AVGy = temp2 / FILTER_NUM;
	MAG_AVGz = temp3 / FILTER_NUM;
	
	filter_cnt1++;
	if(filter_cnt1==FILTER_NUM)	
	filter_cnt1=0;
}