/*****************************************************************************/
/*      iic.C                                                          */
/*      MCU:RL78                                                        */
/*                                                     											*/
/*                                                                          */
/*****************************************************************************/

#pragma sfr
#pragma DI
#pragma EI
#pragma NOP
#pragma HALT
#pragma STOP
#include "mpu6050.h"
U8 iic_start(void)
{
	U8 wait=0;
   	IICAMK0 = 1U;	/* disable INTIICA0 interrupt */
	IICE0 = 1;		/*Enable IIC operation.*/
	WREL0 = 1;
	LREL0 = 1;		/*Exit from other communications*/
	IICAIF0=0;		/*Clear the IIC interrupt flag*/
	STT0= 1 ;		/*sent start commond*/
	while(!TRC0)
	{
		wait++;
		if(wait>100)return ERROR;
	}
	return OK;
}

U8 iic_rstart(void)
{
	U8 wait=0;
	STT0= 1 ;		/*sent start commond*/
	while(!TRC0)
	{
		wait++;
		if(wait>100)return ERROR;
	}
	return OK;
}

U8 iic_stop(void)
{
	U16 wait=0;
	SPT0=1;
	while(!SPD0)
	{
		wait++;
		if(wait>1000)return ERROR;
	}
	return OK;
}

U8 i2c_senddat(U8 dat)
{
	if(!TRC0)return ERROR;	/*if isn't the send mode retrurn error*/
	IICA0=dat;				/*send data*/
	while(!IICAIF0);			/*send end*/
	IICAIF0=0;
	if(IICA0!=dat)return ERROR;
	if(!ACKD0)return ERROR;
	return OK;
}

U8 i2c_recedat(void)
{
	if(TRC0)return ERROR;
//	if(mode!=1)
//	{
//		ACKE0=1;
//	}
//	else
//	{
		ACKE0=0;
	//}
	WREL0=1;
	while(!IICAIF0);
	IICAIF0=0;
	return IICA0;
}
U8 sel_mpu6050_reg(U8 reg)
{
	if(OK!=iic_start())return ERROR;
	if(OK!=i2c_senddat(W_MPU6050_ADDRESS))
	{
		iic_stop();
		return ERROR;
	};
	if(OK!=i2c_senddat(reg))
	{
		iic_stop();
		return ERROR;
	};
	return OK;
}
 U8 write_mpu6050(U8 reg,U8 *datbuf,U16 datl)
{
	if(OK!=sel_mpu6050_reg(reg))
	return ERROR;
	for(;datl!=0;datl--)
	{
		if(OK!=i2c_senddat(*datbuf))
		{
			iic_stop();
			return ERROR;
		};
		datbuf++;
	}
	return iic_stop();
}
 U8 read_mpu6050(U8 reg)
{       U8 DATA;
	if(OK!=sel_mpu6050_reg(reg))return ERROR;
	if(OK!=iic_rstart())return ERROR;
	if(OK!=i2c_senddat(R_MPU6050_ADDRESS))
	{
		iic_stop();
		return ERROR;
	};	
	DATA=i2c_recedat();	
	iic_stop();
	return DATA;
}