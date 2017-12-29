#ifndef _MPU6050_H
#define _MPU6050_H
#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
#include "r_cg_serial.h"


#define U8 unsigned char
#define U16 unsigned int
#define S16 int
#define S32	long
#define OK 0
#define ERROR 1


#define MPU6050_REG_SENSOR_DAT 0x3B////
#define	SMPLRT_DIV		0x19
#define	CONFIGL			0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	0x1C	
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	
#define	WHO_AM_I		0x75	
#define W_MPU6050_ADDRESS        0xD0 
#define R_MPU6050_ADDRESS        0xD1 


#define MPU6050_DLPF_BW_256         0x00

#define MPU6050_DLPF_BW_188         0x01

#define MPU6050_DLPF_BW_98          0x02

#define MPU6050_DLPF_BW_42          0x03

#define MPU6050_DLPF_BW_20          0x04

#define MPU6050_DLPF_BW_10          0x05

#define MPU6050_DLPF_BW_5           0x06




     
#define MPU6050_DLPF  MPU6050_DLPF_BW_42 


#define MPU6050_GYRO_FS_250         0x00

#define MPU6050_GYRO_FS_500         0x08

#define MPU6050_GYRO_FS_1000        0x10

#define MPU6050_GYRO_FS_2000        0x18

#define MPU6050_ACCEL_FS_2          0x01

#define MPU6050_ACCEL_FS_4          0x09

#define MPU6050_ACCEL_FS_8          0x11

#define MPU6050_ACCEL_FS_16         0x19

struct _float{
	      float x;
				float y;
				float z;};

struct _int16{
       int16_t x;
	     int16_t y;
	     int16_t z;};		

struct _trans{
     struct _int16 origin;  
	   struct _float averag;  
	   struct _float histor; 
	   struct _int16 quiet;  
	   struct _float radian; 
          };

struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
              };

extern struct _sensor sensor;	
extern U8	 mpu6050_buffer[14];	
uint8_t InitMPU6050(void);
void MPU6050_Read(void);
void MPU6050_Dataanl(void);
void UART1_ReportIMU(void);
void Gyro_OFFSET(void);
void dddd(void);
void kaijijiaozhun(void);
#endif
/* End user code. Do not edit comment generated here */

