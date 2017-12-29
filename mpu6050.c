#include "mpu6050.h"
#include "IMU.h"
extern  U8 write_mpu6050(U8 reg,U8 *datbuf,U16 datl);
extern  U8 read_mpu6050(U8 reg);

struct _sensor sensor;
U8	 mpu6050_buffer[14];	
 U8 test_mpu6050(void)
{
	U8 buff[2]={0,0};
	buff[0]=read_mpu6050(WHO_AM_I);
	buff[0]&=0x7e;
	if(buff[0]==0x68)return OK;
	return ERROR;
}
uint8_t InitMPU6050(void)
{
	uint8_t ack;
	
	ack =test_mpu6050();
	if (ack)
        return 1;
	
	write_mpu6050( PWR_MGMT_1,  (U8*)0x00,1);  	
	write_mpu6050( SMPLRT_DIV,  (U8*)0x07,1);     
	write_mpu6050( CONFIGL,     (U8*)MPU6050_DLPF,1);             
	write_mpu6050( GYRO_CONFIG, (U8*)MPU6050_GYRO_FS_1000,1);  
	write_mpu6050( ACCEL_CONFIG,(U8*)MPU6050_ACCEL_FS_4,1);  

	return 0;
}
 void get_mpu6050_dat_hK(void)
{
	mpu6050_buffer[0]=read_mpu6050( 0x3B);
	mpu6050_buffer[1]=read_mpu6050( 0x3C);
	mpu6050_buffer[2]=read_mpu6050(0x3D);
	mpu6050_buffer[3]=read_mpu6050( 0x3E);
	mpu6050_buffer[4]=read_mpu6050( 0x3F);
	mpu6050_buffer[5]=read_mpu6050(0x40);
	mpu6050_buffer[8]=read_mpu6050( 0x43);
	mpu6050_buffer[9]=read_mpu6050( 0x44);
	mpu6050_buffer[10]=read_mpu6050( 0x45);
	mpu6050_buffer[11]=read_mpu6050( 0x46);
	mpu6050_buffer[12]=read_mpu6050( 0x47);
	mpu6050_buffer[13]=read_mpu6050( 0x48);
}
void MPU6050_Dataanl(void)
{
	get_mpu6050_dat_hK();
	
	sensor.acc.origin.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - sensor.acc.quiet.x;
	sensor.acc.origin.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - sensor.acc.quiet.y;
	sensor.acc.origin.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);

	sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
	
	sensor.gyro.radian.x = (sensor.gyro.origin.x - sensor.gyro.quiet.x)* Gyro_Gr;//* Gyro_Gr
	sensor.gyro.radian.y = (sensor.gyro.origin.y - sensor.gyro.quiet.y)* Gyro_Gr;
	sensor.gyro.radian.z = (sensor.gyro.origin.z - sensor.gyro.quiet.z)* Gyro_Gr;
}

void kaijijiaozhun(void)
{

   int cnt_g=2000;
	 int32_t  tempgx=0,tempgy=0,tempgz=0;
	 int32_t  tempax=0,tempay=0;
	 int16_t  accx=0,accy=0;
	 
	/* sensor.gyro.averag.x=0;
	 sensor.gyro.averag.y=0;  
	 sensor.gyro.averag.z=0;
	*/
		 
	 while(cnt_g--)
	 {
	get_mpu6050_dat_hK();
	
	accx = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]);
	accy = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]);
	
	sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
	
      	tempgx+= sensor.gyro.origin.x;
	tempgy+= sensor.gyro.origin.y;
	tempgz+= sensor.gyro.origin.z;
	
	tempax+= accx;
	tempay+= accy;
	}
	 sensor.gyro.quiet.x=tempgx/2000;
	 sensor.gyro.quiet.y=tempgy/2000;
	 sensor.gyro.quiet.z=tempgz/2000;
	 
	 sensor.acc.quiet.x=tempax/2000;//;614
	 sensor.acc.quiet.y=tempay/2000;//0-233;
}
