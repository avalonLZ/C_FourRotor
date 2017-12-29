#ifndef __IMU_H
#define	__IMU_H
#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
#include "r_cg_serial.h"


#define RtA 	57.324841		
#define AtR    	0.0174533		
#define Acc_G 	0.0011963		
#define Gyro_G 	0.03051756	// 		
#define Gyro_Gr	0.0005327   // 

struct _angle{
        float pitch;
				float roll;
        float yaw;};


extern struct _angle angle;

void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
//void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az ,float mx, float my, float mz);




#endif





