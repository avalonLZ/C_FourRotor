#include "mpu6050.h"
#include "IMU.h"

typedef struct PID{float P,pout,I,iout,D,dout,IMAX,OUT;}PID;
PID PID_ROL,PID_PIT,PID_YAW;
extern uint16_t youmen;

unsigned int MOTO1_PWM=0;
unsigned int MOTO2_PWM=0;
unsigned int MOTO3_PWM=0;
unsigned int MOTO4_PWM=0;

unsigned char yingji=0;


/*
void Pid_init(void)
{
	PID_ROL.P = 1.6;//0.7   1.4
	PID_ROL.I = 0;
	PID_ROL.D = 0.5;//0.19   0.19
	
	PID_PIT.P = 1.6;//0.7
	PID_PIT.I = 0;
	PID_PIT.D = 0.5;//0.19
	
	PID_YAW.P = 0;
	PID_YAW.I = 0;
	PID_YAW.D = 0.08;
	
	PID_ROL.pout = 0;
	PID_ROL.iout = 0;
	PID_ROL.dout = 0;
	
	PID_PIT.pout = 0;
	PID_PIT.iout = 0;
	PID_PIT.dout = 0;
	
	PID_YAW.pout = 0;
	PID_YAW.iout = 0;
	PID_YAW.dout = 0;
}


void CONTROL(float rol, float pit, float yaw)
{
	float moto1=0,moto2=0,moto3=0,moto4=0;
	
	PID_ROL.pout =PID_ROL.P * rol;
	PID_ROL.dout =-(PID_ROL.D * sensor.gyro.radian.x);
	
	PID_PIT.pout =PID_PIT.P * pit;
	PID_PIT.dout =-(PID_PIT.D * sensor.gyro.radian.y);
	
	PID_YAW.dout =-(PID_YAW.D * sensor.gyro.radian.z);
	
	PID_ROL.OUT = PID_ROL.pout + PID_ROL.iout + PID_ROL.dout;
	PID_PIT.OUT = PID_PIT.pout + PID_PIT.iout + PID_PIT.dout;
	PID_YAW.OUT = PID_YAW.pout + PID_YAW.iout + PID_YAW.dout;

	if(youmen>28800)///////////////////////////////////////////////////////////////////////////
	{
		moto1 = youmen/10.0 - 2880 - PID_ROL.OUT - PID_PIT.OUT + PID_YAW.OUT;
		moto2 = youmen/10.0 - 2880 + PID_ROL.OUT - PID_PIT.OUT - PID_YAW.OUT;
		moto3 = youmen/10.0 - 2880 + PID_ROL.OUT + PID_PIT.OUT + PID_YAW.OUT;
		moto4 = youmen/10.0 - 2880 - PID_ROL.OUT + PID_PIT.OUT - PID_YAW.OUT;
	}
	else
	{
		moto1 = 0;
		moto2 = 0;
		moto3 = 0;
		moto4 = 0;
	}
		
	MOTO1_PWM=(2880+moto1)*10.0;
	MOTO2_PWM=(2880+moto2)*10.0;
	MOTO3_PWM=(2880+moto3)*10.0;
	MOTO4_PWM=(2880+moto4)*10.0;
		
	if(MOTO1_PWM>62000) MOTO1_PWM=62000;
	if(MOTO2_PWM>62000) MOTO2_PWM=62000;
	if(MOTO3_PWM>62000) MOTO3_PWM=62000;
	if(MOTO4_PWM>62000) MOTO4_PWM=62000;
	
	if(MOTO1_PWM<25000) MOTO1_PWM=25000;
	if(MOTO2_PWM<25000) MOTO2_PWM=25000;
	if(MOTO3_PWM<25000) MOTO3_PWM=25000;
	if(MOTO4_PWM<25000) MOTO4_PWM=25000;
	
	if(yingji==1)
	{
		MOTO1_PWM=25000;
		MOTO2_PWM=25000;
		MOTO3_PWM=25000;
		MOTO4_PWM=25000;	
	}

	TDR01 = MOTO1_PWM;
	TDR02 = MOTO2_PWM;
	TDR03 = MOTO3_PWM;
	TDR04 = MOTO4_PWM;

}*/


struct _pid{
        float kp;
			  float ki;
	      float kd;
	      float increment;
	      float increment_max;
	      float kp_out;
			  float ki_out;
	      float kd_out;
	      float pid_out;
          };

struct _tache{
    struct _pid shell;
    struct _pid core;	
          };
	

struct _ctrl{
	unsigned char  ctrlRate;
      struct _tache pitch;    
	    struct _tache roll;  
	    struct _tache yaw;   
            };
	    
struct _ctrl ctrl;

void Pid_init(void)
{
	ctrl.pitch.shell.kp = 0.0;    //3.5
	ctrl.pitch.shell.ki = 0.0;//0.02
	ctrl.pitch.shell.kd =0.0;    //10	
	
	ctrl.pitch.core.kp = 0.45;   //0.45
	ctrl.pitch.core.kd = 0.09;  //0.04

	ctrl.roll.shell.kp = 0.0;
	ctrl.roll.shell.ki = 0.0;
	ctrl.roll.shell.kd = 0.0;

	ctrl.roll.core.kp =0.45;//0.405
	ctrl.roll.core.kd =0.09;//0.33

	ctrl.yaw.shell.kp =0.0;//3.5
	ctrl.yaw.shell.kd =0.0;//0.13
	
	ctrl.yaw.core.kp =0.0;//0.8
	ctrl.yaw.core.kd =0.0;//0.06
	
	ctrl.ctrlRate = 0.0;	
}






void CONTROL(float rol, float pit, float yaw)
{
	signed short int moto1=0,moto2=0,moto3=0,moto4=0;
	static float roll_old,pitch_old;
	if(ctrl.ctrlRate >= 2)
	{
		pit=pit - 0.0;// (float)(Rc_Data.PITCH - Rc_Data.pitch_offset)/15.0
		ctrl.pitch.shell.increment += pit;
		if(ctrl.pitch.shell.increment > 200)
				ctrl.pitch.shell.increment = 200;
		else if(ctrl.pitch.shell.increment < -200)
				ctrl.pitch.shell.increment = -200; 
				
		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * pit + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment + ctrl.pitch.shell.kd * (pit - pitch_old);		
		pitch_old = pit;
		/////////////////////////////////////////////////////////////////////////////////////
		rol=rol + 0.0;//(float)(Rc_Data.ROLL - Rc_Data.roll_offset)/15.0
		ctrl.roll.shell.increment += rol;
		if(ctrl.roll.shell.increment > 200)
				ctrl.roll.shell.increment = 200;
		else if(ctrl.roll.shell.increment < -200)
				ctrl.roll.shell.increment = -200;
		ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * rol + ctrl.roll.shell.ki * ctrl.roll.shell.increment + ctrl.roll.shell.kd * (rol - roll_old);
		roll_old = rol;
		//////////////////////////////////////////////////////////////////////////////////////////
		ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp *0.0  + ctrl.yaw.shell.kd * sensor.gyro.origin.z;//(float)(Rc_Data.YAW - Rc_Data.yaw_offset)/5.0
		 
		ctrl.ctrlRate = 0;		
	}
	ctrl.ctrlRate ++;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	ctrl.roll.core.kp_out = ctrl.roll.core.kp * (ctrl.roll.shell.pid_out + sensor.gyro.radian.y * RtA);
	ctrl.roll.core.kd_out = ctrl.roll.core.kd * (sensor.gyro.origin.y - sensor.gyro.histor.y);
	
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * (ctrl.pitch.shell.pid_out + sensor.gyro.radian.x * RtA);
	ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (sensor.gyro.origin.x - sensor.gyro.histor.x);
	
	ctrl.yaw.core.kp_out = ctrl.yaw.core.kp * (ctrl.yaw.shell.pid_out + sensor.gyro.radian.z * RtA);
	ctrl.yaw.core.kd_out = ctrl.yaw.core.kd * (sensor.gyro.origin.z - sensor.gyro.histor.z);
	
	ctrl.roll.core.pid_out = ctrl.roll.core.kp_out + ctrl.roll.core.kd_out;
	ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.kd_out;
	ctrl.yaw.core.pid_out =  ctrl.yaw.core.kp_out + ctrl.yaw.core.kd_out;
	
	sensor.gyro.histor.x = sensor.gyro.origin.x;
	sensor.gyro.histor.y = sensor.gyro.origin.y;
    	sensor.gyro.histor.z = sensor.gyro.origin.z;
	
	
	if(youmen>2880)///////////////////////////////////////////////////////////////////////////
	{///10.0
		moto1 = youmen/10.0 - 2880 - ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out - ctrl.yaw.core.pid_out;//- PID_ROL.OUT - PID_PIT.OUT + PID_YAW.OUT
		moto2 = youmen/10.0 - 2880 - ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out + ctrl.yaw.core.pid_out;//+ PID_ROL.OUT - PID_PIT.OUT - PID_YAW.OUT
		moto3 = youmen/10.0 - 2880 + ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out - ctrl.yaw.core.pid_out;//+ PID_ROL.OUT + PID_PIT.OUT + PID_YAW.OUT
		moto4 = youmen/10.0 - 2880 + ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out + ctrl.yaw.core.pid_out;//- PID_ROL.OUT + PID_PIT.OUT - PID_YAW.OUT
	}
	else
	{
		moto1 = 0;
		moto2 = 0;
		moto3 = 0;
		moto4 = 0;
	}
		
	MOTO1_PWM=(2880+moto1)*10.0;
	MOTO2_PWM=(2880+moto2)*10.0;
	MOTO3_PWM=(2880+moto3)*10.0;
	MOTO4_PWM=(2880+moto4)*10.0;/**/
		
	if(MOTO1_PWM>62000) MOTO1_PWM=62000;
	if(MOTO2_PWM>62000) MOTO2_PWM=62000;
	if(MOTO3_PWM>62000) MOTO3_PWM=62000;
	if(MOTO4_PWM>62000) MOTO4_PWM=62000;
	
	if(MOTO1_PWM<25000) MOTO1_PWM=25000;
	if(MOTO2_PWM<25000) MOTO2_PWM=25000;
	if(MOTO3_PWM<25000) MOTO3_PWM=25000;
	if(MOTO4_PWM<25000) MOTO4_PWM=25000;
	
	if(yingji==1)
	{
		MOTO1_PWM=25000;
		MOTO2_PWM=25000;
		MOTO3_PWM=25000;
		MOTO4_PWM=25000;	
	}
	
	TDR01 = MOTO1_PWM;
	TDR02 = MOTO2_PWM;
	TDR03 = MOTO3_PWM;
	TDR04 = MOTO4_PWM;

}


