#include "IMU.h"
#include "math.h"
#include "MPU6050.h"

struct _angle angle;

//float File_Buf[3][10];


#define FILTER_NUM 20
int ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];	


extern float X_HMC, Y_HMC, Z_HMC;
void Multiple_Read_HMC5883L(void);

#define KALMAN_Q        0.02
#define KALMAN_R        6.0000

static double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; 
   p_mid=p_last+Q; 
   kg=p_mid/(p_mid+R); 
   x_now=x_mid+kg*(ResrcData-x_mid);
                
   p_now=(1-kg)*p_mid;    
   p_last = p_now; 
   x_last = x_now; 
   return x_now;                
 }
static double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; 
   p_mid=p_last+Q; 
   kg=p_mid/(p_mid+R); 
   x_now=x_mid+kg*(ResrcData-x_mid);
                
   p_now=(1-kg)*p_mid;    
   p_last = p_now; 
   x_last = x_now; 
   return x_now;                
 }
static double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; 
   p_mid=p_last+Q;
   kg=p_mid/(p_mid+R); 
   x_now=x_mid+kg*(ResrcData-x_mid);
                
   p_now=(1-kg)*p_mid;  
   p_last = p_now; 
   x_last = x_now; 
   return x_now;                
 }

 

float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   
	return y;
} 

float FL_ABS(float x)
{
   if(x < 0)  return -x;
	 else return x; 
}

float COS(float x)
{
	float result;
  result = 1 - x * x/2;
	return result; 
}

float SIN(float y)
{
	float result;
  result = y - y * y * y /6;
	return result; 
}

float VariableParameter(float error)
{
	float  result = 0;
	
	if(error < 0)
	{
	   error = -error;
	}
  if(error >0.8f)
	{
	   error = 0.8f;
	}
	result = 1 - 1.28 * error;
	if(result < 0)
	{
	   result = 0;
	}
	return result;
}
/*************************************/

void Prepare_Data(void)
{
	/*static int num = 0;
	int i=0;
	float sum1=0,sum2=0,sum3=0;

	MPU6050_Dataanl();          
	//Multiple_Read_HMC5883L();   
	
	sensor.acc.averag.x = KalmanFilter_x(sensor.acc.origin.x,KALMAN_Q,KALMAN_R);  
	sensor.acc.averag.y = KalmanFilter_y(sensor.acc.origin.y,KALMAN_Q,KALMAN_R);  
	sensor.acc.averag.z = KalmanFilter_z(sensor.acc.origin.z,KALMAN_Q,KALMAN_R);  
	
	File_Buf[0][num] = sensor.gyro.radian.x;
	File_Buf[1][num] = sensor.gyro.radian.y;
	File_Buf[2][num] = sensor.gyro.radian.z;
	
	for(i=0;i<10;i++)
	{
     sum1 += File_Buf[0][i];
		 sum2 += File_Buf[1][i];
		 sum3 += File_Buf[2][i];
  }

	sensor.gyro.radian.x = sum1 / 10;
	sensor.gyro.radian.y = sum2 / 10;
	sensor.gyro.radian.z = sum3 / 10;
	
	num = (num + 1) % 10;*/
	
	static unsigned char filter_cnt=0;
	long temp1=0,temp2=0,temp3=0;
	unsigned char i;
	
	MPU6050_Dataanl();
	
	ACC_X_BUF[filter_cnt] = sensor.acc.origin.x;
	ACC_Y_BUF[filter_cnt] = sensor.acc.origin.y;
	ACC_Z_BUF[filter_cnt] = sensor.acc.origin.z;


	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	sensor.acc.averag.x = temp1 / FILTER_NUM;
	sensor.acc.averag.y = temp2 / FILTER_NUM;
	sensor.acc.averag.z = temp3 / FILTER_NUM;
	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;	
}


void Get_Attitude(void)
{	

	IMUupdate(sensor.gyro.radian.x,
		  sensor.gyro.radian.y,
		  sensor.gyro.radian.z,
		  sensor.acc.averag.x,
	          sensor.acc.averag.y,
	          sensor.acc.averag.z);
}




#define Kp 24.5f                      // 38.5proportional gain governs rate of convergence to accelerometer/magnetometer0.9f 
#define Ki 0.221f                     // integral gain governs rate of convergence of gyroscope biases0.0009f   
#define halfT 0.002f               
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
 
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
  float norm;
	int16_t Xr,Yr;
  float vx, vy, vz;// wx, wy, wz;
  float ex, ey, ez;


  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  //  float q0q3 = q0*q3;//
  float q1q1 = q1*q1;
  //  float q1q2 = q1*q2;//
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
	if(ax*ay*az==0)
 		return;
		
  norm = Q_rsqrt(ax*ax + ay*ay + az*az);      
  ax = ax *norm;
  ay = ay * norm;
  az = az * norm;

  // estimated direction of gravity and flux (v and w)            
  vx = 2*(q1q3 - q0q2);												
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           					
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + VariableParameter(ex) * ex * Ki;								 
  eyInt = eyInt + VariableParameter(ey) * ey * Ki;
  ezInt = ezInt + VariableParameter(ez) * ez * Ki;
// adjusted gyroscope measurements

  gx = gx + Kp *  VariableParameter(ex) * ex + exInt;	
	gy = gy + Kp *  VariableParameter(ey) * ey + eyInt;	
	gz = gz + Kp *  VariableParameter(ez) * ez + ezInt;	
  								
  // integrate quaternion rate and normalise	
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion
  norm = Q_rsqrt(q0q0 + q1q1 + q2q2 + q3q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;


	
 	 angle.roll = atan2(2*q2q3 + 2*q0q1, -2*q1q1 - 2*q2q2 + 1); // roll
	angle.pitch = asin(-2*q1q3 + 2*q0q2); // pitch
	
	
	Xr =0;// X_HMC * COS(angle.pitch/AtR) + Y_HMC * SIN(-angle.pitch/AtR) * SIN(-angle.roll/AtR) - Z_HMC * COS(angle.roll/AtR) * SIN(-angle.pitch/AtR);
	Yr =0;// Y_HMC * COS(angle.roll/AtR) + Z_HMC * SIN(-angle.roll/AtR);

	angle.yaw = atan2((double)Yr,(double)Xr) * RtA; // yaw 
	angle.roll *= RtA;
	angle.pitch *= RtA;
	
}	 













