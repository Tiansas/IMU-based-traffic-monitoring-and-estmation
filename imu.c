
 

#include "imu.h"
#include "retarget.h"
#include "math.h"
#include "float.h"

#include <stdlib.h>    //used for allocate memory for some Intermediate variables dynamically
// add by Tian
/*
#include <stdio.h> 
#include <malloc.h>
*/
/*
**Chip description: ITG3205: triple-axis MEMS gyroscope ADXL345: Digital Accelerometer
**HMC5883L: 3-Axis Digital Compass IC
*/
//
#define ACC_FREQUENCY 12.5 //12.5hz
#define ACC_SAMPLE_TIME 0.08 //.08 sec
#define LAMBDA 0.9
#define RADIANS_2_DEGREE 180/3.14159265
#define PIE 3.14159265

short  ITG3205_X,ITG3205_Y,ITG3205_Z,ITG3205_T;
int16_t  ADXL345_X,ADXL345_Y,ADXL345_Z;
int16_t  HMC5883L_X,HMC5883L_Y,HMC5883L_Z;
float ADXL345_X_AXIS,ADXL345_Y_AXIS,ADXL345_Z_AXIS;
float conver_x,conver_y,conver_z;
float HMC5883L_ARC;
float normal_mat[3];
float scal_prod;
float vec_prod[3];
float Ri[3][3];
float Rg[3][3];
float outMat[3][3];
float outMat2[3][3];
float outMat3[3][3];
uint8_t initialize = 0;
float pitch,roll,yaw,temp;

float Ra_g[3][3];
float preRa_g[3][3];
/////////////////////////  ITG3205
//初始化ITG3205  ITG3205 Initialize
void Init_ITG3205(void)
{	
  Write_single_reg(ITG3205_Addr,ITG3205_PWR_M,0x00);  
	Write_single_reg(ITG3205_Addr,ITG3205_SMPL,0x07);  //we need to change 0x07 to increase the sampling
	Write_single_reg(ITG3205_Addr,ITG3205_DLPF,0x1E); 
	Write_single_reg(ITG3205_Addr,ITG3205_INT_C,0x00);  
//	Write_single_reg(ITG3205_Addr,ITG3205_PWR_M,0x00);  
}
 

//Get ITG3205 X Y Z axis and temperature data
void READ_ITG3205_XYZT(void)
{
	uint8_t  data_l,data_h;
	//ITG3205 slave address
  data_l = Read_single_reg(ITG3205_Addr,ITG3205_WHO);
//	printf("ITG3205_Addr:%d\r\n ",data_l);
	data_l = Read_single_reg(ITG3205_Addr,ITG3205_GX_L);
  data_h = Read_single_reg(ITG3205_Addr,ITG3205_GX_H);
//	printf("ITG3205_XL:%d ,ITG3205_XH:%d \r\n ",data_l,data_h);
	ITG3205_X = (data_h<<8)|data_l;
	ITG3205_X /= 14.375;
	if(ITG3205_X<0)  ITG3205_X = ITG3205_X;
	data_l = Read_single_reg(ITG3205_Addr,ITG3205_GY_L);
  data_h = Read_single_reg(ITG3205_Addr,ITG3205_GY_H);
//	printf("ITG3205_YL:%d ,ITG3205_YH:%d \r\n ",data_l,data_h);
	ITG3205_Y = (data_h<<8)|data_l;
	ITG3205_Y /= 14.375;
	if(ITG3205_Y<0)  ITG3205_Y = ITG3205_Y;
	
	data_l = Read_single_reg(ITG3205_Addr,ITG3205_GZ_L);
  data_h = Read_single_reg(ITG3205_Addr,ITG3205_GZ_H);
//	printf("ITG3205_ZL:%d ,ITG3205_ZH:%d \r\n ",data_l,data_h);
	ITG3205_Z = (data_h<<8)|data_l;
	ITG3205_Z /= 14.375;
	if(ITG3205_Z<0)  ITG3205_Z = ITG3205_Z;
	
	data_l = Read_single_reg(ITG3205_Addr,ITG3205_TMP_L);
  data_h = Read_single_reg(ITG3205_Addr,ITG3205_TMP_H);
//	printf("ITG3205_TEMPL:%d ,ITG3205_TEMPH:%d \r\n ",data_l,data_h);
	ITG3205_T = (data_h<<8)|data_l;
	ITG3205_T =35+((double)ITG3205_T+13200)/280;
	if(ITG3205_T<0)  ITG3205_T = -ITG3205_T;
	temp = sqrt( pow(ITG3205_X,2)+pow(ITG3205_Y,2)+pow(ITG3205_Z,2));

//printf("Gx:%d,Gy:%d,Gz:%d, Norm:%f\r\n ",ITG3205_X,ITG3205_Y,ITG3205_Z,temp);
	 	
}
//////////////////////////// end ITG3205

/////////////////////////  ADXL345
//初始化ADXL345  ADXL345 initialize
void Init_ADXL345(void)
{
  Write_single_reg(ADXL345_Addr,ADXL345_DATA_FORMAT,0x0B);  //setting the range, here it is set at +-16g with 0x0B
	Write_single_reg(ADXL345_Addr,ADXL345_BW_RATE,0x08);  //12.5Hz with 0x08,set up the data rate or sampling frequency
	Write_single_reg(ADXL345_Addr,ADXL345_POWER,0x08);  
	Write_single_reg(ADXL345_Addr,ADXL345_INT_EN,0x80);  
	Write_single_reg(ADXL345_Addr,ADXL345_OFSX,0x00);  
	Write_single_reg(ADXL345_Addr,ADXL345_OFSY,0x00);  
	Write_single_reg(ADXL345_Addr,ADXL345_OFSZ,0x05);  
}

//Get ADXL345 data information
void ADXL345_Read_XYZt(void)
{
  uint8_t  data_l,data_h;
	float temp;
	//ADXL345 slave address
  data_l = Read_single_reg(ADXL345_Addr,ADXL345_ID);
//	printf("ADXL345_Addr:%d\r\n ",data_l);
	
	
	data_l = Read_single_reg(ADXL345_Addr,ADXL345_DATA_X0);
  data_h = Read_single_reg(ADXL345_Addr,ADXL345_DATA_X1);
//	printf("ADXL345_XL:%d ,ADXL345_XH:%d \r\n ",data_l,data_h);
	//sgnX = 0x8000 & data_h;
	ADXL345_X = ((data_h<<8)|data_l);
	
	data_l = Read_single_reg(ADXL345_Addr,ADXL345_DATA_Y0);
  data_h = Read_single_reg(ADXL345_Addr,ADXL345_DATA_Y1);
//	printf("ADXL345_YL:%d ,ADXL345_YH:%d \r\n ",data_l,data_h);
	//sgnY = 0x8000 & data_h;
	ADXL345_Y = ((data_h<<8)|data_l);
	
	data_l = Read_single_reg(ADXL345_Addr,ADXL345_DATA_Z0);
  data_h = Read_single_reg(ADXL345_Addr,ADXL345_DATA_Z1);
//	printf("ADXL345_ZL:%d ,ADXL345_ZH:%d \r\n ",data_l,data_h);
	//sgnZ = 0x8000 & data_h;
	//ADXL345_Z = 0x0000FFFF & ((data_h<<8)|data_l);
	ADXL345_Z = ((data_h<<8)|data_l);
//	printf("ADXL345_X:%x,ADXL345_Y:%x,ADXL345_Z:%x\r\n ",ADXL345_X,ADXL345_Y,ADXL345_Z);
  
//	conver_x=(float)ADXL345_X*4; //3.9 to 4
		conver_x=(float)ADXL345_X*3.9; //3.9 to 4
		conver_y=(float)ADXL345_Y*3.9; //3.9 to 4
		conver_z=(float)ADXL345_Z*3.9; //3.9 to 4


//	conver_y=ADXL345_Y*4;  //float to int
	//conver_z=ADXL345_Z*4;
  //获取ADXL345 angle  ADXL345 angle
	//printf("#%d$%d$%d$",ADXL345_X,ADXL345_Y,ADXL345_Z);
	


temp = sqrt( pow(conver_x,2)+pow(conver_y,2)+pow(conver_z,2));
	//printf("Ax:%f,Ay:%f,Az:%f,Norm:%f\r\n",conver_x,conver_y,conver_z,temp);//Actual Accel VAlues
// ADXL345_X_AXIS=(float)(((atan2(conver_z,conver_x)*180)/3.14159265)+180);    
  //ADXL345_Y_AXIS=(float)(((atan2(conver_z,conver_y)*180)/3.14159265)+180); 
//  printf("ADXL345_X_AXIS:%f,ADXL345_Y_AXIS:%f\r\n ",ADXL345_X_AXIS,ADXL345_Y_AXIS);
}	


//////////////////////////// end ADXL345


/////////////////////////  HMC5883L

//初始化HMC5883L  HMC5883L initialize
void Init_HMC5883L(void)
{
  Write_single_reg(HMC5883L_Addr,HMC5883L_configA,0x14);  
	Write_single_reg(HMC5883L_Addr,HMC5883L_MODE,0x00);  
}

void HMC5883L_Read_XYZt(void)
{
  uint8_t  data_l,data_h;
	
	//获取ITG3205 X轴  X_axle
	
	data_l = Read_single_reg(HMC5883L_Addr,HMC5883L_XL);
	data_h = Read_single_reg(HMC5883L_Addr,HMC5883L_XH);
	//	printf("HMC5883L_XL:%d ,HMC5883L_XH:%d \r\n ",data_l,data_h);
	HMC5883L_X = (data_h<<8)|data_l; 
	
	data_l = Read_single_reg(HMC5883L_Addr,HMC5883L_YL);
	data_h = Read_single_reg(HMC5883L_Addr,HMC5883L_YH);
	//	printf("HMC5883L_YL:%d ,HMC5883L_YH:%d \r\n ",data_l,data_h);
	HMC5883L_Y = (data_h<<8)|data_l;
	
	data_l = Read_single_reg(HMC5883L_Addr,HMC5883L_ZL);
	data_h = Read_single_reg(HMC5883L_Addr,HMC5883L_ZH);
//	printf("HMC5883L_ZL:%d ,HMC5883L_ZL:%d \r\n ",data_l,data_h);
	HMC5883L_Z = (data_h<<8)|data_l;
	//printf("Hx:%d,Hy:%d,Hz:%d\r\n ",HMC5883L_X,HMC5883L_Y,HMC5883L_Z);  
  
	if(HMC5883L_X>0x7fff) HMC5883L_X-=0xffff;	
	if(HMC5883L_Y>0x7fff) HMC5883L_Y-=0xffff;
	if(HMC5883L_Z>0x7fff) HMC5883L_Z-=0xffff;	
	
  HMC5883L_ARC=atan2(HMC5883L_Y,HMC5883L_X)*(180/3.14159265);    //Heading in degrees
  HMC5883L_ARC -= 4.033; // subtracting magnetic declination
	if(HMC5883L_ARC<0)
		HMC5883L_ARC += 360;
		
	if(HMC5883L_ARC > 360)
		HMC5883L_ARC -= 360 ;
	//printf("Heading:%f\r\n ",HMC5883L_ARC);
	//delay_ms();
	
}

//////////////////////////// end HMC5883L

/*Initialize the sensor module 
** imu_ic parameter value:  IMU_ITG3205、IMU_ADXL345、IMU_HMC5883L、IMU_ALL_IC
**  IMU_ITG3205 means ITG3205 initialize,IMU_ADXL345 means ADXL345 initialize
**  IMU_HMC5883L means HMC5883L initialize,IMU_ALL_IC means Three chips are initialized
*/
void IMU_INIT(uint8_t imu_ic)
{
	i2cInit();
	switch(imu_ic){
		case IMU_ITG3205: //ITG3205 initialize
			Init_ITG3205();
			break;
		case IMU_ADXL345: //ADXL345 initialize
			Init_ADXL345();
			break;
		case IMU_HMC5883L: //HMC5883L initialize
			Init_HMC5883L();
			break;
		case IMU_ALL_IC:  //All modules initialize
			Init_HMC5883L();
			Init_ADXL345();
			Init_ITG3205();
			break;
	}
}

//code for rotation matrix
 /*void IMU_EvaluateRotationMatrix(float a, float b, float c){
	 //get alpha, beta and gamma
	 
	 float Ra[3][3];
	 Ra[0][0] = (float)(cos(a)) * (float)(cos(b)) ;
	 Ra[0][1] = (float)cos(a)*(float)sin(b)*(float)sin(c) - (float)sin(a)*(float)cos(c);
	 Ra[0][2] = (float)cos(a)*(float)sin(b)*(float)cos(c) - (float)sin(a)*(float)sin(c);
	 Ra[1][0] = (float)sin(a)*(float)cos(b);
	 Ra[1][1] = (float)sin(a)*(float)sin(b)*(float)sin(c) + (float)cos(a)*(float)cos(c);;
	 Ra[1][2] = (float)sin(a)*(float)sin(b)*(float)cos(c) - (float)cos(a)*(float)sin(c);;
	 Ra[2][0] = (float)(-sin(b));
	 Ra[2][1] = (float)cos(b)*(float)sin(c);
	 Ra[2][2] = (float)cos(b)*(float)cos(c);
 }*/

/*code for evaluating rotation matrix in IMU frame of reference
the gyro component is calculated just as a cross product of the mag and acc data

*/
//float Ri[3][3];
void evaluate_rotation_matrix_imu(void){
	float ax,ay,az,hx,hy,hz,kx,ky,kz,temp_x,temp_y,temp_z, temp_hx,temp_hy,temp_hz;
	int i,j;
	//accelerometer vector
	
	normalize(conver_x/1000,conver_y/1000,conver_z/1000);
	 ax = normal_mat[0];
	 ay = normal_mat[1];
	 az = normal_mat[2];

	//magnetometer vector
	temp_hx = (float)(HMC5883L_X ) / 1000;
	temp_hy = (float)(HMC5883L_Y ) /1000;
	temp_hz = (float)(HMC5883L_Z ) /1000;
	//printf("BEFORE %f, %f, %f\r\n",temp_hx,temp_hy,temp_hz);

	scalar_product(conver_x/1000,conver_y/1000,conver_z/1000,temp_hx,temp_hy,temp_hz);
	//printf("SCALAR PRODUCT %f\r\n",scal_prod);	
	temp_x = scal_prod * conver_x/1000;
	 temp_y = scal_prod * conver_y/1000;
	 temp_z = scal_prod * conver_z/1000;
	//printf("TEMP %f, %f, %f\r\n",temp_x,temp_y,temp_z);
	
	hx = temp_hx - temp_x;
	 hy = temp_hy - temp_y;
	 hz = temp_hz - temp_z;
	//printf("AFTER %f, %f, %f\r\n",hx,hy,hz);
	
	normalize(hx,hy,hz);
	hx = normal_mat[0];
	hy = normal_mat[1];
	hz = normal_mat[2];
	//printf("AFTER NORMALIZE %f, %f, %f\r\n",hx,hy,hz);
	
	// vector perpendicular to both h and a vector
	vector_product(hx,hy,hz,ax,ay,az);
	 kx = vec_prod[0];
	 ky = vec_prod[1];
	 kz = vec_prod[2];
	
	normalize(kx,ky,kz);
	kx=normal_mat[0];
	ky=normal_mat[1];
	kz=normal_mat[2];
	
	Ri[0][0] = hx;
	Ri[1][0] = hy;
	Ri[2][0] = hz;
	
	Ri[0][1] = kx;
	Ri[1][1] = ky;
	Ri[2][1] = kz;
	
	Ri[0][2] = ax;
	Ri[1][2] = ay;
	Ri[2][2] = az;

//calculate RM for gyro or Rg
	Rg[0][0] = 1;
	Rg[1][0] = az*ACC_SAMPLE_TIME;
	Rg[2][0] = -1*ay*ACC_SAMPLE_TIME;
	
	Rg[0][1] = -1*az*ACC_SAMPLE_TIME;
	Rg[1][1] = 1;
	Rg[2][1] = ax*ACC_SAMPLE_TIME;
	
	Rg[0][2] = ay*ACC_SAMPLE_TIME;
	Rg[1][2] = -1*ax*ACC_SAMPLE_TIME;
	Rg[2][2] = 1;
	
	
//		if(initialize ==0 )	
//		{
//			for (i=0;i<3;i++)
//        for(j=0;j<3;j++)
//					preRa_g[i][j] = Ri[i][j] ;
//			initialize = 1;
//		}
//	printf("DCM Gyro Matrix\r\n ");
//	printf("%f, %f, %f\r\n",Rg[0][0],Rg[0][1],Rg[0][2]);
//	printf("%f, %f, %f\r\n",Rg[1][0],Rg[1][1],Rg[1][2]);
//	printf("%f, %f, %f\r\n",Rg[2][0],Rg[2][1],Rg[2][2]);
//	printf("%f, %f, %f\r\n",hx,kx,ax);
//	printf("%f, %f, %f\r\n",hy,ky,ay);
//	printf("%f, %f, %f\r\n",hz,kz,az);

}

void normalize(float vec_x,float vec_y,float vec_z){
	//float normal[3];
	float vec_mag;
	
	vec_mag = sqrt( pow(vec_x,2)+pow(vec_y,2)+pow(vec_z,2));
	//printf("VECMAG:%f\r\n ",vec_mag);
	normal_mat[0] = vec_x/vec_mag;
	normal_mat[1] = vec_y/vec_mag;
	normal_mat[2] = vec_z/vec_mag;
	//printf("Normalize\r\n ");
	//printf("%f, %f, %f, %f\r\n",vec_mag,normal_mat[0],normal_mat[1],normal_mat[2]);

}

void scalar_product(float vec_x1,float vec_y1,float vec_z1,float vec_x2,float vec_y2,float vec_z2){
	scal_prod = vec_x1*vec_x2 + vec_y1*vec_y2 + vec_z1*vec_z2;
}

void vector_product(float vec_x1,float vec_y1,float vec_z1,float vec_x2,float vec_y2,float vec_z2){
	vec_prod[0] = vec_y1*vec_z2 - vec_z1*vec_y2;
	vec_prod[1] = -vec_x1*vec_z2 + vec_z1*vec_x2;
	vec_prod[2] = vec_x1*vec_y2 - vec_y1*vec_x2;
//	printf("%f, %f, %f\r\n",vec_prod[0],vec_prod[1],vec_prod[2]);
}

//Linear Filter
void linear_filter(){
	int i,j;
	float temp[3][3];
	//float in1[3][3] = {{1,1,1},{1,1,1},{1,1,1}};
	//float in2[3][3] = {{1,1,1},{1,1,1},{1,1,1}};
	matrix_multiply(preRa_g,Rg); //answer in outMat2
//	for (i=0;i<3;i++){
//        for(j=0;j<3;j++){
//					temp[i][j] = outMat2[i][j];
//          //printf("%f\r\n",outMat[i][j]);
//        } 
//    }

	scalar_multiply(temp,LAMBDA);//answer is in outMat
	for (i=0;i<3;i++){
        for(j=0;j<3;j++){
					temp[i][j] = outMat[i][j];
          //printf("%f\r\n",outMat[i][j]);
        } 
    }

	scalar_multiply(Ri,1-LAMBDA);
    for (i=0;i<3;i++){
        for(j=0;j<3;j++){
					//preRa_g[i][j] = Ra_g[i][j];
					Ra_g[i][j] = temp[i][j] + outMat[i][j];
					preRa_g[i][j] = Ra_g[i][j];
					
			}
		}
//			printf("DCM after filter\r\n ");
//	printf("%f, %f, %f\r\n",Ra_g[0][0],Ra_g[0][1],Ra_g[0][2]);
//	printf("%f, %f, %f\r\n",Ra_g[1][0],Ra_g[1][1],Ra_g[1][2]);
//	printf("%f, %f, %f\r\n",Ra_g[2][0],Ra_g[2][1],Ra_g[2][2]);
//	printf("%f, %f, %f\r\n",outMat2[0][0],outMat2[0][1],outMat2[0][2]);
//	printf("%f, %f, %f\r\n",outMat2[1][0],outMat2[1][1],outMat2[1][2]);
//	printf("%f, %f, %f\r\n",outMat2[2][0],outMat2[2][1],outMat2[2][2]);

}

//Matrix Multiply
void matrix_multiply(float input1[3][3],float input2[3][3]){
	int i,j,k;
	float sum=0;
//    for (i=0;i<3;i++)
//        for(j=0;j<3;j++)
//					outMat2[i][j] = 0;
    for (i=0;i<3;i++){
        for(j=0;j<3;j++){
                for(k=0;k<3;k++){
                sum = sum + input1[i][k] * input2[k][j];
            } 
				//printf("%f\r\n",sum);
				outMat2[i][j] = sum;
				sum=0;			
        } 
    }
		//			printf("DCM after filter\r\n ");
//	printf("%f, %f, %f\r\n",outMat2[0][0],outMat2[0][1],outMat2[0][2]);
//	printf("%f, %f, %f\r\n",outMat2[1][0],outMat2[1][1],outMat2[1][2]);
//	printf("%f, %f, %f\r\n",outMat2[2][0],outMat2[2][1],outMat2[2][2]);
	

}

void scalar_multiply(float input[3][3] ,float lambda){
	int i,j;
	//float **out=0;
	for (i=0;i<3;i++){
        for(j=0;j<3;j++){
					outMat[i][j] = input[i][j] * lambda;
          //printf("%f\r\n",outMat[i][j]);
        } 
    }
	
}

void calculate_p_r_y(){
	float temp;
	yaw = atan2(Ra_g[1][0],Ra_g[0][0] )*RADIANS_2_DEGREE;
	roll = atan2(Ra_g[2][1],Ra_g[2][2])*RADIANS_2_DEGREE;
	temp = sqrt( pow(Ra_g[2][1],2)+pow(Ra_g[2][2],2));
	pitch = atan2(-1*Ra_g[2][0],temp)*RADIANS_2_DEGREE;
	//printf("pitch = %f, roll = %f, yaw = %f\r\n",pitch,roll,yaw);
}


//Inv of Roation matrix of Device GPS match to F150
/*float inv_v1_x=0.1101;
float inv_v1_y=0.9928;
float inv_v1_z=-0.0463;
float inv_v2_x=0.9837;
float inv_v2_y=-0.1022;
float inv_v2_z=0.1477;
float inv_v3_x=-0.01419;
float inv_v3_y=0.0618;
float inv_v3_z=0.9880;*/	

//Inv of Roation matrix of Device No Ca match to F150
/*float inv_v1_x=-0.20839;
float inv_v1_y=0.977926;
float inv_v1_z=-0.01527;
float inv_v2_x=0.967741;
float inv_v2_y=0.208432;
float inv_v2_z=0.141544;
float inv_v3_x=-0.141602;
float inv_v3_y=-0.014723;
float inv_v3_z=0.989815;	 */
/*
float offset_hx=0.6848;         //offset of magnetometer measurement
float offset_hy=-5.2363;
float offset_hz=-1.5532;
*/

//characteristic of device T in Tian's car
// rotation matrix from calibration 
float inv_v1_x=-0.9999;
float inv_v1_y=0.0129;
float inv_v1_z=0.0088;
float inv_v2_x=0.0031;
float inv_v2_y=0.7188;
float inv_v2_z=-0.6952;
float inv_v3_x=-0.0153;
float inv_v3_y=-0.6951;
float inv_v3_z=-0.7188;	
float cal_ax,cal_ay,cal_az;
// offset of gyroscope and magnetometer measurement
short offset_gx=-7;
short offset_gy=0;
short offset_gz=0;

float offset_hx=-2.7006;         //offset of magnetometer measurement
float offset_hy=3.3772;
float offset_hz=0.2013;
//add by Tian：Estimation process

uint16_t test_count=0;
uint16_t N_test=600;      //try three minute first
float delta_t=0.1;        //sample rate 10Hz
float Ax_cal[600],Ay_cal[600],Ax_new[600],Ay_m[600],Gz[600],HA_mag[600];
int num_turn=0,num_stop=0,num_stop_turn=0;
int total_num;
int Stop_Turn_po[10][2];
float Stop_Turn_sp[10];
int num_piece=0;
float HA_real[9],L_real[9];

float ax_sum=0,ay_sum=0,ax_avg=0,ay_avg=0;
int Start_num_ST=0,End_num_ST=0;

void Estimation_process()
{
	if (test_count<N_test)
	{
		//get the acceleration after calibration	
		Ax_cal[test_count]=cal_ax;
	  Ay_cal[test_count]=cal_ay;
		Gz[test_count]=ITG3205_Z;
		
		//heading angle estimation using magnetometer data 
		float h_x,h_y,h_z,angle1,angle2;
		  h_x=(float)HMC5883L_X/100-offset_hx;  
		  h_y=(float)HMC5883L_Y/100-offset_hy;
			h_z=(float)HMC5883L_Z/100-offset_hz;
			normalize(h_x,h_y,h_z);
			h_x=normal_mat[0];
			h_y=normal_mat[1];
			h_z=normal_mat[2];	
		  // heading angle with respect tot North, range:[-180 180]
			angle1=atan2(-h_y,h_x)*RADIANS_2_DEGREE+90;
			if(angle1>180){
				angle2=360-angle1;
			}
			else{
				angle2=-angle1;
			}
			// heading angle range [0 360]
			if (angle2<0){
				HA_mag[test_count]=360+angle2;
				}
			else{
				HA_mag[test_count]=angle2;
				}
		 test_count+=1;
		ax_sum +=cal_ax;
		ay_sum +=cal_ay;
	}
	else if(test_count==N_test)
	{
		// remove average for Ay_cal 
		ay_avg=ay_sum/test_count;
		for(int i=0;i<N_test;i++)
		{
			Ay_m[i]=Ay_cal[i]-ay_avg;
		}
		//remove avarage and low pass filter for Ax_cal 
		ax_avg=ax_sum/test_count;
		for(int i=0;i<N_test;i++)
		{
			float Ax_m[10];
			int k_count=0;
			float sum_Am_10=0;
			if (i<10) {
				for (int j=0;j<=i;j++)
				{
					Ax_m[j]=Ax_cal[i-k_count]-ax_avg;
					k_count++;
				}
				for (int j=i+1;j<10;j++)
				{
					Ax_m[j]=0;
				}
			}
			else {
				for (int j=0;j<10;j++)
				{
					Ax_m[j]=Ax_cal[i-k_count]-ax_avg;
					k_count++;
				}
			}
		  // low pass filter
			for (int k=0; k<10;k++) 
			{
				if (Ax_m[k]>2) {
					Ax_m[k]=2;
				}
				else if (Ax_m[k]<-2) {
					Ax_m[i]=-2;
				}
				sum_Am_10 +=Ax_m[k];
			}
			Ax_new[i]=1/10*sum_Am_10;
		}

		//initialize the Stop_Turn_po matrix with zero 
			for (int k=0;k<10;k++)
			{
				Stop_Turn_po[k][0]=0;
				Stop_Turn_po[k][1]=0;
				Stop_Turn_sp[k]=0;
			}
			//detection of stops and turns
			Turn_detection(Ay_m,Gz);
      Stop_detection(Ax_new);    
			End_num_ST=num_stop_turn-1;
			Rank_stop_turn();
			if (num_stop_turn==0){
				printf("No stops or turns are detected in this one minute\r\n");
				test_count=0;       //starts from the next 1 minute
			}
			else if (num_stop_turn==1){
				printf("Only one stop or turn is detected, can not do speed estimation considering the bias\r\n");
				test_count=0; 
			}
			else if (num_stop_turn>1){
				//Speed estimation 
				int start_esti=Stop_Turn_po[0][1];    //position of the end of the fist stop or turn, starts from 0 
				int end_esti=Stop_Turn_po[num_stop_turn-1][1];
				int num_parts=num_stop_turn-1;
				int length_V=Stop_Turn_po[num_stop_turn-1][1]-Stop_Turn_po[0][1];  
				float V_esti[length_V];
				int p=0;
				for (int k=0;k<num_parts;k++)  
				{  
					/*float v_1=Stop_Turn_sp[k];
					float v_2=Stop_Turn_sp[k+1];*/
					
					float v_start=Stop_Turn_sp[k];
					float v_end=Stop_Turn_sp[k+1];
					int length_part=Stop_Turn_po[k+1][1]-Stop_Turn_po[k][1];
					float Ax_new_test[length_part];
					int po_run=start_esti;
					for (int i=0;i<length_part;i++)
					{
						Ax_new_test[i]=Ax_new[po_run];
						po_run++;
					}
					// speed estimation for each part
					float V_esti_part[length_part];
					//Speed_estimation_optimization(Ax_new_test,length_part,v_1,v_2);
					int t_period=5;        //define one bias every 5 seconds
					int n_step=100;        // number of loops 100
					float alpha=0.1;       // learning rate
					int num_bias=(int) ceil(length_part*delta_t/t_period);    //ceil: Returns the smallest integer (double) value not less than x, so need to be changed to int
					int po_bias=(num_bias-1)*(t_period/delta_t);
					float sum_ax_test=0,para_a=0,para_b=0,para_c=0,func_pre=0,func_after=0;
					//initialize the bias with the value zero
					float D_bias[num_bias];
					for (int p=0;p<num_bias;p++)
					{
						D_bias[p]=0;
					}
					//get prepared for the optimization
					for (int i=0;i<length_part;i++)
					{
						sum_ax_test+=Ax_new_test[i];
					}
					para_a=sum_ax_test*delta_t;
					// the initial cost function
					float sum_d=0;
					func_pre=pow((v_start+para_a-v_end),2);
					// Updating the bias when there are more than one bias
					if (num_bias>1)
						{
							for (int i=1; i<n_step;i++)
							{
								para_b=v_start+para_a-t_period*sum_d-(length_part-po_bias)*delta_t*D_bias[num_bias-1];
								for (int j=0;j<(num_bias-1);j++)
								{
									para_c=D_bias[j]-alpha*2*para_b*(-t_period);
									D_bias[j]=para_c;
								}
								para_c=D_bias[num_bias-1]-alpha*2*para_b*(-(length_part-po_bias)*delta_t);
								D_bias[num_bias-1]=para_c;
								//get the updated sum_d and cost function
								for (int s=0;s<(num_bias-1);s++)
								{
									sum_d+=D_bias[s];
								}
								func_after=pow((v_start+para_a-t_period*sum_d-(length_part-po_bias)*delta_t*D_bias[num_bias-1]),2);
								// to see how is the update
								if (func_after>func_pre) {
										alpha=alpha*0.1;
									}
									if((func_pre-func_after)<0.001)
										break;
									func_pre=func_after;
							}
						}
						else 
							{
								for (int i=1; i<n_step;i++)
								{
									para_b=v_start+para_a-(length_part*delta_t)*D_bias[num_bias-1];
									para_c=D_bias[num_bias-1]-alpha*2*para_b*(-length_part*delta_t);
									D_bias[num_bias-1]=para_c;
									func_after=pow((v_start+para_a-(length_part*delta_t)*D_bias[num_bias-1]),2);
									if (func_after>func_pre) {
										alpha=alpha*0.1;
									}
									if((func_pre-func_after)<0.0001)
										break;
									func_pre=func_after;
								}
							}
							//speed estimation for this part based on the estimated bias 
							int c=0;   // position of bias
							int sum_a_now=0,sum_b_now=0,sum_bias=0;
							int po_first_bias,po_bias_start,po_bias_end;
							if (num_bias==1){
								for (int i=0;i<length_part;i++)
								{
									sum_a_now+=Ax_new_test[i];
									V_esti_part[i]=v_start+sum_a_now*delta_t-i*D_bias[c]*delta_t;
								}
							}
							else {
								while (c==0)
									{
										po_first_bias= (int)(t_period/delta_t);
										for (int i=0;i<po_first_bias;i++)
										{
											sum_b_now+=Ax_new_test[i];
											V_esti_part[i]=v_start+sum_b_now*delta_t-i*D_bias[c]*delta_t;
										}
										c++;
									}
								while ((c>0)&&(c<num_bias-1))
									{
										po_bias_start= (int)(c*t_period/delta_t+1);
										po_bias_end= (int)((c+1)*t_period/delta_t);
										for (int j=0;j<c;j++)
										{
											sum_bias+=D_bias[j];
										}
										for(int i=po_bias_start;i<po_bias_end;i++)
										{
											sum_b_now+=Ax_new_test[i];
											V_esti_part[i]=v_start+sum_b_now*delta_t-t_period*sum_bias-(i-(c-1)*t_period/delta_t)*D_bias[c];
										}
										c++; 
									}
									while (c==num_bias-1)
										{
											sum_bias+=D_bias[c-1];
											po_bias_start= (int)(c*t_period/delta_t+1);
											for (int i=po_bias_start;i<length_part;i++)
											{
												sum_b_now+=Ax_new_test[i];
												V_esti_part[i]=v_start+sum_b_now*delta_t-t_period*sum_bias-(i-(c-1)*t_period/delta_t)*D_bias[c];
											}
										}
							}		
							// store the estimated speed of each part to the whole estimation speed 
							for (int s=0;s<length_part;s++)
							{
								V_esti[p]=V_esti_part[s];
								p++;
							}
				}
				//compress the information to length and heading angle of each section 
				//get the poition along the route in X,Y coordinates
				float Po_X_Y[length_V][2];
				Po_X_Y[0][0]=0;
				Po_X_Y[0][1]=0;
				int q=start_esti;
				float HA_north;
				for (int i=1;i<length_V;i++)
				{
					if (HA_mag[q]>180){
						HA_north=HA_mag[q]-360;
					}
					else{
						HA_north=HA_mag[q];
					}
					Po_X_Y[i][0]=Po_X_Y[i-1][0]+V_esti[i]*delta_t*sin(HA_north*PIE/180);
					Po_X_Y[i][1]=Po_X_Y[i-1][1]+V_esti[i]*delta_t*cos(HA_north*PIE/180);
					q++;
				}
				//decide the number of pieces, the length and heading angle of each piece
				int po_stop_turn=0,num_piece=0,length_piece=0;
				int piece_start=start_esti, piece_end=end_esti;    //start_esti=Stop_Turn_po[0][1];end_esti=Stop_Turn_po[num_stop_turn-1][1];
				int Po_start=0,Po_end=0;
				float sum_HA=0;
				for (int i=1;i<num_stop_turn-1;i++)     //anyway starts from Stop_Turn_po[0][1], search from Stop_Turn_sp[1]
				{
					if (Stop_Turn_sp[i]!=0) {             // one turn 
						piece_end=Stop_Turn_po[i][1];
						length_piece=piece_end-piece_start;   //length of each piece
						for (int j=piece_start;j<piece_end;j++)
								{
									sum_HA+=HA_mag[j];
								}
						HA_real[num_piece]=sum_HA/length_piece;  //heading angle of this piece 
						Po_end=Po_start+length_piece;
						L_real[num_piece]=sqrt(pow((Po_X_Y[Po_end-1][0]-Po_X_Y[Po_start][0]),2)+pow((Po_X_Y[Po_end-1][1]-Po_X_Y[Po_start][1]),2));	
						num_piece++;
						piece_start=piece_end;
						Po_start=Po_end;
						sum_HA=0;
					}
				}
				// no matter the last one is turn or not, the last piece end at Stop_Turn_po[num_stop_turn-1][1]
				piece_end=Stop_Turn_po[num_stop_turn-1][1];
				length_piece=piece_end-piece_start;
				for (int j=piece_start;j<piece_end;j++)
				{
					sum_HA+=HA_mag[j];
				}
				HA_real[num_piece]=sum_HA/length_piece;
				Po_end=Po_start+length_piece;
				L_real[num_piece]=sqrt(pow((Po_X_Y[Po_end-1][0]-Po_X_Y[Po_start][0]),2)+pow((Po_X_Y[Po_end-1][1]-Po_X_Y[Po_start][1]),2));	
				num_piece++;
			}
			// start again from next one minute
			test_count=0;
			ax_sum=0;
			ay_sum=0;
			ax_avg=0;
			ay_avg=0;
		}
}
//Necessary functions for estimation 
/*
// Low pass filter for acceeration 
void Low_pass_filter(float Acc[600]){
	float sum_first_10=0,sum_others=0;
	for (int k=0;k<10;k++)
	{
		sum_first_10+=Acc[k];
		Ax_new[k]=1/10*sum_first_10;
	}
	for (int k=10;k<N_test;k++)
	{
		for (int count_filter=0;count_filter<10;count_filter++)
		{
			sum_others+=Acc[k-count_filter];
		}
		Ax_new[k]=1/10*sum_others;
	}
}
*/
				
// Turn detection
void Turn_detection(float A_y[600],float G_z[600]){
	float Angle_gyro[600];
	float temp_angle=0;
  Angle_gyro[0]=-G_z[0]*delta_t;
	for (int i=1;i<N_test;i++)
	{
		//move the noise of Gz
		if (abs(G_z[i])<3)
		{
			G_z[i]=0;
		}
    temp_angle=temp_angle-G_z[i]*delta_t;
		Angle_gyro[i]=temp_angle;
	}
	//get the number and position of turn
	int turn_flag=0;
	float diff1;
	int turn_start=0,turn_end=0;
	for (int j=0;j<(N_test-30);j++)
	{
		diff1=abs(Angle_gyro[j+30]-Angle_gyro[j]);
		if ((diff1>20)&&(turn_flag==0))
		{
			turn_flag=1;
			turn_start=j;	
		}
		if ((diff1<10)&&(turn_flag==1))
		{
			turn_flag=0;
			turn_end=j;
			Stop_Turn_po[num_stop_turn][0]=turn_start;
			Stop_Turn_po[num_stop_turn][1]=turn_end;
			num_turn+=1;
			num_stop_turn+=1;
		}
		if (num_stop_turn==10)
				break;
	}	
	//get the initial speed during turns 
	float V_turn_initial[600];
	for (int i=0;i<N_test;i++)
	{
		if(G_z[i]!=0)
			{
				V_turn_initial[i]=abs(A_y[i]/(G_z[i]*PIE/180));
			}
			else
			{
				V_turn_initial[i]=0;
			}
	}
	//get the speed during turns
	for (int k=0;k<num_turn;k++)
	{
		float sum_sp1=0,avg_sp1=0,length_turn=0,sum_sp2=0;
		length_turn=Stop_Turn_po[k][1]-Stop_Turn_po[k][0]+1;
		float Sp_remove=0;
		for (int i=Stop_Turn_po[k][0];i<=Stop_Turn_po[k][1];i++)
		{
			sum_sp1+=V_turn_initial[i];
		}
		avg_sp1=sum_sp1/length_turn;
		for (int j=Stop_Turn_po[k][0];j<=Stop_Turn_po[k][1];j++)
		{
			Sp_remove=V_turn_initial[j]-avg_sp1;
			sum_sp2+=Sp_remove;
		}
		Stop_Turn_sp[k]=sum_sp2/length_turn;
	}
}

/*
// Standard deviation of Ax_new every 1 second	
void Standard_deviation(float X[600]){
	    int n_for_std=10;
			float sum_1=0,avg_1=0,spow_1=0,sum_2=0,avg_2=0,spow_2=0,Std_output[N_test];
	    sum_1=X[0];
      for	(int j=1;j<10;j++)		
				{
					sum_1+=X[j];
					avg_1=sum_1/(j+1);
					for (int i=0;i<j;i++) 
					{
						spow_1+=(X[i]-avg_1)*(X[i]-avg_1);
					}
					Std_Ax[j]=sqrt(spow_1/(j+1));
				}					
			for (int j=10;j<N_test;j++)
				{
					for (int count_std=0;count_std<10;count_std++)
					{
						sum_2+=X[j-count_std];
					}
					avg_2=sum_2/10;
					for (int count_avg=0;count_avg<10;count_avg++)
					{
						spow_2+=(X[j-count_avg]-avg_2)*(X[j-count_avg]-avg_2);
					}
					Std_Ax[j]=sqrt(spow_2/10);
				}	
			}
*/
//Stop detection 
void Stop_detection(float A_x[600]){
	int stop_flag=0,num_meet=0,stop_start=0,stop_end=0;
	float Std_ax_now,ax_pre,ax_now,ax_aft;
	//get the number and position of stops
	for (int i=40;i<N_test-38;i++)
	{
		//get the Standard deviation every 1 second at this time step 
		int n_for_std=10;
		float sum_ax_new=0,avg_ax_new=0,spow_ax_new=0;
		for (int count_std=0;count_std<10;count_std++)
		{
			sum_ax_new+=A_x[i-count_std];
		}
		avg_ax_new=sum_ax_new/n_for_std;
		for (int count_avg=0;count_avg<10;count_avg++)
		{
			spow_ax_new+=(A_x[i-count_avg]-avg_ax_new)*(A_x[i-count_avg]-avg_ax_new);
		}
		Std_ax_now=sqrt(spow_ax_new/n_for_std);
		//prepare for stop detection
		ax_pre=average(A_x,i-40,i-1);
		ax_now=average(A_x,i,i+19);
		ax_aft=average(A_x,i+20,i+39);
		num_meet=find_limit(A_x,i,i+30,0.1);
		// Stop detection
		if((num_meet==30)&&(stop_flag==0)&&(ax_pre<=ax_now)&&(abs(ax_now)<=0.5))
		{
			stop_start=i;
			stop_flag=1;
		}
		if((stop_flag==1)&&(Std_ax_now>0.15)&&(ax_now<ax_aft))
		{
			if(ax_aft<-0.5)
			{
				stop_flag=0;
			}
			else
			{
				stop_end=i;
				stop_flag=0;
				Stop_Turn_po[num_stop_turn][0]=stop_start;
				Stop_Turn_po[num_stop_turn][1]=stop_end;
        Stop_Turn_sp[num_stop_turn]=0;
				num_stop+=1;
				num_stop_turn+=1;
			}
		}
		if (num_stop_turn==10)
				break;
	}
}
// compose and rank tops and turn
void Rank_stop_turn(){
	//rank the array based on the time 
	int temp1,temp2;
	float temp3;
	for (int i=0;i<num_stop_turn-1;i++)
	{
		for (int j=0;j<num_stop_turn-1-i;j++)
		{
			if (Stop_Turn_po[j][0]>Stop_Turn_po[j+1][0]){
				temp1=Stop_Turn_po[j][0];
				temp2=Stop_Turn_po[j][1];
				temp3=Stop_Turn_sp[j];
				Stop_Turn_po[j][0]=Stop_Turn_po[j+1][0];
				Stop_Turn_po[j][1]=Stop_Turn_po[j+1][1];
				Stop_Turn_sp[j]=Stop_Turn_sp[j+1];
				Stop_Turn_po[j+1][0]=temp1;
				Stop_Turn_po[j+1][1]=temp2;
				Stop_Turn_sp[j+1]=temp3;	
			}
		}
	}
	// eliminate overlap
	for (int k=0;k<num_stop_turn-1;k++)
	{
		if (Stop_Turn_po[k][1]>=Stop_Turn_po[k+1][0]){
			Stop_Turn_po[k][1]=Stop_Turn_po[k+1][0]-1;
		}
	}
}

//heading angle estimation using magnetometer data
/*
void Attitude_mag(float H_x[], float H_y[],float H_z[])
{
	float h_x,h_y,h_z,angle1,angle2;
	for (int i=0;i<N_test;i++)
	{
		h_x=H_x[i]-0.6848;
		h_y=H_y[i]+5.2363;
		h_z=H_z[i]+1.5532;
		normalize(h_x,h_y,h_z);
		h_x=normal_mat[0];
		h_y=normal_mat[1];
		h_z=normal_mat[2];	
		angle1=atan2(-h_y,h_x)*RADIANS_2_DEGREE+90;
		if(angle1>180){
			angle2=360-angle1;
			if(angle2<0){
				HA_mag[i]=360+angle2;
			}
		}
		else{
			angle2=-angle1;
			if(angle2<0){
				HA_mag[i]=360+angle2;
			}
	 	}
	}
}*/
//Speed estimation for one unit 
 /*void Speed_estimation_optimization(float Ax_test[], int num_part,float v_start,float v_end)
 {
	 int t_period=5;        //define one bias every 5 seconds
	 int n_step=100;        // number of loops 100
	 float alpha=0.1;       // learning rate
	 int num_bias=(int) ceil(num_part*delta_t/t_period);    //ceil: Returns the smallest integer (double) value not less than x, so need to be changed to int
	 int po_bias=(num_bias-1)*(t_period/delta_t);
	 float sum_acc=0,para_a=0,para_b=0,para_c=0,func_pre=0,func_after=0;
	 float d[num_bias];
	 float D_bias[num_bias];
	 for (int p=0;p<num_bias;p++)
	 {
		 D_bias[p]=0;
	 }
	 //
	 for (int i=0;i<num_part;i++)
	 {
		 sum_acc+=Ax_test[i];
	 }
	 para_a=sum_acc*delta_t;
	 // the initial cost function
	 float sum_d=0;
	 func_pre=pow((v_start+para_a-v_end),2);
	 // Updating the bias when there are more than one bias
	 if (num_bias>1)
	 {
		 for (int i=1; i<n_step;i++)
		 {
			 para_b=v_start+para_a-t_period*sum_d-(num_part-po_bias)*delta_t*D_bias[num_bias-1];
			 for (int j=0;j<(num_bias-1);j++)
			 {
				 para_c=D_bias[j]-alpha*2*para_b*(-t_period);
				 D_bias[j]=para_c;
			 }
			 para_c=D_bias[num_bias-1]-alpha*2*para_b*(-(num_part-po_bias)*delta_t);
			 D_bias[num_bias-1]=para_c;
			 //get the updated sum_d and cost function
			 sum_d=0;
			 for (int s=0;s<(num_bias-1);s++)
			 {
				 sum_d+=D_bias[s];
			 }
			 func_after=pow((v_start+para_a-t_period*sum_d-(num_part-po_bias)*delta_t*D_bias[num_bias-1]),2);
			 // to see how is the update
			 if (func_after>func_pre)
			 {
				 alpha=alpha*0.1;
			 }
			 if((func_pre-func_after)<0.001)
				 break;
			 func_pre=func_after;
		 }
	 }
	 else 
	 {
		 for (int i=1; i<n_step;i++)
		 {
			 para_b=v_start+para_a-(num_part*delta_t)*D_bias[num_bias-1];
			 para_c=D_bias[num_bias-1]-alpha*2*para_b*(-num_part*delta_t);
			 D_bias[num_bias-1]=para_c;
			 func_after=pow((v_start+para_a-(num_part*delta_t)*D_bias[num_bias-1]),2);
			 if (func_after>func_pre)
				 {
					 alpha=alpha*0.1;
				 }
			 if((func_pre-func_after)<0.0001)
				 break;
			 func_pre=func_after;
		 }
	 }
	 //speed estimation for this part
	 int c=0;   // k<num_bias
	 int sum_a_now=0,sum_b_now=0,sum_bias=0;
	 int po_first_bias,po_bias_start,po_bias_end;
	 if (num_bias==1){
		 for (int i=0;i<num_part;i++)
			 {
				 sum_a_now+=Ax_test[i];
				 V_esti_part[i]=v_start+sum_a_now*delta_t-i*D_bias[c]*delta_t;
			 }
	 }
	 else {
		 while (c==0)
		 {
			 po_first_bias= (int)(t_period/delta_t);
			 for (int i=0;i<po_first_bias;i++)
			 {
				 sum_b_now+=Ax_test[i];
				 V_esti_part[i]=v_start+sum_b_now*delta_t-i*D_bias[c]*delta_t;
			 }
			 c++;
		 }
		 while ((c>0)&&(c<num_bias-1))
			 {
				 po_bias_start= (int)(c*t_period/delta_t+1);
				 po_bias_end= (int)((c+1)*t_period/delta_t);
				 for (int j=0;j<c;j++)
				 {
					 sum_bias+=D_bias[j];
				 }
				 for(int i=po_bias_start;i<po_bias_end;i++)
				 {
					 sum_b_now+=Ax_test[i];
					 V_esti_part[i]=v_start+sum_b_now*delta_t-t_period*sum_bias-(i-(c-1)*t_period/delta_t)*D_bias[c];
				 }
				 c++; 
			 }
			 while (c==num_bias-1)
				 {
					 sum_bias+=D_bias[c-1];
					 po_bias_start= (int)(c*t_period/delta_t+1);
					 for (int i=po_bias_start;i<num_part;i++)
					 {
						 sum_b_now+=Ax_test[i];
						 V_esti_part[i]=v_start+sum_b_now*delta_t-t_period*sum_bias-(i-(c-1)*t_period/delta_t)*D_bias[c];
					 }
				 }
			 }			 
 }
 */
// Auxiliary functions
// function: get the average of som parameters in an array
float average(float array[],int start,int end)
{
	int num_array=sizeof(array)/sizeof(array[0]);
	float avg_part=0,sum_part=0;
	for (int k=start;k<=end;k++)
	{
		sum_part+=array[k];
	}
	avg_part=sum_part/(end-start+1);
	return avg_part;
}
//function: find the quantity of parameters under limit within a certain range of array
int find_limit(float X[],int start,int end,float limit)
{
	int num_limit=0;
	for (int k=start;k<=end;k++)
	{
		if (abs(X[k])<=limit)
		{
			num_limit+=1;
		}
	}
	return num_limit;
}
/*
// get a two dimensional space for row and column
void	*Get_space(int num_row,int (*tu_po)[2])	
{
	tu_po=(int(*)[2])malloc(num_row*2*sizeof(int));
} 
*/



//add by Tian
//uint16_t count = 0;
/*uint16_t cal_count = 0;
uint16_t cal_N_max  = 400;
//uint16_t fitted = 0;
float sum_a_x=0.0; 
float sum_a_y=0.0; 
float sum_a_z=0.0; 
float a_vg_x, a_vg_y, a_vg_z;
float u_x,u_y,u_z;
float v1_x,v1_y,v1_z,v2_x,v2_y,v2_z;
float inv_v1_x,inv_v1_y,inv_v1_z,inv_v2_x,inv_v2_y,inv_v2_z,inv_v3_x,inv_v3_y,inv_v3_z;	
 

    float a_for_cal[400][3];  //store the acceleration used for callibration under the condition that norm_g<=3  全局变量
		float r_for_fitting[400][3];  //store the residual r used for fitting
    float projection_rx_test[400],projection_ry_test[400];
		float X_for_direction[400];
		float cal_ax,cal_ay,cal_az;
		uint8_t rotation_matrix_created=0;*/	
		
		/*void Calibration_Process()
	{	
		if (cal_count<cal_N_max )
		{
			float norm_g=sqrt(pow(ITG3205_X,2)+pow(ITG3205_Y,2)+pow(ITG3205_Z,2));
		  if(norm_g<=3){
			  a_for_cal[cal_count][0]=conver_x;
			  a_for_cal[cal_count][1]=conver_y;
			  a_for_cal[cal_count][2]=conver_z;
			
			  cal_count+=1;
			
			  sum_a_x += conver_x;
			  sum_a_y += conver_y;
			  sum_a_z += conver_z;
	
        a_vg_x=sum_a_x/cal_count;
        a_vg_y=sum_a_y/cal_count;
	      a_vg_z=sum_a_z/cal_count;
			}	
		}
			
		else if(cal_count==cal_N_max )
			{
				float norm_a_vg;
				float vv1_x,vv1_y,vv1_z,vv2_x,vv2_y,vv2_z,vv1_x_u, vv1_y_u,vv1_z_u; //choose two points of R as a random vector vv1
				float sum_x=0.0;
        float sum_y=0.0;
        float sum_xy=0.0;
        float sum_x_squr=0.0;
        float mean_x,mean_y,a,b;
				float P1_x,P1_y,P1_z,P2_x,P2_y,P2_z;   //choose two points P1 and P2 on the line to get the vector v1
		    float P3_x,P3_y,P3_z,P4_x,P4_y,P4_z;    //project this two points to the initial reference frame

				float v1_x_try,v1_y_try,v1_z_try,v1_xx,v1_yy,v1_zz;

		    float X_neg=0.0,X_pos=0.0;
				uint8_t k=0;
		    //uint8_t t=0;
		    uint8_t c=0;
				//get vector u
				norm_a_vg=sqrt(pow(a_vg_x,2)+pow(a_vg_y,2)+pow(a_vg_z,2));
				u_x = a_vg_x/norm_a_vg;
				u_y = a_vg_y/norm_a_vg;
				u_z = a_vg_z/norm_a_vg;	
				
				//allocate danamic space for r_for_fitting   int * a = (int *) malloc (sizeof(int) * len);
				/*float *rx_for_fitting,*ry_for_fitting,*rz_for_fitting;
				rx_for_fitting=(float*)malloc(sizeof(float)*500);
				ry_for_fitting=(float*)malloc(sizeof(float)*500);
				rz_for_fitting=(float*)malloc(sizeof(float)*500);
				//get residual acceleration R
				for(int i=0;i<500;i++)
				{
					rx_for_fitting[i]=a_for_cal[i][0]-(a_for_cal[i][0]*u_x + a_for_cal[i][1]*u_y + a_for_cal[i][2]*u_z) * u_x;
					ry_for_fitting[i]=a_for_cal[i][1]-(a_for_cal[i][0]*u_x + a_for_cal[i][1]*u_y + a_for_cal[i][2]*u_z) * u_y;
					rx_for_fitting[i]=a_for_cal[i][2]-(a_for_cal[i][0]*u_x + a_for_cal[i][1]*u_y + a_for_cal[i][2]*u_z) * u_z;
				}*/
			/*
				for(int i=0;i<cal_N_max ;i++)
				{
				r_for_fitting[i][0]=a_for_cal[i][0]-(a_for_cal[i][0]*u_x + a_for_cal[i][1]*u_y + a_for_cal[i][2]*u_z) * u_x;
				r_for_fitting[i][1]=a_for_cal[i][1]-(a_for_cal[i][0]*u_x + a_for_cal[i][1]*u_y + a_for_cal[i][2]*u_z) * u_y;
				r_for_fitting[i][2]=a_for_cal[i][2]-(a_for_cal[i][0]*u_x + a_for_cal[i][1]*u_y + a_for_cal[i][2]*u_z) * u_z;
				}
				//all r is in the plane perpendicular to u vector, choose two points of R as a random vector vv1
				vv1_x=r_for_fitting[10][0]-r_for_fitting[0][0];
				vv1_y=r_for_fitting[10][1]-r_for_fitting[0][1];
				vv1_z=r_for_fitting[10][2]-r_for_fitting[0][2];
				float norm_vv1=sqrt(pow(vv1_x,2)+pow(vv1_y,2)+pow(vv1_z,2));
				vv1_x_u=vv1_x/norm_vv1;
				vv1_y_u=vv1_y/norm_vv1;
				vv1_z_u=vv1_z/norm_vv1;
				vv2_x=vv1_y_u*u_z-vv1_z_u*u_y;
				vv2_y=vv1_z_u*u_x-vv1_x_u*u_z;
				vv2_z=vv1_x_u*u_y-vv1_y_u*u_x;
				
				//get the coordinates of R in the reference frame built by vv1,vv2 and u
			
        //allocate space for projection_r_test
			  /*float *projection_rx_test,*projection_ry_test;
				projection_rz_test;
				projection_rx_test=(float*)malloc(sizeof(float)*500);
				projection_ry_test=(float*)malloc(sizeof(float)*500);
				//projection_rz_test=(float*)malloc(sizeof(float)*500);*/
				/*for(int j=0;j<cal_N_max ;j++)
				{
					projection_rx_test[j]=r_for_fitting[j][0]*vv1_x_u+r_for_fitting[j][1]*vv1_y_u+r_for_fitting[j][2]*vv1_z_u;
					projection_ry_test[j]=r_for_fitting[j][0]*vv2_x+r_for_fitting[j][1]*vv2_y+r_for_fitting[j][2]*vv2_z;
					//projection_rz_test[j]=rx_for_fitting[j]*u_x+ry_for_fitting[j]*u_y+rz_for_fitting[j]*u_z;
					
					//prepare for the fitting 
				  sum_x += projection_rx_test[j];
				  sum_y += projection_ry_test[j];
				  sum_xy += projection_rx_test[j]*projection_ry_test[j];
				  sum_x_squr += pow(projection_rx_test[j],2);
				}
					
				/*
				//free the space applied for projection_rx_test
				free(projection_rx_test);
				free(projection_ry_test);*/
				
				//the 2D fiiting y=a+bx
				/*mean_x = sum_x/cal_N_max ;
				mean_y = sum_y/cal_N_max ;
				a=(mean_y*sum_x_squr - mean_x*sum_xy)/(sum_x_squr - cal_N_max * pow(mean_x,2));
				b=(sum_xy - cal_N_max *mean_x*mean_y)/(sum_x_squr - cal_N_max * pow(mean_x,2));
				
				//get the first vector v1 based on fitting result
				
				P1_x=-10;P1_y=a+b*P1_x;P1_z=0;
				P2_x=10;P2_y=a+b*P2_x;P2_z=0;
				
				P3_x=vv1_x*P1_x+vv2_x*P1_y+u_x*P1_z;
				P3_y=vv1_y*P1_x+vv2_y*P1_y+u_y*P1_z;
				P3_z=vv1_z*P1_x+vv2_z*P1_y+u_z*P1_z;
				
				P4_x=vv1_x*P2_x+vv2_x*P2_y+u_x*P2_z;
				P4_y=vv1_y*P2_x+vv2_y*P2_y+u_y*P2_z;
				P4_z=vv1_z*P2_x+vv2_z*P2_y+u_z*P2_z;
				
				//decide the front and back of vector v1
				
				
					v1_x_try=P4_x-P3_x;
					v1_y_try=P4_y-P3_y;
					v1_z_try=P4_z-P3_z;
					float norm_v1=sqrt(pow(v1_x_try,2)+pow(v1_y_try,2)+pow(v1_z_try,2));
				  v1_xx=v1_x_try/norm_v1;
				  v1_yy=v1_y_try/norm_v1;
				  v1_zz=v1_z_try/norm_v1;
					
					//allacate space for X_for_direction dynamically
					 /*float *X_for_direction;
				   X_for_direction=(float*)malloc(sizeof(float)*500);*/
					/*for(int i=0;i<cal_N_max ;i++){ 
						X_for_direction[i]=r_for_fitting[i][0]*v1_xx+r_for_fitting[i][1]*v1_yy+r_for_fitting[i][2]*v1_zz;
					}
					 
					//对X从小到大进行排序
          for (int i=0; i<=cal_N_max-2; i++) {
						for (int j=i+1; j<=cal_N_max-1; j++) {
							if (X_for_direction[j]<X_for_direction[i]) {  
								c=X_for_direction[i];
								X_for_direction[i]=X_for_direction[j];
								X_for_direction[j]=c;
							}
						}
					}
					
					for (int i=0; i<cal_N_max ; i++) {
						if((X_for_direction[i]<500)&&(X_for_direction[i]>-500)){
							X_for_direction[i]=0;
						}
							
					}
					//计算最大和最小的二十个数的和
					for(int i=0;i<20;i++){
						X_neg=X_neg+X_for_direction[i];
						k=cal_N_max-1-i;
						X_pos=X_pos+X_for_direction[k];
					}
					//free the space for X_for_direction
					//free(X_for_direction);
	
					//X_neg=X_for_direction[0];
					//X_pos=X_for_direction[199];
					
				if(X_neg<0){
					X_neg=-X_neg;
				}
				
				if (X_neg>=X_pos){
					v1_x=v1_xx;
					v1_y=v1_yy;
					v1_z=v1_zz;	
				}
				else{
					v1_x=-v1_xx;
					v1_y=-v1_yy;
					v1_z=-v1_zz;	
				}
				
				v2_x=v1_y*u_z-v1_z*u_y;
				v2_y=v1_z*u_x-v1_x*u_z;
				v2_z=v1_x*u_y-v1_y*u_x;
				
				//get the inverse of V:inv_V=Adjoint_V/norm_V
				float norm_V= v1_x*(v2_y*u_z - v2_z*u_y) - v2_x*(v1_y*u_z - v1_z*u_y) + u_x * (v1_y*v2_z - v1_z*v2_y);
				
				inv_v1_x = (v2_y*u_z - v2_z*u_y)/norm_V;
				inv_v1_y = (v1_z*u_y - v1_y*u_z)/norm_V;
				inv_v1_z = (v1_y*v2_z - v1_z*v2_y)/norm_V;   //the first column
				
				inv_v2_x = (v2_z*u_x - v2_x*u_z)/norm_V;
				inv_v2_y = (v1_x*u_z - v1_z*u_x)/norm_V;
				inv_v2_z = (v1_z*v2_x - v1_x*v2_z)/norm_V;   //the second column
				
				inv_v3_x = (v2_x*u_y - v2_y*u_x)/norm_V;
				inv_v3_y = (v1_y*u_x - v1_x*u_y)/norm_V;
				inv_v3_z = (v1_x*v2_y - v1_y*v2_x)/norm_V;   //the third column
				
				rotation_matrix_created=1;

			}	
			
			/*if (rotation_matrix_created==1)
			{
				cal_ax=inv_v1_x*conver_x+inv_v2_x*conver_y+inv_v3_x*conver_z;
				cal_ay=inv_v1_y*conver_x+inv_v2_y*conver_y+inv_v3_y*conver_z;
				cal_az=inv_v1_z*conver_x+inv_v2_z*conver_y+inv_v3_z*conver_z;
			}
	}*/



	/*//We now start the process of obtaining attitude_rotation matrix
		
		//1. we get the normalized callibrated_a vector
		
		float norm_a = sqrt( pow(conver_x,2)+pow(conver_y,2)+pow(conver_z,2));
		
		float normalized_a_x = conver_x/norm_a;
		float normalized_a_y = conver_y/norm_a;
		float normalized_a_z = conver_z/norm_a;

		//2. we get the normalized projection of the magnetic field vector b on a plane
		//perpendicular to a
		
		float b_x = HMC5883L_X - (HMC5883L_X*normalized_a_x+HMC5883L_Y*normalized_a_y+HMC5883L_Z*normalized_a_z)*normalized_a_x;
		float b_y = HMC5883L_Y - (HMC5883L_X*normalized_a_x+HMC5883L_Y*normalized_a_y+HMC5883L_Z*normalized_a_z)*normalized_a_y;
		float b_z = HMC5883L_Z - (HMC5883L_X*normalized_a_x+HMC5883L_Y*normalized_a_y+HMC5883L_Z*normalized_a_z)*normalized_a_z;
		
		float norm_b = sqrt( pow(b_x,2)+pow(b_y,2)+pow(b_z,2));
		
		float normalized_b_x = b_x/norm_b;
		float normalized_b_y = b_y/norm_b;
		float normalized_b_z = b_z/norm_b;
		
		//3. we take the cross product of a and b to obtain c
		float c_x = normalized_a_y*normalized_b_z - normalized_b_y*normalized_a_z;
		float c_y = normalized_a_x*normalized_b_z - normalized_a_z*normalized_b_x;
		float c_z = normalized_a_x*normalized_b_y - normalized_a_y*normalized_b_x;
		
		
		
		R_ref[0][0] = normalized_a_x;
		R_ref[1][0] = normalized_a_y;
		R_ref[2][0] = normalized_a_z;
		R_ref[0][1] = normalized_b_x;
		R_ref[1][1] = normalized_b_y;
		R_ref[2][1] = normalized_b_z;
		R_ref[0][2] = c_x;
		R_ref[1][2] = c_y;
		R_ref[2][2] = c_z;
		
		//4. we obtain an elementary matrix
		R_elem[0][0] = 1;
		R_elem[1][0] = ITG3205_Z*delta_t;
		R_elem[2][0] = -ITG3205_Y*delta_t;
		R_elem[0][1] = -ITG3205_Z*delta_t;
		R_elem[1][1] = 1;
		R_elem[2][1] = ITG3205_X*delta_t;
		R_elem[0][2] = ITG3205_Y*delta_t;
		R_elem[1][2] = -ITG3205_X*delta_t;
		R_elem[2][2] = 1;
		
		// 5. We initialize R_t matrix only once for the first time
		if (R_t_initial == 0) {
			R_t_initial = 1;
			R_t[0][0] = 0;
			R_t[1][0] = 0;
			R_t[2][0] = 0;
			R_t[0][1] = 0;
			R_t[1][1] = 0;
			R_t[2][1] = 0;
			R_t[0][2] = 0;
			R_t[1][2] = 0;
			R_t[2][2] = 0;
		
		}
		
		
		//6. We now get the callibrated acceleration
		
		
		float temp_conver_x = conver_x - (R_t[0][0]*0 + R_t[0][1]*0 + R_t[0][2]*g);
		float temp_conver_y = conver_y - (R_t[1][0]*0 + R_t[1][1]*0 + R_t[1][2]*g);
		float temp_conver_z = conver_z - (R_t[2][0]*0 + R_t[2][1]*0 + R_t[2][2]*g);
		
		callibrated_a_x = inv_rot_matrix[0][0]*temp_conver_x + inv_rot_matrix[1][0]*temp_conver_y + inv_rot_matrix[2][0]*temp_conver_z;
		callibrated_a_y = inv_rot_matrix[1][0]*temp_conver_x + inv_rot_matrix[1][1]*temp_conver_y + inv_rot_matrix[1][2]*temp_conver_z;
		callibrated_a_z = inv_rot_matrix[2][0]*temp_conver_x + inv_rot_matrix[2][1]*temp_conver_y + inv_rot_matrix[2][2]*temp_conver_z;
		
		
		
		//7. We now make the update step on R_t with new values to use in the next iteration
		float rt1_1x = lambda*R_elem[0][0]*R_t[0][0] + lambda*R_elem[0][1]*R_t[1][0] + lambda*R_elem[0][2]*R_t[2][0];
		float rt1_1y = lambda*R_elem[1][0]*R_t[0][0] + lambda*R_elem[1][1]*R_t[1][0] + lambda*R_elem[1][2]*R_t[2][0];
		float rt1_1z = lambda*R_elem[2][0]*R_t[0][0] + lambda*R_elem[2][1]*R_t[1][0] + lambda*R_elem[2][2]*R_t[2][0];
			
		float rt1_2x = lambda*R_elem[0][0]*R_t[0][1] + lambda*R_elem[0][1]*R_t[1][1] + lambda*R_elem[0][2]*R_t[2][1];
		float rt1_2y = lambda*R_elem[1][0]*R_t[0][1] + lambda*R_elem[1][1]*R_t[1][1] + lambda*R_elem[1][2]*R_t[2][1];
		float rt1_2z = lambda*R_elem[2][0]*R_t[0][1] + lambda*R_elem[2][1]*R_t[1][1] + lambda*R_elem[2][2]*R_t[2][1];
			
		float rt1_3x = lambda*R_elem[0][0]*R_t[0][2] + lambda*R_elem[0][1]*R_t[1][2] + lambda*R_elem[0][2]*R_t[2][2];
		float rt1_3y = lambda*R_elem[1][0]*R_t[0][2] + lambda*R_elem[1][1]*R_t[1][2] + lambda*R_elem[1][2]*R_t[2][2];
		float rt1_3z = lambda*R_elem[2][0]*R_t[0][2] + lambda*R_elem[2][1]*R_t[1][2] + lambda*R_elem[2][2]*R_t[2][2];
		
		R_t[0][0] = rt1_1x + (1-lambda)*R_ref[0][0];
		R_t[1][0] = rt1_1y + (1-lambda)*R_ref[1][0];
		R_t[2][0] = rt1_1z + (1-lambda)*R_ref[2][0];
		
		R_t[0][1] = rt1_2x + (1-lambda)*R_ref[0][1];
		R_t[1][1] = rt1_2y + (1-lambda)*R_ref[1][1];
		R_t[2][1] = rt1_2z + (1-lambda)*R_ref[2][1];
		
		R_t[0][2] = rt1_3x + (1-lambda)*R_ref[0][2];
		R_t[1][2] = rt1_3y + (1-lambda)*R_ref[1][2];
		R_t[2][2] = rt1_3z + (1-lambda)*R_ref[2][2];
		
	}
	
	//printf("R: %f, %f and %f.\n", r_ca[0], r_ca[1], r_ca[2]);
} */ 