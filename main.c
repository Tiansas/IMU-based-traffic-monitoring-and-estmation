#include "stm32f4xx.h"
/*-----------board.h-----------------*/
/*  */

#include "led.h"
#include "stdio.h"
#include "usart.h"
#include "imu.h"
#include "sdio_sdcard.h"
#include "exfuns.h" 
#include "GPS_decode.h"
#include "inttypes.h"

	
#include <stdlib.h>    //used for allocate memory for some Intermediate variables dynamically




extern short  ITG3205_X,ITG3205_Y,ITG3205_Z,ITG3205_T;
extern float ADXL345_X_AXIS,ADXL345_Y_AXIS;
extern float conver_x,conver_y,conver_z;
extern int16_t  HMC5883L_X,HMC5883L_Y,HMC5883L_Z;
extern float HMC5883L_ARC;
extern float pitch,roll,yaw;
extern nmeaINFO info;
extern nmeaGPVTG info_gpvtg;
extern nmeaGPRMC info_gprmc;
extern float Ra_g[3][3];

//variables needed for calibration
extern float u_x,u_y,u_z;
extern float v1_x,v1_y,v1_z,v2_x,v2_y,v2_z;
extern float inv_v1_x,inv_v1_y,inv_v1_z,inv_v2_x,inv_v2_y,inv_v2_z,inv_v3_x,inv_v3_y,inv_v3_z;	
extern float cal_ax,cal_ay,cal_az;

extern uint8_t rotation_matrix_created;

extern uint8_t TIM_SET_FALG; 

//char* concat(const char *s1, const char *s2);
void Change_BT_Name(char * name);
char file_name[20],read_buffer[10];
uint8_t AddSDstr[256];

// extern variables needed for estimation process

extern uint16_t test_count;
extern uint16_t N_test;
extern float Ax_cal[600],Ay_cal[600],Ax_new[600],Ay_m[600],Gz[600],HA_mag[600];
extern float delta_t;
extern int num_tu,num_stop,num_stop_turn; 
extern int Stop_Turn_po[10][2];
extern float Stop_Turn_sp[10];
extern int num_piece;
extern float HA_real[9],L_real[9];


//char* append(char * s, char c);
//char* concat(const char *s1, const char *s2);

///*******Test_Module_TYPE == Test_IMU*******/
void Module_setup(void)
{

	u32 total,free;
	Modules_Power_Control(ALL_Module,1);
   USART_INIT(115200,3,ENABLE);
	//serial port 2 initialize, baud rate 9600 ,enable receive interupt (bluebooth)
	 USART_INIT(9600,2,ENABLE);
	
	 //Change_BT_Name("tian\r\n");
	 /*uint8_t BTNamestr[256];
	 USART_Write_LEN("AT",strlen("AT"),2);	
	 delay_ms(1000);
	 USART_RECV_DATA_LEN(BTNamestr,2);
	 printf("%s",BTNamestr);
	USART_Write_LEN("AT+NAMEtian",strlen("AT+NAMEtian"),2);	
	 delay_ms(1000);
	USART_RECV_DATA_LEN(BTNamestr,2);
	 printf("%s",BTNamestr);*/
	
	//Initializes the IMU all modules
	GPS_Config();
	IMU_INIT(IMU_ALL_IC);
	LED_Init();
	TIM3_Int_Init(10000);            //seting a clock 0.1 sec, use function TIM3_TRQHandler(led.c) to controll


	if(SD_Init()!= SD_OK){
		printf("SD Init failed!\r\n");
	}
	else
	{
		printf("SD Init OK!\r\n");
	}
   exfuns_init();										 
   f_mount(fs[0],"0:",1); 					 

	if(exf_getfree((u8 *)"0",&total,&free))	
	{
		printf("SD Card Fatfs Error!\r\n");
	}	
	else{
		printf("SD Total Size:   %d  MB\r\n",total>>10);
		printf("SD  Free Size:   %d  MB\r\n",free>>10);
		sprintf(file_name,"data.txt");
			if(SD_Creat_File(file_name))
			{
				printf("create file success\r\n");
				sprintf((char *)AddSDstr,"Timecount,Hx,Hy,Hz,Gx,Gy,Gz,Ax,Ay,Az,Ax_cal,Ay_cal,Az_cal,Latitude,Longitude,Speed,Heading\r\n");
				Write_SD(file_name,AddSDstr,0xFFFFFFFFFFFFFFFF);		
			}
			else 	
				printf("create file Failed\r\n");

	}
} 

static uint8_t write_num=2;
static uint32_t file_num,i;
uint64_t timevalue = 0,gps_count=0,sd_count=0;  // BT_count=0;
static uint8_t led_status = 0 ,led_status3;
//#define WRITE_BT //uncomment to                                                     
void loop(void)
	{
		if(gps_count == 0){
			gps_count = TIM_GetCounter(TIM3);
		}
		if(sd_count == 0){
			sd_count = TIM_GetCounter(TIM3);
		}
		/* if (BT_count==0){
			BT_count=TIM_GetCounter(TIM3);
		} */ 
			//IMU sensor data using Bluetooth printing
			//Get ITG3205 data
			READ_ITG3205_XYZT();
			//Get ADXL345 data
			ADXL345_Read_XYZt();
			//Get HMC5883L data
			HMC5883L_Read_XYZt();	
		  
      /*
		  evaluate_rotation_matrix_imu();
			linear_filter();
	   	calculate_p_r_y(); */

		  cal_ax=inv_v1_x*conver_x+inv_v2_x*conver_y+inv_v3_x*conver_z;
		  cal_ay=inv_v1_y*conver_x+inv_v2_y*conver_y+inv_v3_y*conver_z;
		  cal_az=inv_v1_z*conver_x+inv_v2_z*conver_y+inv_v3_z*conver_z;
		
		  printf("Hx=%hd , Hy=%hd\r\n",HMC5883L_X,HMC5883L_Y);
	

		#ifdef WRITE_BT
			printf("ax=%f , ts=%"PRIu64"\r\n",conver_x,timevalue);
		#endif 
			timevalue = TIM_GetCounter(TIM3);
		if((timevalue - gps_count) >=10000)
		{
			GPS_Parse_Decode();
			gps_count = 0;
			#ifdef WRITE_BT
			printf("ts read GPS=%"PRIu64"£¬la=%f,\r\n",timevalue,info.lat);
			#endif
		}
		timevalue = TIM_GetCounter(TIM3);
    if((timevalue - sd_count) >=460)     //write SD every 0.05 sec, 20Hz Infact it's 0.1 sec, 1Hz
		{
		 sprintf((char *)AddSDstr,"%"PRIu64",%hd,%hd,%hd,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n ",\
			 timevalue,HMC5883L_X,HMC5883L_Y,HMC5883L_Z,ITG3205_X,ITG3205_Y,ITG3205_Z,conver_x,conver_y,conver_z,cal_ax,cal_ay,cal_az,info.lat,info.lon,info.speed,info.direction);
		 write_num=Write_SD(file_name,AddSDstr,0xFFFFFFFFFFFFFFFF);
		
		 	Estimation_process();
		 
			#ifdef WRITE_BT
		 printf("ts for writing =%"PRIu64" ,ax=%f, ay=%f, az=%f \r\n",timevalue,cal_ax,cal_ay,cal_az); 
		 #endif
		 	if(write_num==1){
						#ifdef WRITE_BT
			printf("write SD ok\r\n");
						 #endif 

		 (led_status3)?(led_status3=0):(led_status3=1);
		  setLED(3,led_status3);   //D3 blink when SD write OK
			 }
			sd_count = 0;
		 }
			//delay_ms(28);       // make sample rate 50Hz
	}
/*	
void Change_BT_Name(char * name)
{
	 uint8_t BTNamestr[256],length= 0x01;
	
//    char * name = "Tian";
//	char * new_name = append(name,ch);
   // char * full_name = concat("AT+NAME",name);
	 USART_Write_LEN("AT",strlen("AT"),2);	
	 delay_ms(1000);
	 USART_RECV_DATA_LEN(BTNamestr,2);
	 printf("%s\r\n",BTNamestr);
	 USART_Write_LEN("AT+NAMEname",strlen("AT+NAMEname"),2);	
	 delay_ms(1000);
	 USART_RECV_DATA_LEN(BTNamestr,2);                                                                                  
	 printf("%s\r\n",BTNamestr);
   //Receive data from bluetooth 
 	 //length = USART_RECV_DATA_LEN(BTNamestr,2);
	 //printf("%s",BTNamestr);
	 //send a string to bluetooth 
   //SART_Write_LEN("recv data",strlen("recv data"),3);
	 //send data to bluetooth
	 //USART_Write_LEN(BTNamestr,length,2);	
	 //delay_ms(1000);
}
*/

/*
char* append(char * s, char c){
	 uint8_t len = strlen(s);
   s[len] = c;
   s[len+1] = '\0';
	return s;
}
*/
char* concat(const char *s1, const char *s2)
{
    char *result = malloc(strlen(s1)+strlen(s2)+1);//+1 for the zero-terminator
    //in real code you would check for errors in malloc here
    strcpy(result, s1);
    strcat(result, s2);
    return result;
}

int main(void)
{
	//System initialization
  SystemInit();
	//clock initialization  System CLOCK:168MHZ  
  SysTick_Init(168);
	//Closed all module power
	Modules_Power_Control(ALL_Module,DISABLE);
	//Modules_Power_Control(IMU_Blue_Power,ENABLE);
	 //Modules_Power_Control(GPS_Power,ENABLE);
	//Module_setup();
	Module_setup();

	while(1)	
	{ 
	  loop();
	}
}

