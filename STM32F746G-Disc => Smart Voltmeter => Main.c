/* USER CODE BEGIN Header */
/*******************************************************************************
  Created By: Ahmad Alvi Syahrin
						  PENS 2017
	Modified By : Andrean Rangga Kusuma
							PENS 2018
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
 *******************************************************************************
*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "math.h"
#include <stdio.h>
#include "stdbool.h"
#include "string.h"
#include "stm32f7xx_hal_sdram.h"
#include "stm32f7xx_hal_ltdc.h"
#include "stm32f7xx_ll_fmc.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_sdram.h"
#include "lambangpens.h"
#include "profilku.h"
#include "mainmenuku.h"
//#include "wallpaper.h"
#include "calibrationed.h"
#include "voltagewaveform1.h"
#include "voltagewaveform2.h"
#include "voltagewaveform3.h"
#include "voltagewaveform4.h"
#include "condition.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int hour, min, sec,msec;
int day, month, year;
char rtc [100];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//int key;
//key = 10000;
#define sampling_kal 1000 //moving average
#define acdc 20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

LTDC_HandleTypeDef hltdc;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

DMA_HandleTypeDef hdma_memtomem_dma2_stream0;
SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */

float a, b, c, d, e, f, bt,zz;
char masukan,keterangan;

double x1, x2;
float VLM, LM, NM, HM, ST, MT, LT, VST;
float r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11;
float AAA, BBB, Kondisi, Vadc, Iadc;
int normal, interuption, sag, swell, sustained_interuption, undervoltage, overvoltage,ket;

bool awal, menu, profil, sistem, logger, kalibrasi;

float program1, program2, jeda,loop,loop1;
float Vadcnol, Vsum, Vsq[sampling_kal], Vrms, Tegangan, Teg, pengurang = 0, konv,AC,DC=1,minkal = 0,Vnom = 220;
float Iadcnol, Isum, Isq[sampling_kal], Vref, VoltageReference, teganganmasukan, kondisi;
float in1,in2,in3,in4,in5,in6,in7,in8,in9,in10,in11,in12,in13,in14,in15,in16,in17,in18,in19,in20,ACDC[acdc];
int j = 0, k = 0,l = 0, minus,tt1,tt2,tt3=0,tt4=0;
//char AC, DC;


//--------------------------------------------------------------------------------
//----------------------------------update grafik - spectrum---------------------------------
double Vg[420],Ig[420];
int h = 0, i = 0,input;

	uint8_t  lcd_status = LCD_OK;
	uint16_t get_y, get_x;
	
char Buf[30],Buf2[30],Buf3[30];
char buf_RX[100], buf_TX[100];
uint8_t rx_data;
//volatile uint16_t adc[2], Vadc, Iadc;  //tinjau ulang untuk bluetooth
__IO uint16_t adc[2]; //opsi lain
static TS_StateTypeDef  TS_State;
	uint8_t NbLoop = 1;
	#ifndef USE_FULL_ASSERT
	uint32_t    ErrorCounter = 0;
	#endif
	
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DMA2D_Init(void);
static void MX_FMC_Init(void);
static void MX_ADC3_Init(void);
static void MX_LTDC_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

void password(void);
void cover(void);
void main_menu(void);
void my_profil(void);
void main_system(void);
void data_logger(void);
void bluetooth(void);
void calibration(void);
void program(void);
void program_rtc(void);
void waktu(void);
void timer1(void);
void timer2(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

void program_rtc()
{
	{
HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); 
HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
day = sDate.Date;
month = sDate.Month;
year = sDate.Year;
hour = sTime.Hours;
min = sTime.Minutes;
sec = sTime.Seconds;
msec = sTime.SubSeconds;
}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/////COBA/////
	if(htim->Instance==TIM2) // pembacaan sensor
	{
		Vadc = adc[0]  ; //nilai adc tegangan asli
		Vadcnol = Vadc - 1600 - minkal ;//1601.08;//- 1589.58; //- 1760; //nilai adc tegangan dikembalikan ke titik 0
		
		Iadc = adc[1]; //nilai adc VoltageReference asli
		Iadcnol = Iadc; //- 2042; //nilai adc VoltageReference dikembalikan ke titik 0
		
		Vsum -= Vsq[k];
		Isum -= Isq[k];
	  		
		//sensor tegangan//		
		Vsq[k] = pow(Vadcnol,2); //nilai kuadrat adc tegangan titik nol ke - k
		
		Vsum += Vsq[k];	//nilai total adc tegangan sampai - k
										
		Vrms = sqrt((Vsum/sampling_kal)); //nilai adc tegangan rms
		if(Vrms<0)
			Vrms = 0;
		
//		if(0<Vrms<Vrms)
//		{
//			pengurang = Vrms;
//		}		
		 		Teg = Vrms - pengurang ;
		
//		if (AC == 1)
//		{
//			if (Teg <= 0)
//			Tegangan =0;	
		//program ac/dc otomatis//
		
//		if (in3>0 && in4==0)
//		{
//			in4=adc[0];
//			HAL_Delay(5);
//		}
//		if (in2>0 && in3==0)
//		{
//			in3=adc[0];
//			HAL_Delay(5);
//		}
//		if (in1>0 && in2==0)
//		{
//			in2=adc[0];
//			HAL_Delay(5);
//		}
//		if(in1==0)
//		{
//		in1=adc[0];
//		HAL_Delay(5);
//		}
//		if (in1<100 || in2<100 || in3<100 || in4<100)
//		{	
//		AC=1;
//		DC=0;
//		in1=0;
//		in2=0;
//		in3=0;
//		in4=0;	
//		}
//			if (in1>=100 && in2>100 && in3>100 && in4>100)
//			{
//				AC=0;
//				DC=1;
//				in1=0;
//				in2=0;
//				in3=0;
//				in4=0;
//			}				
			

		if(DC==0)
		{
		if(0<Teg && Teg<=0.803)
		{
			Tegangan = Teg*3 + 0.2872;
			konv = 1;
		}
		if(0.803<Teg && Teg<=2.024)
		{
			Tegangan = Teg*0.9001 + 1.2642;
			konv = 2;
		}
		if(2.024<Teg && Teg<=8.094)
		{
			Tegangan = Teg*0.6602 + 1.8105;
			konv = 3;
		}
		if(8.094 <Teg && Teg<=9.728)
		{
			Tegangan = Teg*0.5692 + 2.4933;
			konv = 4;
		}
		if(Teg>9.728 && Teg <63)
		{
			Tegangan = Teg*0.5193 - 0.258;
			konv = 5;
		}
		if(Teg>=63)
		{
			Tegangan = Teg*0.517 + 3.6467;
			konv = 5;
		}
	 }
//	}
		
		if (DC == 1)
		{
//			Tegangan = Teg;
		if(Teg<=1.1)
		{
			Tegangan = Teg*1.5114 + 1.2408;
			konv = 1;
		}
		if(1.1<Teg && Teg<=9.9)
		{
			Tegangan = Teg*0.61 + 2.3285;
			konv = 2;
		}
		if(Teg > 9.9)
		{
			Tegangan = Teg*0.5002 + 3.5409;
			konv = 3;
		}
		
	}
	
		//		Tegangan = Teg;
		//	Tegangan = (0.3194 * Vrms) - 0.4208;
	//		Tegangan = (0,5172 * Vrms) - 827,58; //DC
	//			Tegangan = (16.901 * Vrms) - 27030; //AC

		//sensor VoltageReference//
		Isq[k] = pow(Iadcnol,2); //nilai kuadrat adc VoltageReference titik nol ke - k
		
		Isum += Isq[k];	//nilai total adc VoltageReference sampai - k
										
		Vref = sqrt((Isum/sampling_kal)); //nilai adc VoltageReference rms
		if(Vref<0)
			Vref = 0;
		
		VoltageReference = Vref;//(0.010002*Vref)- 0.004929;
		
		k++;							
		if(k>=sampling_kal)
		{
			j = 1;
			k = 0;
		}
	}
	
	
	/////PROGRAM SENSING AC/DC OTOMATIS////////
	if(htim->Instance==TIM3)
	{
//		l++;
//		if (l<=20)
//		{
//			ACDC[l]= Vadc;
//			
//			if (ACDC[1] > 1598)
//				in1=1;
//			if (ACDC[2] > 1598)
//				in2=1;
//			if (ACDC[3] > 1598)
//				in3=1;
//			if (ACDC[4] > 1598)
//				in4=1;
//			if (ACDC[5] > 1598)
//				in5=1;
//			if (ACDC[6] > 1598)
//				in6=1;
//			if (ACDC[7] > 1598)
//				in7=1;
//			if (ACDC[8] > 1598)
//				in8=1;
//			if (ACDC[9] > 1598)
//				in9=1;
//			if (ACDC[10] > 1598)
//			{	
//			in10=1;
//				if (in1==0 && in2==0 && in3==0 && in4==0 && in5==0 && in6==0 && in7==0 && in8==0 && in9==0 && in10==0)
//				{
//					DC=1;
//					minus =1;
//				}
//				if (in1==1 || in2==1 || in3==1 || in4==1 || in5==1 || in6==1 || in7==1 || in8==1 || in9==1 || in10==1)
//				{
//					DC=0;
//					minus =0;
//				
//					if (in1==1 && in2==1 && in3==1 && in4==1 && in5==1 && in6==1 && in7==1 && in8==1 && in9==1 && in10==1)
//					{
//						DC=1;
//						minus =0;
//					}
//				}
//			}
//			
//			
//			if (l==20)
//			{
//				l=0;
//			 in1 = in2= in3 = in4 = in5 =0;
//			}
//		}
		//titik 0 ketika ADC antara 1598-1605
		//program ac/dc otomatis//
		
//		if (in3>0 && in4==0)
//		{
//			in4=adc[0];
//			HAL_Delay(5);
//		}
//		if (in2>0 && in3==0)
//		{
//			in3=adc[0];
//			HAL_Delay(5);
//		}
//		if (in1>0 && in2==0)
//		{
//			in2=adc[0];
//			HAL_Delay(5);
//		}
//		if(in1==0)
//		{
//		in1 = adc[0];
//		HAL_Delay(5);
//		}
//		if (in1<1600 || in2<1600 || in3<1600 || in4<1600)
//		{	
//		AC=1;
//		DC=0;
//		in1=0;
//		in2=0;
//		in3=0;
//		in4=0;	
//		}
//			if (in1>=100 && in2>100 && in3>100 && in4>100)
//			{
//				AC=0;
//				DC=1;
//				in1=0;
//				in2=0;
//				in3=0;
//				in4=0;
//			}	
 
	}

	if(htim->Instance==TIM4) // pembacaan sensor
	{
		program_rtc();
		if (DC==0)
		{
			
		if (Tegangan <= 0.1*Vnom)
		{
			timer1();
			tt4 = 1;
		}
		if (Tegangan <= 0.9*Vnom && Tegangan > 0.1*Vnom)
		{
			timer1();
			tt4 = 2;
		}
		if (Tegangan >= 1.1*Vnom)
		{
			timer1();
			tt4 = 3;
		}
		if (Tegangan < 1.1*Vnom && Tegangan > 0.9*Vnom)
		{
			tt1 = 0;
			tt2 = 0;
			tt3 = 0;
			tt4 = 0;
		}
		
		}
		if (DC==1)
		{
			
			tt1 = 0;
			tt2 = 0;
			tt3 = 0;
			tt4 = 0;
		}
//		waktu();
	}
	
	if(htim->Instance==TIM5)
	{
		
	}
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
	if( BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize())!=TS_OK)
		Error_Handler();
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_ADC3_Init();
  MX_LTDC_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
				 
	lcd_status = BSP_LCD_Init();//get LCD status
	#define LCD_FRAME_BUFFER		SDRAM_DEVICE_ADDR  //address FB

	BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER,LCD_FRAME_BUFFER);//layer initialitation
	BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);//layer selection
	BSP_LCD_SetTransparency(LTDC_ACTIVE_LAYER,199);//set layer transparancy
	BSP_LCD_DisplayOn();//turn on the Lcd
	HAL_ADC_Start_DMA(&hadc3,(uint32_t*)&adc,2);//DMA ADC start
	HAL_UART_Receive_IT (&huart6, &rx_data,1);
//	HAL_UART_Receive_DMA (&huart6,(uint8_t*) &rx_data,2);
	
	awal = 1;
	menu = 0;
	profil = 0;
	kalibrasi = 0;
	sistem = 0;
	logger = 0;
	a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;

	
	normal = 1;
	interuption = 2;
	sag = 3;
	swell = 4;
	sustained_interuption = 5;
	undervoltage = 6;
	overvoltage = 7;
	
	jeda = 0;
	loop = 0;
//	password();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
//				if (in3>0 && in4==0)
//		{
//			in4=Vadc;
////			HAL_Delay(5);
//		}
//		if (in2>0 && in3==0)
//		{
//			in3=Vadc;
////			HAL_Delay(5);
//		}
//		if (in1>0 && in2==0)
//		{
//			in2=Vadc;
////			HAL_Delay(5);
//		}
//		if(in1==0)
//		{
//		in1=Vadc;
////		HAL_Delay(5);
//		}
//		if (in1<100 || in2<100 || in3<100 || in4<100)
//		{	
//		AC=1;
//		DC=0;
//		in1=0;
//		in2=0;
//		in3=0;
//		in4=0;	
//		}
//			if (in1>=100 && in2>100 && in3>100 && in4>100)
//			{
//				AC=0;
//				DC=1;
//				in1=0;
//				in2=0;
//				in3=0;
//				in4=0;
//			}
		BSP_TS_ResetTouchData(&TS_State);
		
		BSP_TS_GetState(&TS_State);
		get_y = TS_State.touchY[0];
		get_x = TS_State.touchX[0];

		if (a==0)					//cover
		{
			if (awal==1) 		
			{
				HAL_TIM_Base_Start_IT(&htim4);
				HAL_TIM_Base_Stop_IT(&htim2);
				HAL_TIM_Base_Stop_IT(&htim3);
				cover();
//				program_rtc();

				awal = 0;
				
			}
			
			if (271 <= get_x && get_x <= (271+46) && 187 <= get_y  && get_y <= (187+46))
				a = 1;
				
			if (176 <= get_x && get_x <= (176+46) && 187 <= get_y  && get_y <= (187+46))
			{
				profil = 1;
				a = 2;
			}
				waktu();			
		}
		
		if (a==1)					//mainmenu
	
		{
			
			if (awal==0)
			{
				HAL_TIM_Base_Start_IT(&htim2);
				HAL_TIM_Base_Start_IT(&htim3);
				main_menu();

				awal = 1 ;
				
			}
			
			if (10 <= get_x && get_x <= 60 && 1 <= get_y  && get_y <= 55)
				a = 0;
					
			
			if	(218 <= get_x && get_x <= 218+69 && 113  <= get_y  && get_y <= 113+46)
			{
				sistem = 1;
				a = 3;
			}
			
			if (359 <= get_x && get_x <= 359+69 && 113 <= get_y  && get_y <= 113+46)
			{
				logger = 1;
				a = 4 ;
			}
			
			if	(113 <= get_x && get_x <= 113+69 && 113 <= get_y  && get_y <= 113+46)
			{
				kalibrasi = 1;
				a = 5;
			}
			waktu();
		}
		
		if (a==2)					//profil
		{
			if (profil==1)
			{
				my_profil();
				profil = 0;
			}
			
			if (10 <= get_x && get_x <= 60 && 1 <= get_y  && get_y <= 55)
			{
				awal = 1;
				a = 0;
			}
		}
		
		if (a==3)				//mainsystem
		{
			b = 0;
			c = 0;
			if (sistem==1)
			{
		BSP_LCD_Clear(LCD_COLOR_WHITE);

		BSP_LCD_SetTextColor(LCD_COLOR_BLACK); //setting warna 
		BSP_LCD_DrawLine(40,35,40,215);  // sumbu VERTIKAL
		BSP_LCD_DrawLine(40,125,465,125); // sumbu HORIZONTAL				
		BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
		BSP_LCD_FillRect(0,220,160,60);
		BSP_LCD_FillRect(320,220,160,60);
		BSP_LCD_FillRect(160,220,160,60);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillRect(0,220,480,2);
		BSP_LCD_SetBackColor(LCD_COLOR_LIGHTBLUE);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_SetFont(&Font20);
		BSP_LCD_DisplayStringAt(0,230,(uint8_t*)"GELOMBANG TEGANGAN MASUKAN",CENTER_MODE);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE); //setting background warna
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetFont(&Font12);
		BSP_LCD_DisplayStringAt(8,20,(uint8_t*)"   v",LEFT_MODE);
		BSP_LCD_DisplayStringAt(12,55,(uint8_t*)"400",LEFT_MODE);
		BSP_LCD_DisplayStringAt(12,97,(uint8_t*)"200",LEFT_MODE);
		BSP_LCD_DisplayStringAt(5,120,(uint8_t*)"   0",LEFT_MODE);
		BSP_LCD_DisplayStringAt(5,143,(uint8_t*)"-200",LEFT_MODE);
		BSP_LCD_DisplayStringAt(5,171,(uint8_t*)"-400",LEFT_MODE);
		BSP_LCD_DisplayStringAt(465,132,(uint8_t*)"t",LEFT_MODE);
		BSP_LCD_DisplayStringAt(445,8,(uint8_t*)"Back",LEFT_MODE);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetFont(&Font16); //setting ukuran tulisan
		BSP_LCD_DisplayStringAt(0,12,(uint8_t*)"Voltage Waveform",CENTER_MODE);		
				for (sistem =1;sistem>0;sistem++)
				{
				main_system();
//				sistem = 0;
					BSP_TS_ResetTouchData(&TS_State);
		
		BSP_TS_GetState(&TS_State);
		get_y = TS_State.touchY[0];
		get_x = TS_State.touchX[0];
			if (400 <= get_x && get_x <= 480 && 1 <= get_y  && get_y <= 55)
			{
				awal = 0;
				a = 1;
				 break;//coba
			}
			sistem = 0;
		}
	}
//						if (365 <= get_x && get_x <= 460 && 205 <= get_y  && get_y <= 265)
//			{
//				sistem = 1;
//				 //coba
//			}
					
//			if (400 <= get_x && get_x <= 480 && 1 <= get_y  && get_y <= 55)
//			{
//				awal = 0;
//				// break; //coba
//				a = 1;
//			}
		}
		
		if (a==4)				//datalogger
		{
			b = 0;
			c = 0;
			if (logger==1)
			{
				data_logger();
				logger = 0;
			}
						
			if (400 <= get_x && get_x <= 480 && 1 <= get_y  && get_y <= 55)
			{
				awal = 0;
				a = 1;
			}
			if (20 <= get_x && get_x <= 100 && 80 <= get_y  && get_y <= 250)
			{
//				for(bt=0;bt>20;bt++)
				
//				bt =1;

//				if (bt==1)
//				{
					bluetooth();
					HAL_Delay (100);
//				}	
//					bt = 0;
//					logger =1;
				
			}
				
		}
		
		if (a==5)				//kalibrasi
		{
			b = 0;
			c = 0;
			if (kalibrasi==1)
			{
				HAL_TIM_Base_Stop_IT(&htim3);
				calibration();
				kalibrasi = 0;
			}
			
			if (10 <= get_x && get_x <= 60 && 1 <= get_y  && get_y <= 55)
			{
				awal = 0;
				a = 1;
			}
			
			if (260 <= get_x && get_x <= 355 && 205 <= get_y  && get_y <= 265)
			{
				pengurang = pengurang - 0.01;
			}			
			
			if (365 <= get_x && get_x <= 460 && 205 <= get_y  && get_y <= 265)
			{
				pengurang = pengurang + 0.01;
			}
			if (260 <= get_x && get_x <= 355 && 5 <= get_y  && get_y <= 65)
			{
				minkal = minkal - 0.01;
			}			
			
			if (365 <= get_x && get_x <= 460 && 5 <= get_y  && get_y <= 65)
			{
				minkal = minkal + 0.01;
			}
			if (20 <= get_x && get_x <= 115 && 205 <= get_y  && get_y <= 265)
			{
				AC = 1;
				DC = 0;
			}			
			
			if (125 <= get_x && get_x <= 220 && 205 <= get_y  && get_y <= 265)
			{
				DC = 1;
				AC = 0;
			}
			
			if (125 <= get_x && get_x <= 220 && 155 <= get_y  && get_y <= 200)
			{
				minus =0;
			}
			if (20 <= get_x && get_x <= 115 && 155 <= get_y  && get_y <= 200)
			{
				minus =1;
			}
			if (261 <= get_x && get_x <= 261+188 && 79 <= get_y  && get_y <= 79+40)
			{
				Vnom = Tegangan ;
			}
			
			
			program();
//			timer2();
//			if (400 <= get_x && get_x <= 480 && 0 <= get_y  && get_y <= 55)
//			{
//				awal = 0;
//				a = 1;
//			}
			}	
		}
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_USART6|RCC_PERIPHCLK_I2C3;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 400;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
  PeriphClkInitStruct.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00401959;
  hi2c3.Init.OwnAddress1 = 224;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 40;
  hltdc.Init.VerticalSync = 9;
  hltdc.Init.AccumulatedHBP = 53;
  hltdc.Init.AccumulatedVBP = 11;
  hltdc.Init.AccumulatedActiveW = 533;
  hltdc.Init.AccumulatedActiveH = 283;
  hltdc.Init.TotalWidth = 565;
  hltdc.Init.TotalHeigh = 285;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 272;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = 0;
  pLayerCfg.ImageWidth = 0;
  pLayerCfg.ImageHeight = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
	
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 9999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/** 
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream0
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
  hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
  hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_MEDIUM;
  hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 6;
  SdramTiming.SelfRefreshTime = 4;
  SdramTiming.RowCycleDelay = 6;
  SdramTiming.WriteRecoveryTime = 2;
  SdramTiming.RPDelay = 2;
  SdramTiming.RCDDelay = 2;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */

  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void password()
{
//		x = 480 5 - 475 = 470
//		y = 272 5 - 267 = 262
		BSP_LCD_Clear(LCD_COLOR_BLACK);
		
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_FillRect(5,5,470,262);
		
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK); //GARIS
		BSP_LCD_FillRect(0,60,480,2);
		BSP_LCD_FillRect(260,60,2,207);
		BSP_LCD_FillRect(267,65,100,98);
		BSP_LCD_FillRect(372,65,100,98);
		BSP_LCD_FillRect(267,168,100,98);
		BSP_LCD_FillRect(372,168,100,98);
		BSP_LCD_FillRect(86,60,2,207);
		BSP_LCD_FillRect(172,60,2,207);
	
		BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
		BSP_LCD_FillRect(5,62,255,205);
} 

void cover()
{
		BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillRect(317,61,163,45);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DrawRect(317,61,163,45);

	//	BSP_LCD_SetTextColor(LCD_COLOR_WHITE); //MENGATUR WARNA KOTAK PADA BAGIAN ATAS
//		BSP_LCD_FillRect(0,0,480,60); //MENGATUR KOORDINAT KOTAK PADA BAGIAN ATAS
	
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillRect(0,60,480,2);
	
		BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
		BSP_LCD_FillCircle(199,209,23);
		BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
		BSP_LCD_FillCircle(294,209,23);
		BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
		BSP_LCD_DrawCircle(189,209,23);
		BSP_LCD_DrawCircle(284,209,23);
	
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetFont(&Font12); //setting ukuran tulisan
		BSP_LCD_DisplayStringAt(178,238,(uint8_t*)"Profil",LEFT_MODE);
		BSP_LCD_DisplayStringAt(267,238,(uint8_t*)"Main Menu",LEFT_MODE); 
//	int hh;
//	for(hh=0; hh<20 ;hh++){
//			BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
//			BSP_LCD_DrawLine(0,170+hh,400,272+hh);
//		
//	}	
	
	//	BSP_LCD_FillRect(0,63,480,2);
	
		BSP_LCD_DrawBitmap(425,6,(uint8_t*)lambangpens);

    BSP_LCD_SetBackColor(LCD_COLOR_WHITE); //setting background warna pada tulisan
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK); //setting warna text
	  BSP_LCD_SetFont(&Font20); //setting ukuran tulisan
		BSP_LCD_DisplayStringAt(28,10,(uint8_t*)"RANCANG BANGUN", LEFT_MODE);
		BSP_LCD_DisplayStringAt(28,31,(uint8_t*)"VOLTMETER CERDAS", LEFT_MODE);
		BSP_LCD_DisplayStringAt(20,42,(uint8_t*)"", LEFT_MODE);
					 
//	  BSP_LCD_SetBackColor(LCD_COLOR_LIGHTBLUE); 
//		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
//		BSP_LCD_SetFont(&Font16);
//		BSP_LCD_DisplayStringAt(0,228,(uint8_t*)"D3 TEKNIK ELEKTRO INDUSTRI",CENTER_MODE);
//		BSP_LCD_DisplayStringAt(0,243,(uint8_t*)"POLITEKNIK ELEKTRONIKA NEGERI SURABAYA",CENTER_MODE);
//		BSP_LCD_DisplayStringAt(0,258,(uint8_t*)"SURABAYA 2020/2021",CENTER_MODE);
		
//		BSP_LCD_SetTextColor(LCD_COLOR_DARKRED);
//		BSP_LCD_SetFont(&Font12);
//		BSP_LCD_DisplayStringAt(6,72,(uint8_t*)"LOCK",LEFT_MODE);
    
//		BSP_LCD_SetTextColor(LCD_COLOR_BLACK); //setting warna text	
//		BSP_LCD_DrawBitmap(0,127,(uint8_t*)wallpaper);
//		BSP_LCD_DrawBitmap(176,187,(uint8_t*)profilku);
//		BSP_LCD_DrawBitmap(271,187,(uint8_t*)mainmenuku);
//		BSP_LCD_DrawRect(99,95,75,75); //menu 1
//		BSP_LCD_DrawRect(299,95,75,75); //menu 4


	}

void main_menu()
{
		BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillRect(317,61,163,45);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DrawRect(317,61,163,45);

//		BSP_LCD_DrawBitmap(328,0,(uint8_t*)condition);
//		BSP_LCD_DrawBitmap(328,0,(uint8_t*)condition2);
//		BSP_LCD_DrawBitmap(328,0,(uint8_t*)condition3);
//		BSP_LCD_DrawBitmap(328,0,(uint8_t*)condition4);
//		BSP_LCD_DrawBitmap(164,0,(uint8_t*)voltagewaveform1);
//		BSP_LCD_DrawBitmap(164+78,0,(uint8_t*)voltagewaveform2);
//		BSP_LCD_DrawBitmap(164,136,(uint8_t*)voltagewaveform3);
//		BSP_LCD_DrawBitmap(164+78,136,(uint8_t*)voltagewaveform4);	
//		BSP_LCD_DrawBitmap(0,0,(uint8_t*)calibrationed);
//		BSP_LCD_DrawBitmap(0,0,(uint8_t*)calibrationed2);
//		BSP_LCD_DrawBitmap(0,0,(uint8_t*)calibrationed3);
//		BSP_LCD_DrawBitmap(0,0,(uint8_t*)calibrationed4);	
	//		BSP_LCD_SetTextColor(LCD_COLOR_WHITE); //MENGATUR WARNA KOTAK PADA BAGIAN ATAS
//		BSP_LCD_FillRect(0,0,480,60); //MENGATUR KOORDINAT KOTAK PADA BAGIAN ATAS
		BSP_LCD_DrawBitmap(425,6,(uint8_t*)lambangpens);		
		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
		BSP_LCD_FillCircle(110,136,23);
		BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGREEN);
		BSP_LCD_FillCircle(241,136,23);
		BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
		BSP_LCD_FillCircle(382,136,23);
		
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawCircle(87,136,23);
		BSP_LCD_DrawCircle(218,136,23);
		BSP_LCD_DrawCircle(359,136,23);
//	
//		BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
//		BSP_LCD_FillRect(0,63,480,2);
//	
//		BSP_LCD_DrawBitmap(425,6,(uint8_t*)lambangpens);

    BSP_LCD_SetBackColor(LCD_COLOR_WHITE); //setting background warna pada tulisan
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK); //setting warna text
		BSP_LCD_DrawRect(6,20,38,18);
	  BSP_LCD_SetFont(&Font20); //setting ukuran tulisan
		BSP_LCD_DisplayStringAt(0,38,(uint8_t*)"MAIN MENU", CENTER_MODE);
		BSP_LCD_SetFont(&Font12);
		BSP_LCD_DisplayStringAt(12,24,(uint8_t*)"Back",LEFT_MODE);
      		
//		BSP_LCD_SetTextColor(LCD_COLOR_BLACK); //setting warna text
//		BSP_LCD_DrawRect(28,115,80,60); //menu 1
//		BSP_LCD_DrawRect(143,115,80,60); //menu 2
//		BSP_LCD_DrawRect(258,115,80,60); //menu 3
//		BSP_LCD_DrawRect(373,115,80,60); //menu 4

		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);

		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetFont(&Font12); //setting ukuran tulisan
	//	BSP_LCD_DisplayStringAt(19,229,(uint8_t*)"Setting",LEFT_MODE);
		BSP_LCD_DisplayStringAt(100,162,(uint8_t*)"Data",LEFT_MODE);
		BSP_LCD_DisplayStringAt(214,162,(uint8_t*)"Gelombang",LEFT_MODE);
		BSP_LCD_DisplayStringAt(359,162,(uint8_t*)"Logger",LEFT_MODE);
		BSP_LCD_DisplayStringAt(255,205,(uint8_t*)"",LEFT_MODE);
		BSP_LCD_DisplayStringAt(19,229,(uint8_t*)"",LEFT_MODE); 	
}	

void my_profil()
{	
		BSP_LCD_Clear(LCD_COLOR_WHITE);
//		BSP_LCD_SetTextColor(LCD_COLOR_WHITE); //MENGATUR WARNA KOTAK PADA BAGIAN ATAS
//		BSP_LCD_FillRect(0,0,480,60); //MENGATUR KOORDINAT KOTAK PADA BAGIAN ATAS
		
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillRect(0,60,480,2);
	
//		BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
//		BSP_LCD_FillRect(0,63,480,2);
	
		BSP_LCD_DrawBitmap(425,6,(uint8_t*)lambangpens);
	
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE); //setting background warna pada tulisan
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK); //setting warna text
		BSP_LCD_DrawRect(6,10,38,18);
	  BSP_LCD_SetFont(&Font16); //setting ukuran tulisan
		BSP_LCD_DisplayStringAt(0,12,(uint8_t*)"DEPARTEMEN ELEKTRO", CENTER_MODE);
		BSP_LCD_DisplayStringAt(0,32,(uint8_t*)"POLITEKNIK ELEKTRONIKA NEGERI SURABAYA", CENTER_MODE);
		BSP_LCD_SetFont(&Font12);
		BSP_LCD_DisplayStringAt(12,14,(uint8_t*)"Back",LEFT_MODE);

		BSP_LCD_SetBackColor(LCD_COLOR_WHITE); //setting background warna pada tulisan
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK); //setting warna text 	
		BSP_LCD_DrawRect(25,70,62,62);
		BSP_LCD_DrawBitmap(25,70,(uint8_t*)voltagewaveform1);					
		BSP_LCD_DrawRect(25,138,62,62);
		BSP_LCD_DrawBitmap(25,138,(uint8_t*)voltagewaveform2);	
		BSP_LCD_DrawRect(25,206,62,62);
		BSP_LCD_DrawBitmap(25,206,(uint8_t*)voltagewaveform3);	
		BSP_LCD_DrawRect(100,70,368,60); 
		BSP_LCD_DrawRect(100,138,368,60);
		BSP_LCD_DrawRect(100,206,368,60);
		
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_DisplayStringAt(101,78,(uint8_t*)" Andrean Rangga Kusuma",LEFT_MODE);
		BSP_LCD_DisplayStringAt(101,93,(uint8_t*)" 3 D3 Elektro Industri A",LEFT_MODE);
	//	BSP_LCD_DisplayStringAt(101,108,(uint8_t*)" Kedungsumur, Krembung, Sidoarjo",LEFT_MODE);
					
		BSP_LCD_DisplayStringAt(101,148,(uint8_t*)" DOSEN PEMBIMBING 1",LEFT_MODE);
		BSP_LCD_DisplayStringAt(101,163,(uint8_t*)" Eka prasetyono, S.ST., M.T.",LEFT_MODE);
	//	BSP_LCD_DisplayStringAt(101,178,(uint8_t*)" NIP. 19831122.201012.1.004",LEFT_MODE);
			
		BSP_LCD_DisplayStringAt(101,216,(uint8_t*)" DOSEN PEMBIMBING 2",LEFT_MODE);
		BSP_LCD_DisplayStringAt(101,231,(uint8_t*)" Putu Agus Mahadi Putra,S.T.,M.T. ",LEFT_MODE);
	//	BSP_LCD_DisplayStringAt(101,246,(uint8_t*)" NIP. 19910119.201803.1.001",LEFT_MODE);
}

void main_system()
{
//		BSP_LCD_Clear(LCD_COLOR_WHITE);
//		BSP_LCD_SetTextColor(LCD_COLOR_BLACK); //setting warna 
//		BSP_LCD_DrawLine(40,125,465,125); // sumbu HORIZONTAL
//		BSP_LCD_DrawLine(40,35,40,215);  // sumbu VERTIKAL
//	
//		BSP_LCD_SetTextColor(LCD_COLOR_DARKBLUE);
//		BSP_LCD_FillRect(0,220,160,60);
//		BSP_LCD_FillRect(320,220,160,60);
//	
//		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
//		BSP_LCD_FillRect(160,220,160,60);
//		BSP_LCD_SetBackColor(LCD_COLOR_WHITE); //setting background warna
//		BSP_LCD_SetFont(&Font12);
//		BSP_LCD_DisplayStringAt(8,20,(uint8_t*)"   v",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(12,38,(uint8_t*)"400",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(12,79,(uint8_t*)"200",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(5,120,(uint8_t*)"   0",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(5,161,(uint8_t*)"-200",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(5,202,(uint8_t*)"-400",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(465,132,(uint8_t*)"t",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(445,8,(uint8_t*)"Back",LEFT_MODE);
//		BSP_LCD_SetFont(&Font16); //setting ukuran tulisan
//		BSP_LCD_DisplayStringAt(0,12,(uint8_t*)"Voltage Waveform",CENTER_MODE);
	
//		for(i=0;i<3;i++)
//		while(1)
//		{
//		BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK); //setting warna 
		BSP_LCD_DrawLine(40,125,465,125); // sumbu HORIZONTAL
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_FillRect(41,35,424,89);
		BSP_LCD_FillRect(41,126,424,89);
//		BSP_LCD_DrawLine(40,35,40,215);  // sumbu VERTIKAL
	
//		BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
//		BSP_LCD_FillRect(0,220,160,60);
//		BSP_LCD_FillRect(320,220,160,60);
//	
//		BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
//		BSP_LCD_FillRect(160,220,160,60);
//		BSP_LCD_SetBackColor(LCD_COLOR_WHITE); //setting background warna
//		BSP_LCD_SetFont(&Font12);
//		BSP_LCD_DisplayStringAt(8,20,(uint8_t*)"   v",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(12,38,(uint8_t*)"400",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(12,79,(uint8_t*)"200",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(5,120,(uint8_t*)"   0",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(5,161,(uint8_t*)"-200",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(5,202,(uint8_t*)"-400",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(465,132,(uint8_t*)"t",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(445,8,(uint8_t*)"Back",LEFT_MODE);
//		BSP_LCD_SetFont(&Font16); //setting ukuran tulisan
//		BSP_LCD_DisplayStringAt(0,12,(uint8_t*)"Voltage Waveform",CENTER_MODE);			
			///////////////Update Data Untuk Grafik////////////////////////////
//		sprintf(buf_TX,"Vrms = %.2f\r\n",Tegangan);
//		HAL_UART_Transmit(&huart6, (uint8_t*) buf_TX, strlen(buf_TX),500);
		
//		if (awal==0)
//			{
//				
//				break; //coba
//			}
		
		for(h=419;h>=0;h--)
			{
				Vg[h]=(adc[0]*300/4095)-50;
  	  HAL_Delay(1);
			}
					
			///////////////////Tampilkan Grafik ////////////////////////////////
			for(h=418;h>=0;h--)
			{
		
				BSP_LCD_SetTextColor(LCD_COLOR_BLUE); 
				BSP_LCD_DrawLine(40+419-h, 193-Vg[h], 40+419-h, 193-Vg[h+1]);
			}
//			if (h==418)
//			{
//				
//				break; //coba
//			}
			
			HAL_Delay (300);
//			if (awal==0)
//			{
//				break;
//			}	
			i = 0;
//		}  
			
}	

void data_logger()
{	
		BSP_LCD_Clear(LCD_COLOR_CYAN);
		BSP_LCD_SetBackColor(LCD_COLOR_CYAN); //setting background warna pada tulisan
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE); //setting warna text
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_DisplayStringAt(416,14,(uint8_t*)"Back",LEFT_MODE);
		BSP_LCD_FillRect(9,70,463,194);
		BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
		BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
		BSP_LCD_FillRect(20,80,80,170);
		BSP_LCD_DrawRect(110,80,350,170);
		BSP_LCD_DisplayStringAt(230,90,(uint8_t*)"KETERANGAN",LEFT_MODE);
		BSP_LCD_DisplayStringAt(115,115,(uint8_t*)"#Tekan tombol Start Untuk memu-",LEFT_MODE);
		BSP_LCD_DisplayStringAt(125,135,(uint8_t*)"lai RECORD data ke Smartphone",LEFT_MODE);
		BSP_LCD_DisplayStringAt(115,160,(uint8_t*)"Data yang terkirim :",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(125,180,(uint8_t*)"selesai",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(115,205,(uint8_t*)"#Keluar dari mode LOGGER untuk",LEFT_MODE);
//		BSP_LCD_DisplayStringAt(125,225,(uint8_t*)"memilih menu lain",LEFT_MODE);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawRect(20,80,80,170);
		BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
		BSP_LCD_DrawRect(8,69,464,194);
		BSP_LCD_DrawRect(9,70,462,192);
		BSP_LCD_SetFont(&Font24);
		BSP_LCD_SetBackColor(LCD_COLOR_CYAN);
		BSP_LCD_DisplayStringAt(0,20,(uint8_t*)"DATA LOGGER",CENTER_MODE);
		BSP_LCD_DisplayStringAt(0,41,(uint8_t*)"(Serial Mode)",CENTER_MODE);
		BSP_LCD_DisplayStringAt(50,110,(uint8_t*)"S",LEFT_MODE);
		BSP_LCD_DisplayStringAt(50,130,(uint8_t*)"T",LEFT_MODE);	
		BSP_LCD_DisplayStringAt(50,150,(uint8_t*)"A",LEFT_MODE);	
		BSP_LCD_DisplayStringAt(50,170,(uint8_t*)"R",LEFT_MODE);	
		BSP_LCD_DisplayStringAt(50,190,(uint8_t*)"T",LEFT_MODE);
}

void calibration ()
{
//		BSP_LCD_Clear(LCD_COLOR_LIGHTBLUE);
//		BSP_LCD_SetTextColor(LCD_COLOR_WHITE); //MENGATUR WARNA KOTAK PADA BAGIAN ATAS
//		BSP_LCD_FillRect(0,0,480,60); //MENGATUR KOORDINAT KOTAK PADA BAGIAN ATAS
//		
//		BSP_LCD_SetTextColor(LCD_COLOR_YELLOW);
//		BSP_LCD_FillRect(0,60,480,3);
//	
//		BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
//		BSP_LCD_FillRect(0,63,480,2);
//	
//		BSP_LCD_SetBackColor(LCD_COLOR_WHITE); //setting background warna pada tulisan
//    BSP_LCD_SetTextColor(LCD_COLOR_BLACK); //setting warna text
//		BSP_LCD_SetFont(&Font16); //setting ukuran tulisan
//		BSP_LCD_DisplayStringAt(0,38,(uint8_t*)"SENSOR CALIBRATION", CENTER_MODE);
//		BSP_LCD_SetFont(&Font12);
//		
//		BSP_LCD_DrawRect(435,20,38,18);
//		BSP_LCD_DisplayStringAt(440,24,(uint8_t*)"Back",LEFT_MODE);
//		BSP_LCD_DrawBitmap(6,6,(uint8_t*)lambangpens);

		BSP_LCD_Clear(LCD_COLOR_WHITE);
//		BSP_LCD_SetTextColor(LCD_COLOR_WHITE); //MENGATUR WARNA KOTAK PADA BAGIAN ATAS
//		BSP_LCD_FillRect(0,0,480,60); //MENGATUR KOORDINAT KOTAK PADA BAGIAN ATAS
		
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillRect(0,60,480,2);
	
//		BSP_LCD_SetTextColor(LCD_COLOR_ORANGE);
//		BSP_LCD_FillRect(0,63,480,2);
	
		BSP_LCD_DrawBitmap(425,6,(uint8_t*)lambangpens);

    BSP_LCD_SetBackColor(LCD_COLOR_WHITE); //setting background warna pada tulisan
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK); //setting warna text
		BSP_LCD_DrawRect(6,20,38,18);
	  BSP_LCD_SetFont(&Font16); //setting ukuran tulisan
		BSP_LCD_DisplayStringAt(0,38,(uint8_t*)"DATA", CENTER_MODE);
		BSP_LCD_SetFont(&Font12);
		BSP_LCD_DisplayStringAt(12,24,(uint8_t*)"Back",LEFT_MODE);
		
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_FillRect(250,70,210,83);
		BSP_LCD_FillRect(250,155,210,83);
	
//    BSP_LCD_SetTextColor(LCD_COLOR_WHITE); 
//		BSP_LCD_FillRect(9,90,463,160);
				
		BSP_LCD_SetTextColor(LCD_COLOR_LIGHTBLUE);
		BSP_LCD_FillRect(21,71,209,56);
		BSP_LCD_FillRect(21,128,209,26);
		BSP_LCD_FillRect(251,155,209,20);
		BSP_LCD_FillRect(262,80,186,38);
		
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DrawRect(8,64,464,185);
    BSP_LCD_DrawRect(9,65,462,183);
		BSP_LCD_DrawRect(20,70,210,162);
//		BSP_LCD_DrawRect(20,127,1,1);
		BSP_LCD_DrawRect(250,70,210,162);
		BSP_LCD_FillRect(20,155,210,83);
		
		BSP_LCD_FillRect(144,76,80,46);
		
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_DrawRect(143,75,82,48);
		BSP_LCD_DrawRect(261,79,188,40);
		BSP_LCD_DrawLine(250,127,460,127);
		
//		BSP_LCD_SetTextColor(LCD_COLOR_RED);
//		BSP_LCD_FillRect(260,205,95,60); //a
//		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
//		BSP_LCD_FillRect(365,205,95,60); //a
		
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_SetBackColor(LCD_COLOR_LIGHTBLUE);
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_DisplayStringAt(25,73,(uint8_t*)"TEGANGAN :",LEFT_MODE);
		BSP_LCD_DisplayStringAt(298,158,(uint8_t*)"Keterangan",LEFT_MODE);
		BSP_LCD_SetFont(&Font12);
		BSP_LCD_DisplayStringAt(305,85,(uint8_t*)"Tekan untuk set",LEFT_MODE);
		BSP_LCD_DisplayStringAt(325,99,(uint8_t*)"V.Nominal",LEFT_MODE);
		
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
			BSP_LCD_SetFont(&Font16);
			sprintf((char*)Buf3,"---------------");
			BSP_LCD_DisplayStringAt(260,185,(uint8_t*)Buf3, LEFT_MODE);
			
}

void program ()
{
		jeda++;
		if (jeda>=1000)
		{
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			BSP_LCD_SetBackColor(LCD_COLOR_LIGHTBLUE);
			BSP_LCD_SetFont(&Font16);
			sprintf((char*)Buf2,"V.ADC     = %.0f", Vadc);
			BSP_LCD_DisplayStringAt(30,134,(uint8_t*)Buf2,LEFT_MODE);
			
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
			BSP_LCD_SetFont(&Font16);
			sprintf((char*)Buf2,"V.Nominal = %.0f   ", Vnom);
			BSP_LCD_DisplayStringAt(251,134,(uint8_t*)Buf2,LEFT_MODE);
//			sprintf((char*)Buf2,"V.ADC.RMS = %.2f", Vrms);
//			BSP_LCD_DisplayStringAt(30,175,(uint8_t*)Buf2, LEFT_MODE);
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
			BSP_LCD_SetFont(&Font24);
			sprintf((char*)Buf2,"%.2f", Tegangan);
			BSP_LCD_DisplayStringAt(80,175,(uint8_t*)Buf2, LEFT_MODE);
			
			
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
			BSP_LCD_SetFont(&Font16);
			sprintf((char*)Buf2,"Volt");
			BSP_LCD_DisplayStringAt(100,215,(uint8_t*)Buf2, LEFT_MODE);
//			sprintf((char*)Buf3,"Tegangan AC %.0f", konv );//teganganmasukan); //%.0f
//			BSP_LCD_DisplayStringAt(260,145,(uint8_t*)Buf3,LEFT_MODE);
//			sprintf((char*)Buf3,"Teg = %.2f", Teg);
//			BSP_LCD_DisplayStringAt(260,175,(uint8_t*)Buf3, LEFT_MODE);
//			sprintf((char*)Buf3,"Vref = %.2f", VoltageReference);
//			BSP_LCD_DisplayStringAt(260,145,(uint8_t*)Buf3, LEFT_MODE);
//			sprintf((char*)Buf3,"Vradc = %.2f", Iadcnol);
//			BSP_LCD_DisplayStringAt(260,205,(uint8_t*)Buf3, LEFT_MODE);
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
			BSP_LCD_SetFont(&Font24);
			if (DC == 0)
			{
			sprintf((char*)Buf3,"AC ", konv );//teganganmasukan); //%.0f
			BSP_LCD_DisplayStringAt(167,89,(uint8_t*)Buf3,LEFT_MODE);
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
			BSP_LCD_SetFont(&Font16);
	if (tt4 == 0)
	{
			sprintf((char*)Buf3,"Normal         ");
			BSP_LCD_DisplayStringAt(260,185,(uint8_t*)Buf3, LEFT_MODE);
			
	}
	if (tt4 == 1 && tt3 >60)
	{
			sprintf((char*)Buf3,"S. Interruption");
			BSP_LCD_DisplayStringAt(260,185,(uint8_t*)Buf3, LEFT_MODE);
			
	}
	if (tt4 == 2 && tt3 >60)
	{
			
			sprintf((char*)Buf3,"Undervoltage   ");
			BSP_LCD_DisplayStringAt(260,185,(uint8_t*)Buf3, LEFT_MODE);
			
	}
	if (tt4 == 3 && tt3 >60)
	{
			sprintf((char*)Buf3,"Overvoltage    ");
			BSP_LCD_DisplayStringAt(260,185,(uint8_t*)Buf3, LEFT_MODE);
			
	}
				
//			if (Tegangan <= 22)
//			{
//			sprintf((char*)Buf3,"S. Interruption", VoltageReference);
//			BSP_LCD_DisplayStringAt(260,175,(uint8_t*)Buf3, LEFT_MODE);
//			}
//			if (Tegangan > 22 && Tegangan <= 198)
//			{
//			sprintf((char*)Buf3,"Undervoltage   ", VoltageReference);
//			BSP_LCD_DisplayStringAt(260,175,(uint8_t*)Buf3, LEFT_MODE);
//			}
//			if (Tegangan > 198 && Tegangan <= 242)
//			{
//			sprintf((char*)Buf3,"Normal         ", VoltageReference);
//			BSP_LCD_DisplayStringAt(260,175,(uint8_t*)Buf3, LEFT_MODE);
//			}
//			if (Tegangan > 242 && Tegangan <= 400)
//			{
//			sprintf((char*)Buf3,"Overvoltage    ", VoltageReference);
//			BSP_LCD_DisplayStringAt(260,175,(uint8_t*)Buf3, LEFT_MODE);
//			}			
		}	
			if (DC == 1)
			{
			sprintf((char*)Buf3,"DC" );//teganganmasukan); //%.0f
			BSP_LCD_DisplayStringAt(167,89,(uint8_t*)Buf3,LEFT_MODE);
//			sprintf((char*)Buf3,"                ", VoltageReference);
//			BSP_LCD_DisplayStringAt(260,175,(uint8_t*)Buf3, LEFT_MODE);
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
			BSP_LCD_SetFont(&Font24);
				if(minus==1)
			{
				sprintf((char*)Buf2,"-");
				BSP_LCD_DisplayStringAt(55,175,(uint8_t*)Buf2, LEFT_MODE);
			}
			if(minus==0)
			{
				sprintf((char*)Buf2," ");
				BSP_LCD_DisplayStringAt(55,175,(uint8_t*)Buf2, LEFT_MODE);
			}
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
			BSP_LCD_SetFont(&Font16);
			sprintf((char*)Buf3,"---------------");
			BSP_LCD_DisplayStringAt(260,185,(uint8_t*)Buf3, LEFT_MODE);
			
			}
			
			
	
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
			BSP_LCD_SetFont(&Font16);
			sprintf((char*)rtc,"%d:%d:%d   ",hour, min,sec); 
			BSP_LCD_DisplayStringAt(290,5,(uint8_t*)rtc,LEFT_MODE);
		
//			sprintf((char*)Buf3,"Vref     = %.2f", VoltageReference);
//			BSP_LCD_DisplayStringAt(260,205,(uint8_t*)Buf3, LEFT_MODE);
			//	sprintf((char*)Buf3,"Kondisi = %.2f", Vref);
		//	BSP_LCD_DisplayStringAt(260,175,(uint8_t*)Buf3, LEFT_MODE);
		//	sprintf((char*)Buf3,"I.RMS     = %.2f", VoltageReference);
		//	BSP_LCD_DisplayStringAt(260,205,(uint8_t*)Buf3, LEFT_MODE);
			
			jeda = 0;
		//	HAL_Delay(200);
		}
}


void bluetooth ()
{
	
//		BSP_LCD_Clear(LCD_COLOR_LIGHTBLUE);
//		BSP_LCD_SetTextColor(LCD_COLOR_RED); //MENGATUR WARNA KOTAK PADA BAGIAN ATAS
//		BSP_LCD_FillRect(0,0,480,60);
//	loop1=0;
//	for (loop1=0;loop1>=50;loop++)
//{
	if (DC==0)
	{
		
		if (tt4 ==0)
		{
		sprintf(buf_TX,"Vrms = %.2f (AC) || %.2f\n %d:%d\n Vnom = %.0f\n Keterangan : NORMAL\n",Tegangan,Vadc,sec,msec,Vnom );
		HAL_UART_Transmit_IT(&huart6, (uint8_t*) buf_TX, strlen(buf_TX));
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillRect(124,205,320,42);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
		BSP_LCD_SetFont(&Font16);
		sprintf((char*)Buf2,"Vrms = %.2f (AC) || %.2f",Tegangan,Vadc);
		BSP_LCD_DisplayStringAt(125,190,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"%d:%d Vnom = %.0f",min,sec,Vnom);
		BSP_LCD_DisplayStringAt(125,205,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"Keterangan : NORMAL");
		BSP_LCD_DisplayStringAt(125,220,(uint8_t*)Buf2, LEFT_MODE);
		}
		if (tt4 == 1 && tt3 >60)
	{
		sprintf(buf_TX,"Vrms = %.2f (AC) || %.2f\n %d:%d\n Vnom = %.0f\n Keterangan : S.INTERRUPTION\n",Tegangan,Vadc,sec,msec,Vnom );
		HAL_UART_Transmit_IT(&huart6, (uint8_t*) buf_TX, strlen(buf_TX));
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillRect(124,205,320,42);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
		BSP_LCD_SetFont(&Font16);
		sprintf((char*)Buf2,"Vrms = %.2f (AC) || %.2f",Tegangan,Vadc);
		BSP_LCD_DisplayStringAt(125,190,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"%d:%d Vnom = %.0f",min,sec,Vnom);
		BSP_LCD_DisplayStringAt(125,205,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"Keterangan : S.INTERRUPTION");
		BSP_LCD_DisplayStringAt(125,220,(uint8_t*)Buf2, LEFT_MODE);
	}
	if (tt4 == 2 && tt3 >60)
	{
		sprintf(buf_TX,"Vrms = %.2f (AC) || %.2f\n %d:%d\n Vnom = %.0f\n Keterangan : UNDERVOLTAGE\n",Tegangan,Vadc,sec,msec,Vnom );
		HAL_UART_Transmit_IT(&huart6, (uint8_t*) buf_TX, strlen(buf_TX));
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillRect(124,205,320,42);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
		BSP_LCD_SetFont(&Font16);
		sprintf((char*)Buf2,"Vrms = %.2f (AC) || %.2f",Tegangan,Vadc);
		BSP_LCD_DisplayStringAt(125,190,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"%d:%d Vnom = %.0f",min,sec,Vnom);
		BSP_LCD_DisplayStringAt(125,205,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"Keterangan : UNDERVOLTAGE");
		BSP_LCD_DisplayStringAt(125,220,(uint8_t*)Buf2, LEFT_MODE);
	}
	if (tt4 == 3 && tt3 >60)
	{	
		sprintf(buf_TX,"Vrms = %.2f (AC) || %.2f\n %d:%d\n Vnom = %.0f\n Keterangan : OVERVOLTAGE\n",Tegangan,Vadc,sec,msec,Vnom );
		HAL_UART_Transmit_IT(&huart6, (uint8_t*) buf_TX, strlen(buf_TX));
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillRect(124,205,320,42);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
		BSP_LCD_SetFont(&Font16);
		sprintf((char*)Buf2,"Vrms = %.2f (AC) || %.2f",Tegangan,Vadc);
		BSP_LCD_DisplayStringAt(125,190,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"%d:%d Vnom = %.0f",min,sec,Vnom);
		BSP_LCD_DisplayStringAt(125,205,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"Keterangan : OVERVOLTAGE");
		BSP_LCD_DisplayStringAt(125,220,(uint8_t*)Buf2, LEFT_MODE);
	}
	if (tt4 == 1 && tt3 <60)
	{
		sprintf(buf_TX,"Vrms = %.2f (AC) || %.2f\n %d:%d\n Vnom = %.0f\n Keterangan : ------\n",Tegangan,Vadc,sec,msec,Vnom );
		HAL_UART_Transmit_IT(&huart6, (uint8_t*) buf_TX, strlen(buf_TX));
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillRect(124,205,320,42);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
		BSP_LCD_SetFont(&Font16);
		sprintf((char*)Buf2,"Vrms = %.2f (AC) || %.2f",Tegangan,Vadc);
		BSP_LCD_DisplayStringAt(125,190,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"%d:%d Vnom = %.0f",min,sec,Vnom);
		BSP_LCD_DisplayStringAt(125,205,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"Keterangan : ------");
		BSP_LCD_DisplayStringAt(125,220,(uint8_t*)Buf2, LEFT_MODE);
	}
	if (tt4 == 2 && tt3 <60)
	{
		sprintf(buf_TX,"Vrms = %.2f (AC) || %.2f\n %d:%d\n Vnom = %.0f\n Keterangan : ------\n",Tegangan,Vadc,sec,msec,Vnom );
		HAL_UART_Transmit_IT(&huart6, (uint8_t*) buf_TX, strlen(buf_TX));
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillRect(124,205,320,42);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
		BSP_LCD_SetFont(&Font16);
		sprintf((char*)Buf2,"Vrms = %.2f (AC) || %.2f",Tegangan,Vadc);
		BSP_LCD_DisplayStringAt(125,190,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"%d:%d Vnom = %.0f",min,sec,Vnom);
		BSP_LCD_DisplayStringAt(125,205,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"Keterangan : ------");
		BSP_LCD_DisplayStringAt(125,220,(uint8_t*)Buf2, LEFT_MODE);
	}
	if (tt4 == 3 && tt3 <60)
	{
		sprintf(buf_TX,"Vrms = %.2f (AC) || %.2f\n %d:%d\n Vnom = %.0f\n Keterangan : ------\n",Tegangan,Vadc,sec,msec,Vnom );
		HAL_UART_Transmit_IT(&huart6, (uint8_t*) buf_TX, strlen(buf_TX));
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillRect(124,205,320,42);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
		BSP_LCD_SetFont(&Font16);
		sprintf((char*)Buf2,"Vrms = %.2f (AC) || %.2f",Tegangan,Vadc);
		BSP_LCD_DisplayStringAt(125,190,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"%d:%d Vnom = %.0f",min,sec,Vnom);
		BSP_LCD_DisplayStringAt(125,205,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"Keterangan : ------");
		BSP_LCD_DisplayStringAt(125,220,(uint8_t*)Buf2, LEFT_MODE);
	}
	}
	if (DC==1)
	{
		sprintf(buf_TX,"Vrms = %.2f (DC) || %.2f\n %d:%d\n Vnom = %.0f\n Keterangan : ------\n",Tegangan,Vadc,sec,msec,Vnom );
		HAL_UART_Transmit_IT(&huart6, (uint8_t*) buf_TX, strlen(buf_TX));
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillRect(124,205,320,42);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
		BSP_LCD_SetFont(&Font16);
		sprintf((char*)Buf2,"Vrms = %.2f (DC) || %.2f",Tegangan,Vadc);
		BSP_LCD_DisplayStringAt(125,190,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"%d:%d Vnom = %.0f",min,sec,Vnom);
		BSP_LCD_DisplayStringAt(125,205,(uint8_t*)Buf2, LEFT_MODE);
		sprintf((char*)Buf2,"Keterangan : ------\n");
		BSP_LCD_DisplayStringAt(125,220,(uint8_t*)Buf2, LEFT_MODE);
		
	}
//		HAL_Delay(2);
//}
	
}

void waktu ()
{
	loop++;
	if (loop>=1)
	{

	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
	
	BSP_LCD_SetFont(&Font16);
	sprintf((char*)rtc,"%d-%d-%d   ",day, month,year);
	BSP_LCD_DisplayStringAt(320,64,(uint8_t*)rtc,LEFT_MODE);
		sprintf((char*)rtc,"%d:%d:%d:%d   ",hour, min,sec,msec); 
	BSP_LCD_DisplayStringAt(320,84,(uint8_t*)rtc,LEFT_MODE);
		loop =0;
	}
}

void timer1 ()
{
	tt1 = sec;
	if (tt2 != tt1)
	{
	tt3 = tt3+1;
		tt2 = tt1;
	}		
		
}

void timer2 ()
{
//			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
//			BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
//			BSP_LCD_SetFont(&Font16);
//	if (tt4 == 0)
//	{
//			sprintf((char*)Buf3,"Normal         ");
//			BSP_LCD_DisplayStringAt(260,185,(uint8_t*)Buf3, LEFT_MODE);
//	}
//	if (tt4 == 1 && tt3 >60)
//	{
//			sprintf((char*)Buf3,"S. Interruption");
//			BSP_LCD_DisplayStringAt(260,185,(uint8_t*)Buf3, LEFT_MODE);
//	}
//	if (tt4 == 2 && tt3 >60)
//	{
//			sprintf((char*)Buf3,"Undervoltage   ");
//			BSP_LCD_DisplayStringAt(260,185,(uint8_t*)Buf3, LEFT_MODE);
//	}
//	if (tt4 == 3 && tt3 >60)
//	{
//			sprintf((char*)Buf3,"Overvoltage    ");
//			BSP_LCD_DisplayStringAt(260,185,(uint8_t*)Buf3, LEFT_MODE);
//	}
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();
  /** Initializes and configures the Region and the memory to be protected 
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x20010000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_16MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
