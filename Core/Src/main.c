/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD16X2.h"
#include "stdint.h"
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "FLASH_PAGE_F1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
  RTC_DateTypeDef gDate;
  RTC_TimeTypeDef gTime;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Ac_Available HAL_GPIO_ReadPin(Ac_Available_GPIO_Port, Ac_Available_Pin)
#define Flash_Address 0x0800FC00
#define MyLCD LCD16X2_1
#define DS1307_ADDRESS 0xD0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
  typedef struct {
  	uint8_t seconds;
  	uint8_t minutes;
  	uint8_t hour;
  	uint8_t dayofweek;
  	uint8_t dayofmonth;
  	uint8_t month;
  	uint8_t year;
  } TIME;

  TIME time_ds1307;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */
char time[10];
char date[10];
uint8_t alarm =0;
uint32_t adc_value=0;
uint32_t adc_vref=0;
uint32_t adc_buffer=0;
float Battery_Voltage=0,Vin_Battery=0;
unsigned char buffer[32];
uint32_t voltage=0;
//-------------------------------VARIABLES---------------------------------------
//unsigned short old_time_compare_pv,old_time_update_pv,old_time_screen_1=0,old_time_screen_2=0; // to async
char set_status=0;    //variable for the set button state
char txt[21];
char seconds_lcd_1=0,minutes_lcd_1=0,hours_lcd_1=0;
char seconds_lcd_2=0,minutes_lcd_2=0,hours_lcd_2=0;
char hours_lcd_timer2_start=8,hours_lcd_timer2_stop=0,seconds_lcd_timer2_start=0;
char minutes_lcd_timer2_start=0,minutes_lcd_timer2_stop=0,seconds_lcd_timer2_stop=0;
char hours_lcd_timer3_start=0,hours_lcd_timer3_stop=0,minutes_lcd_timer3_start=0,minutes_lcd_timer3_stop=0;
char Relay_State; // variable for toggling relay
char set_ds1307_minutes=12,set_ds1307_hours=12,set_ds1307_seconds=0,set_ds1307_day=0,set_ds1307_month=0,set_ds1307_year=23;
char ByPassState=0;    //enabled is default 0 is enabled and 1 is disabled
float Battery_Voltage,PV_Voltage,Vin_PV,Vin_PV_Old=0,Vin_PV_Present=0,vref;
char BatteryVoltageSystem=0; // to save the battery voltage system if it is 12v/24v/48v
unsigned int ADC_Value;   // adc value for battery voltage
unsigned int ADC_Value_PV;
float Vin_Battery;      //voltage of battery
float Mini_Battery_Voltage=0.0,Mini_Battery_Voltage_T2=0,Mini_Battery_Voltage_T3=0;     // for timer 1 and timer 2
char Timer_Enable=1;   // timer 1
char Timer_2_Enable=1; // timer 2
char Timer_3_Enable=1; //timer 3
char CorrectionTime_State=0;  // this function to solve the error when battery is low and timer didn't start because of the low battery
char VoltageProtectorGood;
char BatteryGuardEnable=1;   // enabled is default
char VoltageProtectionEnable; // enable voltage protection on grid
char Error_Voltage=0;       //difference between voltage and reading voltage
float v; // ac voltage as global variable
char Saved_Voltage;     // volatge when user hits set
char Adjusted_Voltage; // voltage saved by user
char AcBuzzerActiveTimes=0;  //for not making buzzer always on
char AcBuzzerActive=0;  //  for controlling buzzer activation just for one time
char matched_timer_1_start,matched_timer_1_stop, matched_timer_2_start,matched_timer_2_stop,matched_timer_3_start,matched_timer_3_stop;
char Old_Reg=0;
char SolarOnGridOff=0,SolarOffGridOn=0;
char SolarOnGridOff_2=0,SolarOffGridOn_2=0;
char Timer_isOn=0,Timer_2_isOn=0,Timer_3_isOn=0;
unsigned int Timer_Counter_2=0, Timer_Counter_3=0,Timer_Counter_4=0,Timer_Counter_5=0;
unsigned int Low_PV_Voltage=50;       // PV panels low voltage
bool Grid_Already_On=false;            // to not enter conditions as the grid is available
unsigned short old_timer_1=0,old_timer_2=0,temp=0;
unsigned int startupTIme_1=0,startupTIme_2=0,startupTIme_3=0;  // 25 seconds for load one to start up and 50 seconds for load 2 to startup
unsigned int delayTimerOff_1=0,delayTimerOff_2=0,delayTimerOff_3=0;
char updateScreen=0;
float arrayBatt[21];
float StartLoadsVoltage=0,StartLoadsVoltage_T2=0,StartLoadsVoltage_T3=0;
unsigned short ReadMinutesMinusOldTimer_1=0;
unsigned short ReadMinutesMinusOldTimer_2=0;
unsigned int Timer_Counter_For_Grid_Turn_Off=0;
char RunTimersNowState=0;
unsigned int SecondsRealTime=0;         // for holding reading seconds in real time for ac grid and startup timers
unsigned int SecondsRealTimePv_ReConnect_T1=0,SecondsRealTimePv_ReConnect_T2=0,SecondsRealTimePv_ReConnect_T3=0; // for reactive timers in sequence when timer switch off because off battery and wants to reload
unsigned int realTimeLoop=0;
bool RunWithOutBattery=false;
char const ButtonDelay=200;
char RunLoadsByBass=0;
char TurnOffLoadsByPass=0; // to turn off for error
char VoltageProtectorEnableFlag=1;
char every30MinutesInitScreen=0;
char initedScreenOnce=0;
unsigned int UpdateScreenTime=0,TimeToExitSetupProgram=0;
char SystemBatteryMode=0;
char EnterSetupProgram=0; // variable to detect if mcu in loop of setup program
unsigned int  ReadBatteryTime=0;
char RunOnBatteryVoltageMode=1;
bool UPSMode=0;       // i made ups mode and upo mode in same variable
char LoadsAlreadySwitchedOFF=0;
unsigned short Full_Minutes,Full_Hours,Full_Seconds;
unsigned short seconds_reg_1_On;
unsigned short minutes_reg_1_On,hours_reg_1_On;
unsigned short seconds_reg_2_On;
unsigned short minutes_reg_2_On,hours_reg_2_On;
char bcd_value_seconds_L_On,bcd_value_seconds_H_On;
char bcd_value_minutes_L_On,bcd_value_minutes_H_On;
char bcd_value_hours_L_On,bcd_value_hours_H_On;
unsigned short seconds_reg_1_Off;
unsigned short minutes_reg_1_Off,hours_reg_1_Off;
unsigned short seconds_reg_2_Off;
unsigned short minutes_reg_2_Off,hours_reg_2_Off;
char bcd_value_seconds_L_Off,bcd_value_seconds_H_Off;
char bcd_value_minutes_L_Off,bcd_value_minutes_H_Off;
char bcd_value_hours_L_Off,bcd_value_hours_H_Off;
unsigned int CountSecondsRealTime=0;   // for secondsrealtime
unsigned int CountSecondsRealTimePv_ReConnect_T2=0,CountSecondsRealTimePv_ReConnect_T1=0,CountSecondsRealTimePv_ReConnect_T3=0;
unsigned int CountCutSecondsRealTime_T1=0,CountCutSecondsRealTime_T2=0; // time for cutting loads off
//-> eeprom variables
float numberFlash,numberFlash2,numberFlash3,numberFlash4;
float  flash_data[50];
uint32_t startTime;
bool insideSetup=0;
bool timerCounterStart_3=0,timerCounterStart_4=0,timerCounterStart_5=0; // these variables for starting the delay off timer
uint32_t currentMillis_1=0,previousMiliis_1=0;
uint32_t currentMillis_2=0,previousMiliis_2=0;
char usedInsideRTC=1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_IWDG_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void SetupProgram(void);
void CheckForSet(void);
void SetTimerOn_1(void);
void SetTimerOff_1(void);
void SetTimerOn_2(void);
void SetTimerOff_2(void);
void SetTimerOn_3();
void SetTimerOff_3();
void SetLowBatteryVoltage();
void SetStartUpLoadsVoltage();
void Startup_Timers();
void SetRTC_Time();
void Check_Timers();
void TurnLoadsOffWhenGridOff();
void CheckSystemBatteryMode();
void Screen_1();
void Flash_Save();  // save data to flash
void Flash_Load();  // read data from flash to variables
void Factory_Settings(); // factory settings
void SetUPSMode(); // to set ups mode
void SetVoltageMode();
void SetDS1307_Time(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year);
void ReadDS1307_Time();
uint8_t decToBcd(int val);
int bcdToDec(uint8_t val);
void DisplayTimeDS1307();
void SetDS1307(); // set ds1307 time in setup program
void SetDelayOff_Timers();
void ADC_Select_VDD(); // for selecting the reading of VREF
void ADC_Select_BATTERY(); // for selecting reading battery voltage
void CheckDS1307WorkingState();
void SelectRTC();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_time (void)
{

	if (usedInsideRTC==1)
	{
	  RTC_TimeTypeDef sTime = {0};
	  RTC_DateTypeDef DateToUpdate = {0};

	  sTime.Hours = 10;
	  sTime.Minutes =0;
	  sTime.Seconds = 0;

	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	  {
	    Error_Handler();
	  }
	 // DateToUpdate.WeekDay = 8;
	  DateToUpdate.Month = 8;
	  DateToUpdate.Date =19;
	  DateToUpdate.Year = 23;

	  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
	  {
	    Error_Handler();
	  }
  //HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);  // backup register
	} // end if useInsiderRTC
  /* USER CODE END RTC_Init 4 */
}


void get_time(void)
{

   if(usedInsideRTC==1){
  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

  /* Display time Format: hh:mm:ss */
  sprintf((char*)time,"%02d:%02d:%02d ",gTime.Hours, gTime.Minutes, gTime.Seconds);
   } // end if
}

//Let's display the time and date on lcd

void display_time (void)
{
	if(usedInsideRTC==1)
	{
	get_time();
	LCD16X2_Set_Cursor(MyLCD,1,1);
	LCD16X2_Write_String(MyLCD, "Time:");
	LCD16X2_Write_String(MyLCD,time);
	}

}




void set_alarm (void)
{
	  RTC_AlarmTypeDef sAlarm;

	   sAlarm.AlarmTime.Hours = 5;
	   sAlarm.AlarmTime.Minutes = 0;
	   sAlarm.AlarmTime.Seconds = 5;


	   sAlarm.Alarm = RTC_ALARM_A;
	   if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
	   {
	     Error_Handler();
	   }
	   /* USER CODE BEGIN RTC_Init 2 */

	   /* USER CODE END RTC_Init 2 */

	 }

//-----------------------------------DS1307 TIME--------------------------------------
void SetDS1307_Time (uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year)
{
	uint8_t set_time[7];
	set_time[0] = decToBcd(sec);
	set_time[1] = decToBcd(min);
	set_time[2] = decToBcd(hour);
	set_time[3] = decToBcd(dow);
	set_time[4] = decToBcd(dom);
	set_time[5] = decToBcd(month);
	set_time[6] = decToBcd(year);
	HAL_I2C_Mem_Write(&hi2c1, DS1307_ADDRESS, 0x00, 1, set_time, 7, 1000);
}


void ReadDS1307_Time (void)
{
	uint8_t get_time[7];
	HAL_I2C_Mem_Read(&hi2c1, DS1307_ADDRESS, 0x00, 1, get_time, 7, 1000);
	time_ds1307.seconds = bcdToDec(get_time[0]);
	time_ds1307.minutes = bcdToDec(get_time[1]);
	time_ds1307.hour = bcdToDec(get_time[2]);
	time_ds1307.dayofweek = bcdToDec(get_time[3]);
	time_ds1307.dayofmonth = bcdToDec(get_time[4]);
	time_ds1307.month = bcdToDec(get_time[5]);
	time_ds1307.year = bcdToDec(get_time[6]);
}


void DisplayTimeDS1307()
{
	ReadDS1307_Time();
	sprintf ((char*)time, "%02d:%02d:%02d ", time_ds1307.hour, time_ds1307.minutes, time_ds1307.seconds);
	LCD16X2_Set_Cursor(MyLCD,1,1);
	LCD16X2_Write_String(MyLCD, "Time:");
	LCD16X2_Write_String(MyLCD,time);

}


// Convert normal decimal numbers to binary coded decimal
uint8_t decToBcd(int val)
{
  return (uint8_t)( (val/10*16) + (val%10) );
}
// Convert binary coded decimal to normal decimal numbers
int bcdToDec(uint8_t val)
{
  return (int)( (val/16*10) + (val%16) );
}
//-----------------------------------Configs------------------------------------------
void Config()
{
HAL_ADCEx_Calibration_Start(&hadc);	// start calibration
//HAL_ADC_Start_DMA(&hadc1, &adc_buffer, 1);
HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, GPIO_PIN_SET); // turning on the backlight
LCD16X2_Init(MyLCD);
LCD16X2_Clear(MyLCD);
LCD16X2_Set_Cursor(MyLCD,1,5);
LCD16X2_Write_String(MyLCD,"SLC PLUS");
LCD16X2_Set_Cursor(MyLCD,2,6);
LCD16X2_Write_String(MyLCD," V1.3");
//lcd_init();
//lcd_puts(0,0," SLC PLUS V1.0 " );
HAL_Delay(2000);
LCD16X2_Clear(MyLCD);
//lcd_clear();

}
//----------------------------------LCD_CLEAR------------------------------------------
void LCD_Clear(unsigned short Row, unsigned short Start, unsigned short End)
{
  unsigned short Column;
  for(Column=Start; Column<=End; Column++)
  {
	  LCD16X2_Set_Cursor(MyLCD,Row,Column);
	  LCD16X2_Write_String(MyLCD,32);
   // lcd_puts(Row,Column,32);
  }
}
//--------------------------------Read Battery Voltage-----------------------------------
void Read_Battery()
{
float sum=0 , Battery[100];
char i=0, j=0;
float addError_1=0,addError_2=0,addError_3=0;
//--------------------------------------READ VDD-------------------------------------------
/*
ADC_Select_VDD();
HAL_ADC_Start(&hadc1); // START ADC CONVERSION
HAL_ADC_PollForConversion(&hadc1, 100); //// Poll ADC1 Perihperal & TimeOut = 1mSec
adc_vref=HAL_ADC_GetValue(&hadc1);
vref=(1.20/adc_vref)*4096.0;
sprintf(buffer,"V=%4.3f",vref);     // re format vin_battery to have 2 decimals
LCD16X2_Set_Cursor(MyLCD,2,8);
LCD16X2_Write_String(MyLCD,buffer);
*/

//-------------------------------------READ BATTERY----------------------------------------
for ( i=0; i<100 ; i++)
{

HAL_ADC_Start(&hadc); // START ADC CONVERSION
HAL_ADC_PollForConversion(&hadc, 100); //// Poll ADC1 Perihperal & TimeOut = 1mSec
adc_value=HAL_ADC_GetValue(&hadc);
Battery_Voltage=(adc_value*3.3)/4095.0;  //3300/4096
Battery[i]=((10.5/0.5)*Battery_Voltage);
HAL_Delay(10);
sum+=Battery[i];
}
//error handling
if (HAL_GPIO_ReadPin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin)==1) addError_1=0.07; else if (HAL_GPIO_ReadPin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin)==0) addError_1=0;
if (HAL_GPIO_ReadPin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin)==1) addError_2=0.07; else if (HAL_GPIO_ReadPin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin)==0) addError_2=0;
if (HAL_GPIO_ReadPin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin)==1) addError_3=0.07; else if (HAL_GPIO_ReadPin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin)==0) addError_3=0;

Vin_Battery= (sum/100.0) + addError_1+ addError_2 +addError_3;
sprintf(buffer,"V=%4.1f",Vin_Battery);     // re format vin_battery to have 2 decimals
LCD16X2_Set_Cursor(MyLCD,2,1);
LCD16X2_Write_String(MyLCD,buffer);
}
//****************************INTERRUPTS----------------------------------------------------
//----------------------------ADC INTERRUPTS------------------------------------------------
 void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
 //adc_value=adc_buffer;
  /* NOTE : This function should not be modified. When the callback is needed,
            function HAL_ADC_ConvCpltCallback must be implemented in the user file.
   */
}
 //--------------------------TIMER INTERRUPT-----------------------------------
 //-> start timer for battery low voltage
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 {

   /* Prevent unused argument(s) compilation warning */
    if (htim==&htim3 )
    {
		//- give some time to battery to turn off loads
    	Timer_Counter_3++;
     	Timer_Counter_4++;
    	Timer_Counter_5++;
	if (Timer_Counter_3>=delayTimerOff_1)              // more than 10 seconds
	{

	if(Vin_Battery<Mini_Battery_Voltage && Ac_Available==1 && RunLoadsByBass==0 )
	{
	SecondsRealTime=0;
	CountCutSecondsRealTime_T1=0;
	SecondsRealTimePv_ReConnect_T1=0;
	HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_RESET);

	}
	Timer_Counter_3=0;
	//HAL_TIM_Base_Stop_IT(&htim2);

	}   // end if timer_counter_3

	//- give some time to battery to turn off loads
	if (Timer_Counter_4>=delayTimerOff_2)              // more than 10 seconds
	{

	if(Vin_Battery<Mini_Battery_Voltage_T2 && Ac_Available==1  && RunLoadsByBass==0)
	{
	SecondsRealTime=0;
	CountSecondsRealTimePv_ReConnect_T2=0;
	SecondsRealTimePv_ReConnect_T2=0;
	HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_RESET);

	}
	Timer_Counter_4=0;
	//HAL_TIM_Base_Stop_IT(&htim2);
	}  // end if timer_counter_4

	//- give some time to battery to turn off loads
	if (Timer_Counter_5>=delayTimerOff_3)              // more than 10 seconds
	{

	if(Vin_Battery<Mini_Battery_Voltage_T3 && Ac_Available==1  && RunLoadsByBass==0)
	{
	SecondsRealTime=0;
	CountSecondsRealTimePv_ReConnect_T3=0;
	SecondsRealTimePv_ReConnect_T3=0;
	HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_RESET);

	}
	Timer_Counter_5=0;
	//HAL_TIM_Base_Stop_IT(&htim2);
	}  // end if timer_counter_5

	//-> turn off timer if all timers are off
	if (Timer_Counter_3==0 && Timer_Counter_4==0 && Timer_Counter_5==0)
	{
		HAL_TIM_Base_Stop_IT(&htim3);
	}

    }
    //*********************************END TIMER 2************************************
    if(htim==&htim15  )
    {

    	UpdateScreenTime++;
    	if (CountSecondsRealTime==1) SecondsRealTime++;                                     // for counting real time for  grid count
    	if (CountSecondsRealTimePv_ReConnect_T1==1) SecondsRealTimePv_ReConnect_T1++; // for counting real time for pv connect
    	if (CountSecondsRealTimePv_ReConnect_T2==1) SecondsRealTimePv_ReConnect_T2++; // for counting real timer
    	if (CountSecondsRealTimePv_ReConnect_T3==1) SecondsRealTimePv_ReConnect_T3++; // for counting real timer
    	if (UpdateScreenTime==180)
    	{
    	// disable interrupts
    	HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, GPIO_PIN_RESET);
    	UpdateScreenTime=0;
    	LCD16X2_Clear(MyLCD);
    	LCD16X2_Init(MyLCD);
    	}
    }


   /* NOTE : This function should not be modified, when the callback is needed,
             the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
    */
 }



 //-------------------------------Read Seconds-----------------------------------
 unsigned short ReadSeconds()
 {

if (usedInsideRTC==1)
{
 RTC_TimeTypeDef gTime;
 HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
 Full_Seconds=gTime.Seconds;
 return Full_Seconds;
}
else
{
  ReadDS1307_Time();
 return time_ds1307.seconds;
 }
 }

 //-------------------------------Read Minutes-----------------------------------
 unsigned short ReadMinutes()
 {

	 if (usedInsideRTC==1)
	 {
 	RTC_TimeTypeDef gTime;
 	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
 	Full_Seconds=gTime.Minutes;
 	return Full_Seconds;
	 }
	 else
	 {
	 ReadDS1307_Time();
	 return time_ds1307.minutes;
	 }
 }
 //--------------------------------Read hours----------------------------------------
 unsigned short ReadHours()
 {
	 if(usedInsideRTC)
	 {
 	RTC_TimeTypeDef gTime;
 	HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
 	Full_Seconds=gTime.Hours;
 	return Full_Seconds;
	 }
	 else
	 {
	 ReadDS1307_Time();
	 return time_ds1307.hour;
	 }
 }

 //-----------------------------Check Time Occured-------------------------------
 char CheckTimeOccuredOn(char seconds_required, char minutes_required,char hours_required)
 {

	 if (usedInsideRTC==1) {

	  RTC_DateTypeDef gDate;
	  RTC_TimeTypeDef gTime;
	  // Get the RTC current Time
	  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	  // Get the RTC current Date
	  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

	if (gTime.Hours==hours_required && gTime.Minutes==minutes_required)
	{
	return 1;
	}
	else {
		return 0;
	}



	 } // end if
	 else {
		if (time_ds1307.hour==hours_required && time_ds1307.minutes==minutes_required)
		{
		return 1;
		}
		else {
			return 0;
		}
	 } // end else
 }
 //-> to check time off
 char CheckTimeOccuredOff(char seconds_required, char minutes_required,char hours_required)
 {
 if (usedInsideRTC==1) {

	  RTC_DateTypeDef gDate;
	  RTC_TimeTypeDef gTime;
	  // Get the RTC current Time
	  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	  // Get the RTC current Date
	  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

	if (gTime.Hours==hours_required && gTime.Minutes==minutes_required)
	{
	return 1;
	}
	else {
		return 0;
	}

 }
 else {
    // for ds1307
		if (gTime.Hours==time_ds1307.hour && time_ds1307.minutes==minutes_required)
		{
		return 1;
		}
		else {
			return 0;
		}
 } // end else
 }
 //-------------------------------CHECK TIMER IN RANGE--------------------------------------------
 //-------------------Check for timer activation inside range--------------------
 void CheckForTimerActivationInRange()
 {
 if (RunOnBatteryVoltageMode==0)
 {
 //-> first compare is hours
 if(ReadHours() > hours_lcd_1 && ReadHours()< hours_lcd_2)
 {
 Timer_isOn=1;
  }
 //-> seconds compare hours if equal now then compare minutes
 if(ReadHours()== hours_lcd_1 || ReadHours()== hours_lcd_2)
 {
 if(ReadHours()==hours_lcd_1)
 {
 //-> minutes must be bigger
 if(ReadMinutes()>=minutes_lcd_1) Timer_isOn=1;
 }
 if(ReadHours()==hours_lcd_2)
 {
 //-> minutes must be less
 if(ReadMinutes()< minutes_lcd_2) Timer_isOn=1;
 }
 }
 //------------------------------Timer 2-----------------------------------------
 if(ReadHours() > hours_lcd_timer2_start && ReadHours()< hours_lcd_timer2_stop)
 {
 Timer_2_isOn=1;
 }
 //-> seconds compare hours if equal now then compare minutes
 if(ReadHours()== hours_lcd_timer2_start || ReadHours()== hours_lcd_timer2_stop )
 {
 if(ReadHours()==hours_lcd_timer2_start)
 {
 //-> minutes must be bigger
 if(ReadMinutes()>=minutes_lcd_timer2_start) Timer_2_isOn=1;
 }
 if(ReadHours()==hours_lcd_timer2_stop)
 {
 //-> minutes must be less
 if(ReadMinutes()<minutes_lcd_timer2_stop) Timer_2_isOn=1;
 }
 }
 //-----------------------------Timer 3 ---------------------------------------
 if(ReadHours() > hours_lcd_timer3_start && ReadHours()< hours_lcd_timer3_stop)
 {
 Timer_3_isOn=1;
 }
 //-> seconds compare hours if equal now then compare minutes
 if(ReadHours()== hours_lcd_timer3_start || ReadHours()== hours_lcd_timer3_stop )
 {
 if(ReadHours()==hours_lcd_timer3_start)
 {
 //-> minutes must be bigger
 if(ReadMinutes()>=minutes_lcd_timer3_start) Timer_3_isOn=1;
 }
 if(ReadHours()==hours_lcd_timer3_stop)
 {
 //-> minutes must be less
 if(ReadMinutes()<minutes_lcd_timer3_stop) Timer_3_isOn=1;
 }
 }
 } // run on battery voltage mode
 }  // end function
 //**************************************1****************************************
 void CheckForTimerActivationOutRange()
 {
 if (RunOnBatteryVoltageMode==0)
 {
 //------------------------------First Timer-------------------------------------
 if (ReadHours() < hours_lcd_1  && ReadHours() < hours_lcd_2 )
 {
 Timer_isOn=0;
 }

 if (ReadHours() > hours_lcd_1  && ReadHours() > hours_lcd_2 )
 {
 Timer_isOn=0;
 }


 if (ReadHours()==hours_lcd_1)
 {
 if(ReadMinutes() < minutes_lcd_1)
 {
 Timer_isOn=0;
 }
 }
 //-> check for hours
 if (ReadHours()==hours_lcd_2)
 {
 if(ReadMinutes() > minutes_lcd_2)
 {
 Timer_isOn=0;
 }
 }
 //----------------------------Second Timer--------------------------------------
 if (ReadHours() < hours_lcd_timer2_start  && ReadHours() < hours_lcd_timer2_stop )
 {
 Timer_2_isOn=0;
 }

 if (ReadHours() > hours_lcd_timer2_start  && ReadHours() > hours_lcd_timer2_stop )
 {
 Timer_2_isOn=0;
 }


 if (ReadHours()==hours_lcd_timer2_start)
 {
 if(ReadMinutes() < minutes_lcd_timer2_start)
 {
 Timer_2_isOn=0;
 }
 }
 //-> check for hours
 if (ReadHours()==hours_lcd_timer2_stop)
 {
 if(ReadMinutes() > minutes_lcd_timer2_stop)
 {
 Timer_2_isOn=0;
 }
 }
 //--------------------------------TIMER 3 ----------------------------
  if (ReadHours() < hours_lcd_timer3_start  && ReadHours() < hours_lcd_timer3_stop )
  {
  Timer_3_isOn=0;
  }

  if (ReadHours() > hours_lcd_timer3_start  && ReadHours() > hours_lcd_timer3_stop )
  {
  Timer_3_isOn=0;
  }


  if (ReadHours()==hours_lcd_timer3_start)
  {
  if(ReadMinutes() < minutes_lcd_timer3_start)
  {
  Timer_3_isOn=0;
  }
  }
  //-> check for hours
  if (ReadHours()==hours_lcd_timer3_stop)
  {
  if(ReadMinutes() > minutes_lcd_timer3_stop)
  {
  Timer_3_isOn=0;
  }
  }
 } // end of run on battery voltage
 }
 //--------------------------Save Data to Flash----------------------
void Flash_Save()
{

	flash_data[0]=hours_lcd_1;
	flash_data[1]=minutes_lcd_1;
	flash_data[2]=hours_lcd_2;
	flash_data[3]=minutes_lcd_2;
	flash_data[4]=hours_lcd_timer2_start;
	flash_data[5]=minutes_lcd_timer2_start;
	flash_data[6]=hours_lcd_timer2_stop;
	flash_data[7]=minutes_lcd_timer2_stop;
	flash_data[8]=hours_lcd_timer3_start;
	flash_data[9]=minutes_lcd_timer3_start;
	flash_data[10]=hours_lcd_timer3_stop;
	flash_data[11]=minutes_lcd_timer3_stop;
	flash_data[12]=Mini_Battery_Voltage;
	flash_data[13]=Mini_Battery_Voltage_T2;
	flash_data[14]=Mini_Battery_Voltage_T3;
	flash_data[15]=StartLoadsVoltage;
	flash_data[16]=StartLoadsVoltage_T2;
	flash_data[17]=StartLoadsVoltage_T3;
	flash_data[18]=startupTIme_1;
	flash_data[19]=startupTIme_2;
	flash_data[20]=startupTIme_3;
	flash_data[21]=RunOnBatteryVoltageMode;
	flash_data[22]=UPSMode;
	flash_data[23]=delayTimerOff_1;
	flash_data[24]=delayTimerOff_2;
	flash_data[25]=delayTimerOff_3;
	flash_data[26]=usedInsideRTC;
	Flash_Write_Data(Flash_Address, flash_data, 27);
}
//-> to read flash contents to variable
void Flash_Load()
{

Flash_Read_Data(Flash_Address, flash_data, 27);
hours_lcd_1=flash_data[0];
minutes_lcd_1=flash_data[1];
hours_lcd_2=flash_data[2];
minutes_lcd_2=flash_data[3];
hours_lcd_timer2_start=flash_data[4];
minutes_lcd_timer2_start=flash_data[5];
hours_lcd_timer2_stop=flash_data[6];
minutes_lcd_timer2_stop=flash_data[7];
hours_lcd_timer3_start=flash_data[8];
minutes_lcd_timer3_start=flash_data[9];
hours_lcd_timer3_stop=flash_data[10];
minutes_lcd_timer3_stop=flash_data[11];
Mini_Battery_Voltage=flash_data[12];
Mini_Battery_Voltage_T2=flash_data[13];
Mini_Battery_Voltage_T3=flash_data[14];
StartLoadsVoltage=flash_data[15];
StartLoadsVoltage_T2=flash_data[16];
StartLoadsVoltage_T3=flash_data[17];
startupTIme_1=flash_data[18];
startupTIme_2=flash_data[19];
startupTIme_3=flash_data[20];
RunOnBatteryVoltageMode=flash_data[21];
UPSMode=flash_data[22];
delayTimerOff_1=flash_data[23];
delayTimerOff_2=flash_data[24];
delayTimerOff_3=flash_data[25];
usedInsideRTC=flash_data[26];

}

//------------------------------------Interrupt for  Enter Button ---------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{


  /* Prevent unused argument(s) compilation warning */
     //----------------------------------------------SETUP PROGRAM--------------------------------------------------------
	if (GPIO_Pin==Enter_Pin)
	{
		HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, GPIO_PIN_SET);
		UpdateScreenTime=0;
	}
	//-------------------------------------------Ac_Available_INTRUPPT----------------------------------------------------
	if (GPIO_Pin==Ac_Available_Pin)
	{

		 //-> functions for shutting down loads if there is no timers and grid is off
		if(Ac_Available==GPIO_PIN_SET && Timer_isOn==0  && RunLoadsByBass==0 && RunOnBatteryVoltageMode==0)
		{

		SecondsRealTime=0;
		CountSecondsRealTime=0;
		HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_RESET);

		}

		if (Ac_Available==GPIO_PIN_SET && Timer_2_isOn==0 && RunLoadsByBass==0 && RunOnBatteryVoltageMode==0)  // it must be   Timer_2_isOn==0    but because of error in loading eeprom value
		{
			CountSecondsRealTime=0;
			SecondsRealTime=0;
		HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_RESET);

		}


		if (Ac_Available==GPIO_PIN_SET && Timer_3_isOn==0 && RunLoadsByBass==0 && RunOnBatteryVoltageMode==0)  // it must be   Timer_2_isOn==0    but because of error in loading eeprom value
		{
			CountSecondsRealTime=0;
			SecondsRealTime=0;
		    HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_RESET);

		}



		if (Ac_Available==GPIO_PIN_SET &&  RunLoadsByBass==0 && UPSMode==1 && LoadsAlreadySwitchedOFF==1)
		{
			LoadsAlreadySwitchedOFF=0;
			SecondsRealTime=0;
			SecondsRealTimePv_ReConnect_T1=0;
			SecondsRealTimePv_ReConnect_T2=0;
			SecondsRealTimePv_ReConnect_T3=0;
			CountSecondsRealTime=0;
			CountSecondsRealTimePv_ReConnect_T1=0;
			CountSecondsRealTimePv_ReConnect_T2=0;
			CountSecondsRealTimePv_ReConnect_T3=0;
			HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_RESET);
	     //	LCD_Clear(2,7,16); // to clear lcd when grid is not available
	  	}

	    	//LCD16X2_Clear(MyLCD);
         //  	LCD16X2_Init(MyLCD);
		}
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}
//----------------------------SetUpProgram-------------------------------
void SetupProgram()
{

LCD16X2_Clear(MyLCD);
LCD16X2_Set_Cursor(MyLCD,1,1);
LCD16X2_Write_String(MyLCD,"Setup Program");
HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // turn off flashing led
HAL_Delay(1000);

while(HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET )
{
insideSetup=1;
HAL_Delay(500);
SetTimerOn_1();
if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break;
//SetTimerOff_1();
if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break;
//SetTimerOn_2();
if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break;
//SetTimerOff_2();
if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break;
//SetTimerOn_3();
if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break;
//SetTimerOff_3();
if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break;
SetLowBatteryVoltage();
if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break;
SetStartUpLoadsVoltage();
if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break;
//Startup_Timers();
if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break;
//SetDelayOff_Timers();
if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break;
//SetVoltageMode();
if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break;
//SetUPSMode();
if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break;
SelectRTC();
if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break;
if(usedInsideRTC==0)SetDS1307(); else SetRTC_Time();
if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break;
LCD16X2_Clear(MyLCD);
HAL_Delay(500);
break;
}

insideSetup=0;
}
//----------------------------Set Timer 1 ----------------------------------
void SetTimerOn_1()
{

HAL_Delay(500);
currentMillis_1=0,currentMillis_2=0;
previousMiliis_1=0,previousMiliis_2=0;
//LCD16X2_Clear(MyLCD);
while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
		&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)

{

    HAL_IWDG_Refresh(&hiwdg);


	 currentMillis_2=HAL_GetTick();
	 if(currentMillis_2-previousMiliis_2 >=1000)
	 {
	 previousMiliis_2=currentMillis_2;
	 sprintf((char*)txt,"[1] H:%02d-M:%02d",hours_lcd_1,minutes_lcd_1);
	 LCD16X2_Set_Cursor(MyLCD,1,1);
	 LCD16X2_Write_String(MyLCD,txt);
	 }

	 currentMillis_1=HAL_GetTick();
	 if(currentMillis_1-previousMiliis_1 >=2000)
	 {
	 previousMiliis_1=currentMillis_1;
	 LCD16X2_Set_Cursor(MyLCD,1,10);
	 LCD16X2_Write_String(MyLCD,"    ");
	 }
	 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
		|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
{
	 sprintf((char*)txt,"[1] H:%02d-M:%02d",hours_lcd_1,minutes_lcd_1);
	 LCD16X2_Set_Cursor(MyLCD,1,1);
	 LCD16X2_Write_String(MyLCD,txt);

	if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
	{
	HAL_Delay(ButtonDelay);
	minutes_lcd_1++;
	}
	if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
	{
	HAL_Delay(ButtonDelay);
	minutes_lcd_1--;
	}
	//-> perfect
	if (minutes_lcd_1>59)  minutes_lcd_1=0;
	if (minutes_lcd_1<0) minutes_lcd_1=0;
	Timer_isOn=0; //
	SecondsRealTimePv_ReConnect_T1=0;
	}    //end while incremet and decremet

}   // end while Enter
currentMillis_1=0,currentMillis_2=0;
previousMiliis_1=0,previousMiliis_2=0;
HAL_Delay(500);
while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
	&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
{


     HAL_IWDG_Refresh(&hiwdg);
	 currentMillis_2=HAL_GetTick();
	 if(currentMillis_2-previousMiliis_2 >=1000)
	 {
	 previousMiliis_2=currentMillis_2;
	 sprintf((char*)txt,"[1] H:%02d-M:%02d",hours_lcd_1,minutes_lcd_1);
	 LCD16X2_Set_Cursor(MyLCD,1,1);
	 LCD16X2_Write_String(MyLCD,txt);
	 }


	 currentMillis_1=HAL_GetTick();
	 if(currentMillis_1-previousMiliis_1 >=2000)
	 {
	 previousMiliis_1=currentMillis_1;
	 //-> to convert to seconds previousMiliis/1000
	 LCD16X2_Set_Cursor(MyLCD,1,5);
	 LCD16X2_Write_String(MyLCD,"    ");
	 }


	 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
		|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
{

	 sprintf((char*)txt,"[1] H:%02d-M:%02d",hours_lcd_1,minutes_lcd_1);
	 LCD16X2_Set_Cursor(MyLCD,1,1);
	 LCD16X2_Write_String(MyLCD,txt);

	if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
	{
	HAL_Delay(ButtonDelay);
	hours_lcd_1++;
	}
	if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
	{
	HAL_Delay(ButtonDelay);
	hours_lcd_1--;
	}
	//-> perfect
	if (hours_lcd_1>23)  hours_lcd_1=0;
	if (hours_lcd_1<0) hours_lcd_1=0;
	SecondsRealTimePv_ReConnect_T1=0;
	Timer_isOn=0;
	}    //end while incremet and decremet

}
//-> save to epprom
Flash_Save();
//lcd_clear();
LCD16X2_Clear(MyLCD);
}  // end function
//--------------------------------Set Timer 1 Off------------------------------------------------------
void SetTimerOff_1()
{

	currentMillis_1=0,currentMillis_2=0;
	previousMiliis_1=0,previousMiliis_2=0;
	HAL_Delay(500);
	while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
		&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
	{

	    HAL_IWDG_Refresh(&hiwdg);

		 currentMillis_2=HAL_GetTick();
		 if(currentMillis_2-previousMiliis_2 >=1000)
		 {
		 previousMiliis_2=currentMillis_2;
		 sprintf((char*)txt,"[2] H:%02d-M:%02d",hours_lcd_2,minutes_lcd_2);
		 LCD16X2_Set_Cursor(MyLCD,1,1);
		 LCD16X2_Write_String(MyLCD,txt);
		 }


		 currentMillis_1=HAL_GetTick();
		 if(currentMillis_1-previousMiliis_1 >=2000)
		 {
		 previousMiliis_1=currentMillis_1;
		 LCD16X2_Set_Cursor(MyLCD,1,10);
		 LCD16X2_Write_String(MyLCD,"    ");
		 }


		 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

	while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
			|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
	{

		         sprintf((char*)txt,"[2] H:%02d-M:%02d",hours_lcd_2,minutes_lcd_2);
				 LCD16X2_Set_Cursor(MyLCD,1,1);
				 LCD16X2_Write_String(MyLCD,txt);

		if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
		{
		HAL_Delay(ButtonDelay);
		minutes_lcd_2++;
		}
		if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
		{
		HAL_Delay(ButtonDelay);
		minutes_lcd_2--;
		}
		//-> perfect
		if (minutes_lcd_2>59)  minutes_lcd_2=0;
		if (minutes_lcd_2<0) minutes_lcd_2=0;

		}    //end while incremet and decremet

	}   // end while Enter
	currentMillis_1=0,currentMillis_2=0;
	previousMiliis_1=0,previousMiliis_2=0;
	HAL_Delay(500);
	while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
			&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
	{

	    HAL_IWDG_Refresh(&hiwdg);


         currentMillis_2=HAL_GetTick();
		 if(currentMillis_2-previousMiliis_2 >=1000)
		 {
		 previousMiliis_2=currentMillis_2;
		 sprintf((char*)txt,"[2] H:%02d-M:%02d",hours_lcd_2,minutes_lcd_2);
		 LCD16X2_Set_Cursor(MyLCD,1,1);
		 LCD16X2_Write_String(MyLCD,txt);
		 }


		 currentMillis_1=HAL_GetTick();
		 if(currentMillis_1-previousMiliis_1 >=2000)
		 {
		 previousMiliis_1=currentMillis_1;
		 LCD16X2_Set_Cursor(MyLCD,1,5);
		 LCD16X2_Write_String(MyLCD,"    ");
		 }

		 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;  // to break the loop


	while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
			|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
	{

		 sprintf((char*)txt,"[2] H:%02d-M:%02d",hours_lcd_2,minutes_lcd_2);
		 LCD16X2_Set_Cursor(MyLCD,1,1);
		 LCD16X2_Write_String(MyLCD,txt);
		if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
		{
		HAL_Delay(ButtonDelay);
		hours_lcd_2++;
		}
		if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
		{
		HAL_Delay(ButtonDelay);
		hours_lcd_2--;
		}
		//-> perfect
		if (hours_lcd_2>23)  hours_lcd_2=0;
		if (hours_lcd_2<0) hours_lcd_2=0;

		}    //end while incremet and decremet
	}
	//-> save to epprom
	Flash_Save();
	//lcd_clear();
  	LCD16X2_Clear(MyLCD);
}
//---------------------------------Set Timer 2 On---------------------------------------------------
void SetTimerOn_2()
{
	//lcd_clear();
	//LCD16X2_Clear(MyLCD);
	currentMillis_1=0,currentMillis_2=0;
	previousMiliis_1=0,previousMiliis_2=0;
	HAL_Delay(500);
	while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
			&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
	{

	    HAL_IWDG_Refresh(&hiwdg);

	          currentMillis_2=HAL_GetTick();
			 if(currentMillis_2-previousMiliis_2 >=1000)
			 {
			 previousMiliis_2=currentMillis_2;
			 sprintf((char*)txt,"[3] H:%02d-M:%02d",hours_lcd_timer2_start,minutes_lcd_timer2_start);
			 LCD16X2_Set_Cursor(MyLCD,1,1);
			 LCD16X2_Write_String(MyLCD,txt);
			 }


			 currentMillis_1=HAL_GetTick();
			 if(currentMillis_1-previousMiliis_1 >=2000)
			 {
			 previousMiliis_1=currentMillis_1;
			 LCD16X2_Set_Cursor(MyLCD,1,10);
			 LCD16X2_Write_String(MyLCD,"    ");
			 }

		 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

	while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
			|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
	{

		 sprintf((char*)txt,"[3] H:%02d-M:%02d",hours_lcd_timer2_start,minutes_lcd_timer2_start);
		 LCD16X2_Set_Cursor(MyLCD,1,1);
		 LCD16X2_Write_String(MyLCD,txt);

		if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
		{
		HAL_Delay(ButtonDelay);
		minutes_lcd_timer2_start++;
		}
		if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
		{
		HAL_Delay(ButtonDelay);
		minutes_lcd_timer2_start--;
		}
		//-> perfect
		if (minutes_lcd_timer2_start>59)  minutes_lcd_timer2_start=0;
		if (minutes_lcd_timer2_start<0) minutes_lcd_timer2_start=0;
		Timer_2_isOn=0;
		SecondsRealTimePv_ReConnect_T2=0;
		}    //end while incremet and decremet

	}   // end while Enter
	currentMillis_1=0,currentMillis_2=0;
	previousMiliis_1=0,previousMiliis_2=0;
	HAL_Delay(500);
	while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
			&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
	{

	     HAL_IWDG_Refresh(&hiwdg);

	     currentMillis_2=HAL_GetTick();
		 if(currentMillis_2-previousMiliis_2 >=1000)
		 {
		 previousMiliis_2=currentMillis_2;
		 sprintf((char*)txt,"[3] H:%02d-M:%02d",hours_lcd_timer2_start,minutes_lcd_timer2_start);
		 LCD16X2_Set_Cursor(MyLCD,1,1);
		 LCD16X2_Write_String(MyLCD,txt);
		 }


		 currentMillis_1=HAL_GetTick();
		 if(currentMillis_1-previousMiliis_1 >=2000)
		 {
		 previousMiliis_1=currentMillis_1;
		 LCD16X2_Set_Cursor(MyLCD,1,5);
		 LCD16X2_Write_String(MyLCD,"    ");
		 }

		 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

	while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
			|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
	{

		 sprintf((char*)txt,"[3] H:%02d-M:%02d",hours_lcd_timer2_start,minutes_lcd_timer2_start);
		 LCD16X2_Set_Cursor(MyLCD,1,1);
		 LCD16X2_Write_String(MyLCD,txt);

		if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
		{
		HAL_Delay(ButtonDelay);
		hours_lcd_timer2_start++;
		}
		if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
		{
		HAL_Delay(ButtonDelay);
		hours_lcd_timer2_start--;
		}
		//-> perfect
		if (hours_lcd_timer2_start>23)  hours_lcd_timer2_start=0;
		if (hours_lcd_timer2_start<0) hours_lcd_timer2_start=0;
		Timer_2_isOn=0;
		SecondsRealTimePv_ReConnect_T2=0;
		}    //end while incremet and decremet
	}
	//-> save to epprom
	Flash_Save();
//	lcd_clear();
	LCD16X2_Clear(MyLCD);
	}  // end function
//---------------------------------Set Timer 2 Off----------------------------------------------------
void SetTimerOff_2()
{
	//lcd_clear();
//	LCD16X2_Clear(MyLCD);
	currentMillis_1=0,currentMillis_2=0;
	previousMiliis_1=0,previousMiliis_2=0;
	HAL_Delay(500);
	while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
			&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
	{

	     HAL_IWDG_Refresh(&hiwdg);


	         currentMillis_2=HAL_GetTick();
			 if(currentMillis_2-previousMiliis_2 >=1000)
			 {
			 previousMiliis_2=currentMillis_2;
			 sprintf((char*)txt,"[4] H:%02d-M:%02d",hours_lcd_timer2_stop,minutes_lcd_timer2_stop);
			 LCD16X2_Set_Cursor(MyLCD,1,1);
			 LCD16X2_Write_String(MyLCD,txt);
			 }


			 currentMillis_1=HAL_GetTick();
			 if(currentMillis_1-previousMiliis_1 >=2000)
			 {
			 previousMiliis_1=currentMillis_1;
			 LCD16X2_Set_Cursor(MyLCD,1,10);
			 LCD16X2_Write_String(MyLCD,"    ");
			 }

		 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

	while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
			|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
	{

		 sprintf((char*)txt,"[4] H:%02d-M:%02d",hours_lcd_timer2_stop,minutes_lcd_timer2_stop);
		 LCD16X2_Set_Cursor(MyLCD,1,1);
		 LCD16X2_Write_String(MyLCD,txt);

		if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
		{
		HAL_Delay(ButtonDelay);
		minutes_lcd_timer2_stop++;
		}
		if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
		{
		HAL_Delay(ButtonDelay);
		minutes_lcd_timer2_stop--;
		}
		//-> perfect
		if (minutes_lcd_timer2_stop>59)  minutes_lcd_timer2_stop=0;
		if (minutes_lcd_timer2_stop<0) minutes_lcd_timer2_stop=0;
		Timer_2_isOn=0;
		SecondsRealTimePv_ReConnect_T2=0;
		}    //end while incremet and decremet

	}   // end while Enter
	currentMillis_1=0,currentMillis_2=0;
	previousMiliis_1=0,previousMiliis_2=0;
	HAL_Delay(500);
	while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
			&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
	{

	    HAL_IWDG_Refresh(&hiwdg);

	     currentMillis_2=HAL_GetTick();
		 if(currentMillis_2-previousMiliis_2 >=1000)
		 {
		 previousMiliis_2=currentMillis_2;
		 sprintf((char*)txt,"[4] H:%02d-M:%02d",hours_lcd_timer2_stop,minutes_lcd_timer2_stop);
		 LCD16X2_Set_Cursor(MyLCD,1,1);
		 LCD16X2_Write_String(MyLCD,txt);
		 }


		 currentMillis_1=HAL_GetTick();
		 if(currentMillis_1-previousMiliis_1 >=2000)
		 {
		 previousMiliis_1=currentMillis_1;
		 LCD16X2_Set_Cursor(MyLCD,1,5);
		 LCD16X2_Write_String(MyLCD,"    ");
		 }


		 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

	while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
			|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
	{

		 sprintf((char*)txt,"[4] H:%02d-M:%02d",hours_lcd_timer2_stop,minutes_lcd_timer2_stop);
		 LCD16X2_Set_Cursor(MyLCD,1,1);
		 LCD16X2_Write_String(MyLCD,txt);

		if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
		{
		HAL_Delay(ButtonDelay);
		hours_lcd_timer2_stop++;
		}
		if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
		{
		HAL_Delay(ButtonDelay);
		hours_lcd_timer2_stop--;
		}
		//-> perfect
		if (hours_lcd_timer2_stop>23)  hours_lcd_timer2_stop=0;
		if (hours_lcd_timer2_stop<0) hours_lcd_timer2_stop=0;
		Timer_2_isOn=0;
		SecondsRealTimePv_ReConnect_T2=0;
		}    //end while incremet and decremet
	}
	//-> save to epprom
	Flash_Save();
	//lcd_clear();
	LCD16X2_Clear(MyLCD);
}
//---------------------------------Set Timer 3 On----------------------------------------------------
void SetTimerOn_3()
{
	    //lcd_clear();
	   // LCD16X2_Clear(MyLCD);

     	currentMillis_1=0,currentMillis_2=0;
	    previousMiliis_1=0,previousMiliis_2=0;
		HAL_Delay(500);
		while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
				&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
		{

		    HAL_IWDG_Refresh(&hiwdg);

			     currentMillis_2=HAL_GetTick();
				 if(currentMillis_2-previousMiliis_2 >=1000)
				 {
				 previousMiliis_2=currentMillis_2;
				 sprintf((char*)txt,"[5] H:%02d-M:%02d",hours_lcd_timer3_start,minutes_lcd_timer3_start);
				 LCD16X2_Set_Cursor(MyLCD,1,1);
				 LCD16X2_Write_String(MyLCD,txt);
				 }


				 currentMillis_1=HAL_GetTick();
				 if(currentMillis_1-previousMiliis_1 >=2000)
				 {
				 previousMiliis_1=currentMillis_1;
				 LCD16X2_Set_Cursor(MyLCD,1,10);
				 LCD16X2_Write_String(MyLCD,"    ");
				 }

			 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

		while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
				|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
		{
			 sprintf((char*)txt,"[5] H:%02d-M:%02d",hours_lcd_timer3_start,minutes_lcd_timer3_start);
			 LCD16X2_Set_Cursor(MyLCD,1,1);
			 LCD16X2_Write_String(MyLCD,txt);

			if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
			{
			HAL_Delay(ButtonDelay);
			minutes_lcd_timer3_start++;
			}
			if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
			{
			HAL_Delay(ButtonDelay);
			minutes_lcd_timer3_start--;
			}
			//-> perfect
			if (minutes_lcd_timer3_start>59)  minutes_lcd_timer3_start=0;
			if (minutes_lcd_timer3_start<0) minutes_lcd_timer3_start=0;
			Timer_3_isOn=0;
			SecondsRealTimePv_ReConnect_T3=0;
			}    //end while incremet and decremet

		}   // end while Enter
		currentMillis_1=0,currentMillis_2=0;
		previousMiliis_1=0,previousMiliis_2=0;
		HAL_Delay(500);
		while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
				&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
		{

		    HAL_IWDG_Refresh(&hiwdg);

		     currentMillis_2=HAL_GetTick();
			 if(currentMillis_2-previousMiliis_2 >=1000)
			 {
			 previousMiliis_2=currentMillis_2;
			 sprintf((char*)txt,"[5] H:%02d-M:%02d",hours_lcd_timer3_start,minutes_lcd_timer3_start);
			 LCD16X2_Set_Cursor(MyLCD,1,1);
			 LCD16X2_Write_String(MyLCD,txt);
			 }


			 currentMillis_1=HAL_GetTick();
			 if(currentMillis_1-previousMiliis_1 >=2000)
			 {
			 previousMiliis_1=currentMillis_1;
			 LCD16X2_Set_Cursor(MyLCD,1,5);
			 LCD16X2_Write_String(MyLCD,"    ");
			 }

			 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

		while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
				|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
		{

			 sprintf((char*)txt,"[5] H:%02d-M:%02d",hours_lcd_timer3_start,minutes_lcd_timer3_start);
			 LCD16X2_Set_Cursor(MyLCD,1,1);
			 LCD16X2_Write_String(MyLCD,txt);

			if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
			{
			HAL_Delay(ButtonDelay);
			hours_lcd_timer3_start++;
			}
			if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
			{
			HAL_Delay(ButtonDelay);
			hours_lcd_timer3_start--;
			}
			//-> perfect
			if (hours_lcd_timer3_start>23)  hours_lcd_timer3_start=0;
			if (hours_lcd_timer3_start<0) hours_lcd_timer3_start=0;
			Timer_3_isOn=0;
			SecondsRealTimePv_ReConnect_T3=0;
			}    //end while incremet and decremet
		}
		//-> save to epprom
		Flash_Save();
	//	lcd_clear();
		LCD16X2_Clear(MyLCD);
}
//--------------------------------Set Timer 3 Off--------------------------------------------------
void SetTimerOff_3()
{
//	lcd_clear();
	//LCD16X2_Clear(MyLCD);
	currentMillis_1=0,currentMillis_2=0;
	previousMiliis_1=0,previousMiliis_2=0;
	HAL_Delay(500);
	while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
			&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
	{

	         HAL_IWDG_Refresh(&hiwdg);

		     currentMillis_2=HAL_GetTick();
			 if(currentMillis_2-previousMiliis_2 >=1000)
			 {
			 previousMiliis_2=currentMillis_2;
			 sprintf((char*)txt,"[6] H:%02d-M:%02d",hours_lcd_timer3_stop,minutes_lcd_timer3_stop);
			 LCD16X2_Set_Cursor(MyLCD,1,1);
			 LCD16X2_Write_String(MyLCD,txt);
			 }


			 currentMillis_1=HAL_GetTick();
			 if(currentMillis_1-previousMiliis_1 >=2000)
			 {
			 previousMiliis_1=currentMillis_1;
			 LCD16X2_Set_Cursor(MyLCD,1,10);
			 LCD16X2_Write_String(MyLCD,"    ");
			 }

		 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

	while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
			|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
	{

		 sprintf((char*)txt,"[6] H:%02d-M:%02d",hours_lcd_timer3_stop,minutes_lcd_timer3_stop);
		 LCD16X2_Set_Cursor(MyLCD,1,1);
		 LCD16X2_Write_String(MyLCD,txt);

		if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
		{
		HAL_Delay(ButtonDelay);
		minutes_lcd_timer3_stop++;
		}
		if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
		{
		HAL_Delay(ButtonDelay);
		minutes_lcd_timer3_stop--;
		}
		//-> perfect
		if (minutes_lcd_timer3_stop>59)  minutes_lcd_timer3_stop=0;
		if (minutes_lcd_timer3_stop<0) minutes_lcd_timer3_stop=0;
		Timer_3_isOn=0;
		SecondsRealTimePv_ReConnect_T3=0;
		}    //end while incremet and decremet

	}   // end while Enter
	currentMillis_1=0,currentMillis_2=0;
	previousMiliis_1=0,previousMiliis_2=0;
	HAL_Delay(500);
	while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
			&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
	{

	    HAL_IWDG_Refresh(&hiwdg);

	     currentMillis_2=HAL_GetTick();
		 if(currentMillis_2-previousMiliis_2 >=1000)
		 {
		 previousMiliis_2=currentMillis_2;
		 sprintf((char*)txt,"[6] H:%02d-M:%02d",hours_lcd_timer3_stop,minutes_lcd_timer3_stop);
		 LCD16X2_Set_Cursor(MyLCD,1,1);
		 LCD16X2_Write_String(MyLCD,txt);
		 }


		 currentMillis_1=HAL_GetTick();
		 if(currentMillis_1-previousMiliis_1 >=2000)
		 {
		 previousMiliis_1=currentMillis_1;
		 LCD16X2_Set_Cursor(MyLCD,1,5);
		 LCD16X2_Write_String(MyLCD,"    ");
		 }

		 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

	while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
			|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
	{

		 sprintf((char*)txt,"[6] H:%02d-M:%02d",hours_lcd_timer3_stop,minutes_lcd_timer3_stop);
		 LCD16X2_Set_Cursor(MyLCD,1,1);
		 LCD16X2_Write_String(MyLCD,txt);

		if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
		{
		HAL_Delay(ButtonDelay);
		hours_lcd_timer3_stop++;
		}
		if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
		{
		HAL_Delay(ButtonDelay);
		hours_lcd_timer3_stop--;
		}
		//-> perfect
		if (hours_lcd_timer3_stop>23)  hours_lcd_timer3_stop=0;
		if (hours_lcd_timer3_stop<0) hours_lcd_timer3_stop=0;
		Timer_3_isOn=0;
		SecondsRealTimePv_ReConnect_T3=0;
		}    //end while incremet and decremet
	}
	//-> save to epprom
	Flash_Save();
	//lcd_clear();
	LCD16X2_Clear(MyLCD);
}
//---------------------------------Set Low Battery Voltage-------------------------------------------
void SetLowBatteryVoltage()
{
	//lcd_clear();
	//LCD16X2_Clear(MyLCD);
	HAL_Delay(500);
	while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
			&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
		{

	    HAL_IWDG_Refresh(&hiwdg);
			    sprintf(txt,"[7] LV1 %4.1f V",Mini_Battery_Voltage);
				LCD16X2_Set_Cursor(MyLCD,1,1);
				LCD16X2_Write_String(MyLCD,txt);

			 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

		while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
				|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
		{
		    sprintf(txt,"[7] LV1 %4.1f V",Mini_Battery_Voltage);
			LCD16X2_Set_Cursor(MyLCD,1,1);
			LCD16X2_Write_String(MyLCD,txt);

			if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
			{
			HAL_Delay(100);
			Mini_Battery_Voltage+=0.1;
			}
			if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
			{
			HAL_Delay(100);
			Mini_Battery_Voltage-=0.1;
			}
			//-> perfect
			if (Mini_Battery_Voltage>65)  Mini_Battery_Voltage=0;
			if (Mini_Battery_Voltage<0) Mini_Battery_Voltage=0;

			}    //end while incremet and decremet

		}   // end while Enter
	    Flash_Save();
		HAL_Delay(500);
		while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
				&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
		{

		    HAL_IWDG_Refresh(&hiwdg);
			sprintf(txt,"[7] LV2 %4.1f V",Mini_Battery_Voltage_T2);
			LCD16X2_Set_Cursor(MyLCD,1,1);
			LCD16X2_Write_String(MyLCD,txt);

			 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

		while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
				|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
		{

			sprintf(txt,"[7] LV2 %4.1f V",Mini_Battery_Voltage_T2);
			LCD16X2_Set_Cursor(MyLCD,1,1);
			LCD16X2_Write_String(MyLCD,txt);

			if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
			{
			HAL_Delay(100);
			Mini_Battery_Voltage_T2+=0.1;
			}
			if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
			{
			HAL_Delay(100);
			Mini_Battery_Voltage_T2-=0.1;
			}
			//-> perfect
			if (Mini_Battery_Voltage_T2>65)  Mini_Battery_Voltage_T2=0;
			if (Mini_Battery_Voltage_T2<0) Mini_Battery_Voltage_T2=0;

			}    //end while incremet and decremet
		}

		// > timer 3
		        Flash_Save();
				HAL_Delay(500);
			/*	while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
						&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
				{

				    HAL_IWDG_Refresh(&hiwdg);
				   	sprintf(txt,"[7] LV3 %4.1f V",Mini_Battery_Voltage_T3);
					LCD16X2_Set_Cursor(MyLCD,1,1);
					LCD16X2_Write_String(MyLCD,txt);

					 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

				while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
						|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
				{

				   	sprintf(txt,"[7] LV3 %4.1f V",Mini_Battery_Voltage_T3);
					LCD16X2_Set_Cursor(MyLCD,1,1);
					LCD16X2_Write_String(MyLCD,txt);


					if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
					{
					HAL_Delay(100);
					Mini_Battery_Voltage_T3+=0.1;
					}
					if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
					{
					HAL_Delay(100);
					Mini_Battery_Voltage_T3-=0.1;
					}
					//-> perfect
					if (Mini_Battery_Voltage_T3>65)  Mini_Battery_Voltage_T3=0;
					if (Mini_Battery_Voltage_T3<0) Mini_Battery_Voltage_T3=0;

					}    //end while incremet and decremet
				}
				*/
		//-> save to epprom

		Flash_Save();
		//lcd_clear();
	   LCD16X2_Clear(MyLCD);
} // end function
//---------------------------------StartUpLoadsVoltage------------------------------------------------
void SetStartUpLoadsVoltage()
{
	//lcd_clear();
//	LCD16X2_Clear(MyLCD);
	HAL_Delay(500);
		while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
				&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
			{

		         HAL_IWDG_Refresh(&hiwdg);
			        sprintf(txt,"[8] HV1 %4.1f V",StartLoadsVoltage);
					LCD16X2_Set_Cursor(MyLCD,1,1);
					LCD16X2_Write_String(MyLCD,txt);

				 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

			while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
					|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
			{

		        sprintf(txt,"[8] HV1 %4.1f V",StartLoadsVoltage);
				LCD16X2_Set_Cursor(MyLCD,1,1);
				LCD16X2_Write_String(MyLCD,txt);

				if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
				{
				HAL_Delay(100);
				StartLoadsVoltage+=0.1;
				}
				if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
				{
				HAL_Delay(100);
				StartLoadsVoltage-=0.1;
				}
				//-> perfect
				if (StartLoadsVoltage>65)  StartLoadsVoltage=0;
				if (StartLoadsVoltage<0) StartLoadsVoltage=0;

				}    //end while incremet and decremet

			}   // end while Enter
		//-> save to Flash
        Flash_Save();
			HAL_Delay(500);
			while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
					&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
			{
			    HAL_IWDG_Refresh(&hiwdg);

				sprintf(txt,"[8] HV2 %4.1f V",StartLoadsVoltage_T2);
				LCD16X2_Set_Cursor(MyLCD,1,1);
				LCD16X2_Write_String(MyLCD,txt);

				 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

			while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
					|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
			{

				sprintf(txt,"[8] HV2 %4.1f V",StartLoadsVoltage_T2);
				LCD16X2_Set_Cursor(MyLCD,1,1);
				LCD16X2_Write_String(MyLCD,txt);


				if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
				{
				HAL_Delay(100);
				StartLoadsVoltage_T2+=0.1;
				}
				if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
				{
				HAL_Delay(100);
				StartLoadsVoltage_T2-=0.1;
				}
				//-> perfect
				if (StartLoadsVoltage_T2>65)  StartLoadsVoltage_T2=0;
				if (StartLoadsVoltage_T2<0) StartLoadsVoltage_T2=0;

				}    //end while incremet and decremet
			}
			//-> timer 3
			Flash_Save();
						HAL_Delay(500);
						/*
						while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
								&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
						{

						    HAL_IWDG_Refresh(&hiwdg);
							sprintf(txt,"[8] HV3 %4.1f V",StartLoadsVoltage_T3);
							LCD16X2_Set_Cursor(MyLCD,1,1);
							LCD16X2_Write_String(MyLCD,txt);

							 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

						while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
								|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
						{

							sprintf(txt,"[8] HV3 %4.1f V",StartLoadsVoltage_T3);
							LCD16X2_Set_Cursor(MyLCD,1,1);
							LCD16X2_Write_String(MyLCD,txt);

							if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
							{
							HAL_Delay(100);
							StartLoadsVoltage_T3+=0.1;
							}
							if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
							{
							HAL_Delay(100);
							StartLoadsVoltage_T3-=0.1;
							}
							//-> perfect
							if (StartLoadsVoltage_T3>65)  StartLoadsVoltage_T3=0;
							if (StartLoadsVoltage_T3<0) StartLoadsVoltage_T3=0;

							}    //end while incremet and decremet
						}
						*/
						//-> save to epprom
						Flash_Save();
				//		lcd_clear();
						LCD16X2_Clear(MyLCD);

}  // end function
//---------------------------------StartUp Timers---------------------------------------------------
void Startup_Timers()
{
	  //  lcd_clear();
	//	LCD16X2_Clear(MyLCD);
		HAL_Delay(500);
		while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
				&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
		{

		     HAL_IWDG_Refresh(&hiwdg);
			 sprintf(txt,"[9]T1 ON %02d S  ",startupTIme_1);
			 LCD16X2_Set_Cursor(MyLCD,1,1);
			 LCD16X2_Write_String(MyLCD,txt);

			 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

		while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
				|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
		{

			 sprintf(txt,"[9]T1 ON %02d S  ",startupTIme_1);
			 LCD16X2_Set_Cursor(MyLCD,1,1);
			 LCD16X2_Write_String(MyLCD,txt);

			if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
			{
			HAL_Delay(100);
			startupTIme_1++;
			}
			if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
			{
			HAL_Delay(100);
			startupTIme_1--;
			}
			//-> perfect
			if (startupTIme_1>900)  startupTIme_1=0;
			if (startupTIme_1<0) startupTIme_1=0;

			}    //end while incremet and decremet

		}   // end while Enter
		Flash_Save();
		HAL_Delay(500);
		while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
				&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
		{

		    HAL_IWDG_Refresh(&hiwdg);
			 sprintf((char*)txt,"[9]T2 ON %02d S  ",startupTIme_2);
			 LCD16X2_Set_Cursor(MyLCD,1,1);
			 LCD16X2_Write_String(MyLCD,txt);

			 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

		while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
				|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
		{

			 sprintf((char*)txt,"[9]T2 ON %02d S  ",startupTIme_2);
			 LCD16X2_Set_Cursor(MyLCD,1,1);
			 LCD16X2_Write_String(MyLCD,txt);


			if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
			{
			HAL_Delay(100);
			startupTIme_2++;
			}
			if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
			{
			HAL_Delay(100);
			startupTIme_2--;
			}
			//-> perfect
			if (startupTIme_2>900)  startupTIme_2=0;
			if (startupTIme_2<0) startupTIme_2=0;
					}    //end while incremet and decremet
		}

		//-> timer 3
		Flash_Save();

		HAL_Delay(500);
		/*
		while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
				&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
		{

		    HAL_IWDG_Refresh(&hiwdg);
			 sprintf((char*)txt,"[9]T3 ON %02d S  ",startupTIme_3);
			 LCD16X2_Set_Cursor(MyLCD,1,1);
			 LCD16X2_Write_String(MyLCD,txt);

			 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

		while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
				|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
		{

			 sprintf((char*)txt,"[9]T3 ON %02d S  ",startupTIme_3);
			 LCD16X2_Set_Cursor(MyLCD,1,1);
			 LCD16X2_Write_String(MyLCD,txt);

			if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
			{
			HAL_Delay(100);
			startupTIme_3++;
			}
			if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
			{
			HAL_Delay(100);
			startupTIme_3--;
			}
			//-> perfect
			if (startupTIme_3>900)  startupTIme_3=0;
			if (startupTIme_3<0) startupTIme_3=0;
					}    //end while incremet and decremet
		}
		*/
		//-> save to epprom
		Flash_Save();
		//lcd_clear();
		LCD16X2_Clear(MyLCD);

} // end function
//--------------------------------Delay OFF Timer-------------------------------------
void SetDelayOff_Timers()
{
	HAL_Delay(500);
			while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
					&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
			{

			    HAL_IWDG_Refresh(&hiwdg);
				 sprintf(txt,"[10]T1 OFF %02d S  ",delayTimerOff_1);
				 LCD16X2_Set_Cursor(MyLCD,1,1);
				 LCD16X2_Write_String(MyLCD,txt);

				 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

			while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
					|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
			{

				 sprintf(txt,"[10]T1 OFF %02d S  ",delayTimerOff_1);
				 LCD16X2_Set_Cursor(MyLCD,1,1);
				 LCD16X2_Write_String(MyLCD,txt);


				if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
				{
				HAL_Delay(100);
				delayTimerOff_1++;
				}
				if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
				{
				HAL_Delay(100);
				delayTimerOff_1--;
				}
				//-> perfect
				if (delayTimerOff_1>240)  delayTimerOff_1=0;
				if (delayTimerOff_1<0)    delayTimerOff_1=0;

				}    //end while incremet and decremet

			}   // end while Enter
			Flash_Save();
			HAL_Delay(500);
			while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
					&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
			{

			    HAL_IWDG_Refresh(&hiwdg);
				 sprintf((char*)txt,"[10]T2 OFF %02d S  ",delayTimerOff_2);
				 LCD16X2_Set_Cursor(MyLCD,1,1);
				 LCD16X2_Write_String(MyLCD,txt);

				 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

			while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
					|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
			{

				 sprintf((char*)txt,"[10]T2 OFF %02d S  ",delayTimerOff_2);
				 LCD16X2_Set_Cursor(MyLCD,1,1);
				 LCD16X2_Write_String(MyLCD,txt);

				if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
				{
				HAL_Delay(100);
				delayTimerOff_2++;
				}
				if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
				{
				HAL_Delay(100);
				delayTimerOff_2--;
				}
				//-> perfect
				if (delayTimerOff_2>240)  delayTimerOff_2=0;
				if (delayTimerOff_2<0)    delayTimerOff_2=0;
				}    //end while incremet and decremet
			}

			//-> timer 3
			Flash_Save();
			HAL_Delay(500);
			/*
			while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
					&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
			{

			    HAL_IWDG_Refresh(&hiwdg);
				 sprintf((char*)txt,"[10]T3 OFF %02d S  ",delayTimerOff_3);
				 LCD16X2_Set_Cursor(MyLCD,1,1);
				 LCD16X2_Write_String(MyLCD,txt);

				 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

			while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
					|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
			{

				 sprintf((char*)txt,"[10]T3 OFF %02d S  ",delayTimerOff_3);
				 LCD16X2_Set_Cursor(MyLCD,1,1);
				 LCD16X2_Write_String(MyLCD,txt);

				if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
				{
				HAL_Delay(100);
				delayTimerOff_3++;
				}
				if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
				{
				HAL_Delay(100);
				delayTimerOff_3--;
				}
				//-> perfect
				if (delayTimerOff_3>240)  delayTimerOff_3=0;
				if (delayTimerOff_3<0)    delayTimerOff_3=0;
				}    //end while increment and decrement
			}
			*/
			//-> save to epprom
			Flash_Save();
			//lcd_clear();
			LCD16X2_Clear(MyLCD);



}
//--------------------------------Set Voltage Mode-------------------------------------
void SetVoltageMode()
{
		HAL_Delay(500);
	//	LCD16X2_Clear(MyLCD);
		while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
				&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
		{

		    HAL_IWDG_Refresh(&hiwdg);
			 sprintf(txt,"[11] Timer Mode");
				LCD16X2_Set_Cursor(MyLCD,1,1);
				LCD16X2_Write_String(MyLCD,txt);
			 if (RunOnBatteryVoltageMode==1)
			 {

				 LCD16X2_Set_Cursor(MyLCD,2,8);
				 LCD16X2_Write_String(MyLCD,"OFF");
			 }
			 if (RunOnBatteryVoltageMode==0)
			 {
				 LCD16X2_Set_Cursor(MyLCD,2,8);
				 LCD16X2_Write_String(MyLCD,"ON ");

			 }

			 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

		while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
				|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
		{
			if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
			{
			HAL_Delay(100);
			RunOnBatteryVoltageMode=1;
			}
			if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
			{
			HAL_Delay(100);
			RunOnBatteryVoltageMode=0;
			}
			}    //end while incremet and decremet

		}   // end while Enter
		Flash_Save();
		LCD16X2_Clear(MyLCD);
		HAL_Delay(500);
}

//--------------------------------Set UPS MOde---------------------------------------
void SetUPSMode()
{
   // lcd_clear();
 //	LCD16X2_Clear(MyLCD);
	HAL_Delay(500);
	while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
			&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
	{
	    HAL_IWDG_Refresh(&hiwdg);
		 sprintf(txt,"[12] UPS Mode");
		// lcd_puts(0,0,txt);
			LCD16X2_Set_Cursor(MyLCD,1,1);
			LCD16X2_Write_String(MyLCD,txt);
		 if (UPSMode==0)
		 {
			// lcd_puts(1,8,"OFF");
			 LCD16X2_Set_Cursor(MyLCD,2,8);
			 LCD16X2_Write_String(MyLCD,"OFF");
		 }
		 if (UPSMode==1)
		 {
			 LCD16X2_Set_Cursor(MyLCD,2,8);
			 LCD16X2_Write_String(MyLCD,"ON ");
			// lcd_puts(1,8,"ON ");
		 }

		 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

	while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
			|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
	{
		if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
		{
		HAL_Delay(100);
		UPSMode=1;
		}
		if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
		{
		HAL_Delay(100);
		UPSMode=0;
		}
		}    //end while incremet and decremet

	}   // end while Enter
	Flash_Save();
	LCD16X2_Clear(MyLCD);
	HAL_Delay(500);

}

//---------------------------------SELECT DS1307 OR INTERNAL RTC--------------------
void SelectRTC()
{
	HAL_Delay(500);
		while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
				&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
		{

		    HAL_IWDG_Refresh(&hiwdg);
			 sprintf(txt,"[13]EXTERNAL RTC");
			// lcd_puts(0,0,txt);
				LCD16X2_Set_Cursor(MyLCD,1,1);
				LCD16X2_Write_String(MyLCD,txt);
			 if (usedInsideRTC==1)    // USE INTERNAL RTC EXTERNAL DS1307 IS OFF
			 {
				// lcd_puts(1,8,"OFF");
				 LCD16X2_Set_Cursor(MyLCD,2,8);
				 LCD16X2_Write_String(MyLCD,"OFF");
			 }
			 if (usedInsideRTC==0)  //DS1307 RTC IS OFF
			 {
				 LCD16X2_Set_Cursor(MyLCD,2,8);
				 LCD16X2_Write_String(MyLCD,"ON ");
				// lcd_puts(1,8,"ON ");
			 }

			 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

		while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
				|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
		{
			if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
			{
			HAL_Delay(100);
			usedInsideRTC=1;
			}
			if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
			{
			HAL_Delay(100);
			usedInsideRTC=0;
			}
			}    //end while incremet and decremet

		}   // end while Enter
		Flash_Save();
		LCD16X2_Clear(MyLCD);
		HAL_Delay(500);



}
//--------------------------------Set RTC Time---------------------------------------
void SetRTC_Time()
{
RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef DateToUpdate = {0};
/* Get the RTC current Time */
HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
/* Get the RTC current Date */
HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

set_ds1307_hours=gTime.Hours;
set_ds1307_minutes=gTime.Minutes;
set_ds1307_day=gDate.Date;
set_ds1307_month=gDate.Month;
set_ds1307_year=gDate.Year;


	      //  lcd_clear();
         //   LCD16X2_Clear(MyLCD);

currentMillis_1=0,currentMillis_2=0;
previousMiliis_1=0,previousMiliis_2=0;
			HAL_Delay(500);


			while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
					&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
					{

			                         HAL_IWDG_Refresh(&hiwdg);

							         currentMillis_2=HAL_GetTick();
									 if(currentMillis_2-previousMiliis_2 >=1000)
									 {
									 previousMiliis_2=currentMillis_2;
									 sprintf((char*)txt,"[14] H:%02d-M:%02d ",set_ds1307_hours,set_ds1307_minutes);
									 LCD16X2_Set_Cursor(MyLCD,1,1);
									 LCD16X2_Write_String(MyLCD,txt);
									 }

									 currentMillis_1=HAL_GetTick();
									 if(currentMillis_1-previousMiliis_1 >=2000)
									 {
									 previousMiliis_1=currentMillis_1;
									 LCD16X2_Set_Cursor(MyLCD,1,6);
									 LCD16X2_Write_String(MyLCD,"    ");
									 }


						 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

					while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
							|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
					{
						 sprintf((char*)txt,"[14] H:%02d-M:%02d ",set_ds1307_hours,set_ds1307_minutes);
						 LCD16X2_Set_Cursor(MyLCD,1,1);
						 LCD16X2_Write_String(MyLCD,txt);

						if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
						{
						HAL_Delay(ButtonDelay);
						set_ds1307_hours++;
						}
						if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
						{
						HAL_Delay(ButtonDelay);
						set_ds1307_hours--;
						}
						//-> perfect
						if (set_ds1307_hours>23)  set_ds1307_hours=0;
						if (set_ds1307_hours<0) set_ds1307_hours=0;
						}    //end while incremet and decremet
					}   // end while Enter
			currentMillis_1=0,currentMillis_2=0;
			previousMiliis_1=0,previousMiliis_2=0;
			HAL_Delay(500);
			while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
					&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
					{

			             HAL_IWDG_Refresh(&hiwdg);


				         currentMillis_2=HAL_GetTick();
						 if(currentMillis_2-previousMiliis_2 >=1000)
						 {
						 previousMiliis_2=currentMillis_2;
						 sprintf((char*)txt,"[14] H:%02d-M:%02d ",set_ds1307_hours,set_ds1307_minutes);
						 LCD16X2_Set_Cursor(MyLCD,1,1);
						 LCD16X2_Write_String(MyLCD,txt);
						 }

						 currentMillis_1=HAL_GetTick();
						 if(currentMillis_1-previousMiliis_1 >=2000)
						 {
						 previousMiliis_1=currentMillis_1;
						 LCD16X2_Set_Cursor(MyLCD,1,11);
						 LCD16X2_Write_String(MyLCD,"    ");
						 }

						 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

					while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
							|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
					{

						 sprintf((char*)txt,"[14] H:%02d-M:%02d ",set_ds1307_hours,set_ds1307_minutes);
						 LCD16X2_Set_Cursor(MyLCD,1,1);
						 LCD16X2_Write_String(MyLCD,txt);

						if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
						{
						HAL_Delay(ButtonDelay);
						set_ds1307_minutes++;
						}
						if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
						{
						HAL_Delay(ButtonDelay);
						set_ds1307_minutes--;
						}
						//-> perfect
						if (set_ds1307_minutes>59)  set_ds1307_minutes=0;
						if (set_ds1307_minutes<0) set_ds1307_minutes=0;
						}    //end while incremet and decremet
					}   // end while Enter

			//-> write new Time and Date To RTC
					  sTime.Hours = set_ds1307_hours;
					  sTime.Minutes =set_ds1307_minutes;
					  sTime.Seconds = 0;

					  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
					  {
					    Error_Handler();
					  }

					  //----------------------------------------Setting Date-------------------------
					  /*

			HAL_Delay(500);
						while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
								&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
								{
									 sprintf((char*)txt,"[11] %02d/%02d/%02d ",set_ds1307_day,set_ds1307_month,set_ds1307_year);
									// lcd_puts(0,0,txt);
										LCD16X2_Set_Cursor(MyLCD,1,1);
										 LCD16X2_Write_String(MyLCD,txt);

									 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

								while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
										|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
								{
									if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
									{
									HAL_Delay(ButtonDelay);
									set_ds1307_day++;
									}
									if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
									{
									HAL_Delay(ButtonDelay);
									set_ds1307_day--;
									}
									//-> perfect
									if (set_ds1307_day>31)  set_ds1307_day=0;
									if (set_ds1307_day<0) set_ds1307_day=0;
									}    //end while incremet and decremet
								}   // end while Enter
						HAL_Delay(500);

						while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
								&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
							{
							 sprintf((char*)txt,"[11] %02d/%02d/%02d ",set_ds1307_day,set_ds1307_month,set_ds1307_year);
					  //  lcd_puts(0,0,txt);
								LCD16X2_Set_Cursor(MyLCD,1,1);
								 LCD16X2_Write_String(MyLCD,txt);

					if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

					while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
						|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
						{
						if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
						{
						HAL_Delay(ButtonDelay);
						set_ds1307_month++;
						}
						if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
						{
						HAL_Delay(ButtonDelay);
						set_ds1307_month--;
						}
						//-> perfect
						if (set_ds1307_month>12)  set_ds1307_month=0;
						if (set_ds1307_month<0) set_ds1307_month=0;
						}    //end while incremet and decremet
						}   // end while Enter

                   HAL_Delay(500);

						while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
								&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
							{
							 sprintf((char*)txt,"[11] %02d/%02d/%02d ",set_ds1307_day,set_ds1307_month,set_ds1307_year);
					 //   lcd_puts(0,0,txt);
								LCD16X2_Set_Cursor(MyLCD,1,1);
								 LCD16X2_Write_String(MyLCD,txt);

						if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

					while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
						|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
						{
						if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
						{
						HAL_Delay(ButtonDelay);
						set_ds1307_year++;
						}
						if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
						{
						HAL_Delay(ButtonDelay);
						set_ds1307_year--;
						}
						//-> perfect
						if (set_ds1307_year>99)  set_ds1307_year=0;
						if (set_ds1307_year<0) set_ds1307_year=0;
						}    //end while incremet and decremet
						}   // end while Enter





		 // DateToUpdate.WeekDay = 8;
		  DateToUpdate.Month = set_ds1307_month;
		  DateToUpdate.Date =set_ds1307_day;
		  DateToUpdate.Year = set_ds1307_year;

		  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
		  {
		    Error_Handler();
		  }
		  */
	//  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);  // backup register
}

//---------------------------------SET DS1307 TIME-----------------------------------------------
void SetDS1307()
{
	ReadDS1307_Time();
	set_ds1307_hours=time_ds1307.hour;
	set_ds1307_minutes=time_ds1307.minutes;
	set_ds1307_day=time_ds1307.dayofmonth;
	set_ds1307_month=time_ds1307.month;
	set_ds1307_year=time_ds1307.year;


	currentMillis_1=0,currentMillis_2=0;
	previousMiliis_1=0,previousMiliis_2=0;


				HAL_Delay(500);

				while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
						&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
						{

				                HAL_IWDG_Refresh(&hiwdg);

								 currentMillis_2=HAL_GetTick();
								 if(currentMillis_2-previousMiliis_2 >=1000)
								 {
								 previousMiliis_2=currentMillis_2;
								 sprintf((char*)txt,"[14] H:%02d-M:%02d ",set_ds1307_hours,set_ds1307_minutes);
								 LCD16X2_Set_Cursor(MyLCD,1,1);
								 LCD16X2_Write_String(MyLCD,txt);
								 }

								 currentMillis_1=HAL_GetTick();
								 if(currentMillis_1-previousMiliis_1 >=2000)
								 {
								 previousMiliis_1=currentMillis_1;
								 LCD16X2_Set_Cursor(MyLCD,1,6);
								 LCD16X2_Write_String(MyLCD,"    ");
								 }

							 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

						while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
								|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
						{

							 sprintf((char*)txt,"[14] H:%02d-M:%02d ",set_ds1307_hours,set_ds1307_minutes);
							 LCD16X2_Set_Cursor(MyLCD,1,1);
							 LCD16X2_Write_String(MyLCD,txt);

							if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
							{
							HAL_Delay(ButtonDelay);
							set_ds1307_hours++;
							}
							if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
							{
							HAL_Delay(ButtonDelay);
							set_ds1307_hours--;
							}
							//-> perfect
							if (set_ds1307_hours>23)  set_ds1307_hours=0;
							if (set_ds1307_hours<0) set_ds1307_hours=0;
							}    //end while incremet and decremet
						}   // end while Enter

				currentMillis_1=0,currentMillis_2=0;
				previousMiliis_1=0,previousMiliis_2=0;
				HAL_Delay(500);
				while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
						&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
						{

				                 HAL_IWDG_Refresh(&hiwdg);

								 currentMillis_2=HAL_GetTick();
								 if(currentMillis_2-previousMiliis_2 >=1000)
								 {
								 previousMiliis_2=currentMillis_2;
								 sprintf((char*)txt,"[14] H:%02d-M:%02d ",set_ds1307_hours,set_ds1307_minutes);
								 LCD16X2_Set_Cursor(MyLCD,1,1);
								 LCD16X2_Write_String(MyLCD,txt);
								 }

								 currentMillis_1=HAL_GetTick();
								 if(currentMillis_1-previousMiliis_1 >=2000)
								 {
								 previousMiliis_1=currentMillis_1;
								 LCD16X2_Set_Cursor(MyLCD,1,11);
								 LCD16X2_Write_String(MyLCD,"    ");
								 }

							 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

						while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
								|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
						{

							 sprintf((char*)txt,"[14] H:%02d-M:%02d ",set_ds1307_hours,set_ds1307_minutes);
							 LCD16X2_Set_Cursor(MyLCD,1,1);
							 LCD16X2_Write_String(MyLCD,txt);

							if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
							{
							HAL_Delay(ButtonDelay);
							set_ds1307_minutes++;
							}
							if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
							{
							HAL_Delay(ButtonDelay);
							set_ds1307_minutes--;
							}
							//-> perfect
							if (set_ds1307_minutes>59)  set_ds1307_minutes=0;
							if (set_ds1307_minutes<0) set_ds1307_minutes=0;
							}    //end while incremet and decremet
						}   // end while Enter

				//-> write new Time and Date To RTC

						  time_ds1307.hour=set_ds1307_hours;
						  time_ds1307.minutes=set_ds1307_minutes;
						  time_ds1307.seconds=0;

						  SetDS1307_Time(time_ds1307.seconds, time_ds1307.minutes, time_ds1307.hour, time_ds1307.dayofweek, 0, time_ds1307.month, time_ds1307.year);

						  //----------------------------------------Setting Date-------------------------

				HAL_Delay(500);
							while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
									&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
									{

							          HAL_IWDG_Refresh(&hiwdg);
										 sprintf((char*)txt,"[14] %02d/%02d/%02d ",set_ds1307_day,set_ds1307_month,set_ds1307_year);
										 LCD16X2_Set_Cursor(MyLCD,1,1);
										 LCD16X2_Write_String(MyLCD,txt);

										 if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

									while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
											|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
									{
										 sprintf((char*)txt,"[14] %02d/%02d/%02d ",set_ds1307_day,set_ds1307_month,set_ds1307_year);
										 LCD16X2_Set_Cursor(MyLCD,1,1);
										 LCD16X2_Write_String(MyLCD,txt);
										if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
										{
										HAL_Delay(ButtonDelay);
										set_ds1307_day++;
										}
										if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
										{
										HAL_Delay(ButtonDelay);
										set_ds1307_day--;
										}
										//-> perfect
										if (set_ds1307_day>31)  set_ds1307_day=0;
										if (set_ds1307_day<0) set_ds1307_day=0;
										}    //end while incremet and decremet
									}   // end while Enter
							HAL_Delay(500);

							while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
									&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
								{

							    HAL_IWDG_Refresh(&hiwdg);
								 sprintf((char*)txt,"[14] %02d/%02d/%02d ",set_ds1307_day,set_ds1307_month,set_ds1307_year);
								LCD16X2_Set_Cursor(MyLCD,1,1);
							    LCD16X2_Write_String(MyLCD,txt);

						if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

						while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
							|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
							{

							 sprintf((char*)txt,"[14] %02d/%02d/%02d ",set_ds1307_day,set_ds1307_month,set_ds1307_year);
							LCD16X2_Set_Cursor(MyLCD,1,1);
						    LCD16X2_Write_String(MyLCD,txt);

							if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
							{
							HAL_Delay(ButtonDelay);
							set_ds1307_month++;
							}
							if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
							{
							HAL_Delay(ButtonDelay);
							set_ds1307_month--;
							}
							//-> perfect
							if (set_ds1307_month>12)  set_ds1307_month=0;
							if (set_ds1307_month<0) set_ds1307_month=0;
							}    //end while incremet and decremet
							}   // end while Enter

	                   HAL_Delay(500);

							while (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET
									&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
								{

							    HAL_IWDG_Refresh(&hiwdg);
								 sprintf((char*)txt,"[14] %02d/%02d/%02d ",set_ds1307_day,set_ds1307_month,set_ds1307_year);

									LCD16X2_Set_Cursor(MyLCD,1,1);
									 LCD16X2_Write_String(MyLCD,txt);

							if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET) break ;

						while(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
							|| HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET )
							{

							 sprintf((char*)txt,"[14] %02d/%02d/%02d ",set_ds1307_day,set_ds1307_month,set_ds1307_year);

								LCD16X2_Set_Cursor(MyLCD,1,1);
								 LCD16X2_Write_String(MyLCD,txt);

							if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET  )
							{
							HAL_Delay(ButtonDelay);
							set_ds1307_year++;
							}
							if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin	)==GPIO_PIN_SET)
							{
							HAL_Delay(ButtonDelay);
							set_ds1307_year--;
							}
							//-> perfect
							if (set_ds1307_year>99)  set_ds1307_year=0;
							if (set_ds1307_year<0) set_ds1307_year=0;
							}    //end while incremet and decremet
							}   // end while Enter

			  time_ds1307.month=set_ds1307_month;
			  time_ds1307.dayofmonth=set_ds1307_day;
			  time_ds1307.year=set_ds1307_year;

			  SetDS1307_Time(time_ds1307.seconds, time_ds1307.minutes, time_ds1307.hour, 0, time_ds1307.dayofmonth, time_ds1307.month, time_ds1307.year);

	}
//---------------------------------Check Timers------------------------------------------------------
void Check_Timers()
{
if (RunOnBatteryVoltageMode==0)
{
	matched_timer_1_start=CheckTimeOccuredOn(seconds_lcd_1,minutes_lcd_1,hours_lcd_1);
	matched_timer_1_stop=CheckTimeOccuredOff(seconds_lcd_2,minutes_lcd_2,hours_lcd_2);
	matched_timer_2_start=CheckTimeOccuredOn(seconds_lcd_timer2_start,minutes_lcd_timer2_start,hours_lcd_timer2_start);
	matched_timer_2_stop=CheckTimeOccuredOff(seconds_lcd_timer2_stop,minutes_lcd_timer2_stop,hours_lcd_timer2_stop);
	matched_timer_3_start=CheckTimeOccuredOff(0,minutes_lcd_timer3_start,hours_lcd_timer3_start);
	matched_timer_3_stop=CheckTimeOccuredOff(0,minutes_lcd_timer3_stop,hours_lcd_timer3_stop);
	//---------------------------- Timer 1 -----------------------------------------
	//-> turn Load On
	if (matched_timer_1_start==1)
	{
	Timer_isOn=1;
	TurnOffLoadsByPass=0;
	RunLoadsByBass=0;
	//-> when grid is available and timer is on after grid so access the condition to active timer after grid is off
	if (Ac_Available==GPIO_PIN_SET && Timer_Enable==1  && Vin_Battery >= StartLoadsVoltage && RunWithOutBattery==false )
	{
	HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_SET);


	}
	//-> if run with out battery is selected
	if (Ac_Available==GPIO_PIN_SET && Timer_Enable==1  && RunWithOutBattery==true )
	{
		HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_SET);
	}
	} // end if Ac_Available
	//-> Turn Load off
	//-----------------------------------------------------------------------------------------------
	if (matched_timer_1_stop==1)
	{
	Timer_isOn=0;        // to continue the timer after breakout the timer when grid is available
	///EEPROM_write(0x49,0);        //- save it to eeprom if power is cut
	//-> when grid is available and timer is on after grid so access the condition to active timer after grid is off
	if (Ac_Available==GPIO_PIN_SET && Timer_Enable==1  &&  RunWithOutBattery==false  )
	{
	//for the turn off there is no need for delay
	SecondsRealTimePv_ReConnect_T1=0;
	CountSecondsRealTimePv_ReConnect_T1=0;
	HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_RESET);

	}
	if (Ac_Available==GPIO_PIN_SET && Timer_Enable==1  && RunWithOutBattery==true  )
	{
	//for the turn off there is no need for delay
	SecondsRealTimePv_ReConnect_T1=0;
	CountSecondsRealTimePv_ReConnect_T1=0;
	HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_RESET);
	}
	}
	//----------------------------------- Timer 1 End---------------------------------------------------------
	//----------------------------------- Timer 2 Start-------------------------------------------------------
	if (matched_timer_2_start==1)
	{
	Timer_2_isOn=1;
	TurnOffLoadsByPass=0;     // this variable just for if user shutdown loads and don't want to reactivated so it will be zeroed until next timer
	RunLoadsByBass=0;
		//-> when grid is available and timer is on after grid so access the condition to active timer after grid is off
	if (Ac_Available==GPIO_PIN_SET && Timer_Enable==1  && Vin_Battery >= StartLoadsVoltage_T2 && RunWithOutBattery==false)
	{
	HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_SET);
	}

	if (Ac_Available==GPIO_PIN_SET && Timer_Enable==1  && RunWithOutBattery==true)
	{
	HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_SET);
	}
	} // end if Ac_Available

	if (matched_timer_2_stop==1)
	{
	Timer_2_isOn=0;        // to continue the timer after breakout the timer when grid is available
	//-> when grid is available and timer is on after grid so access the condition to active timer after grid is off
	if (Ac_Available==GPIO_PIN_SET && Timer_Enable==1 && RunWithOutBattery==false )
	{
	HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_RESET);
	SecondsRealTimePv_ReConnect_T2=0;
	CountSecondsRealTimePv_ReConnect_T2=0;
	}

	if (Ac_Available==GPIO_PIN_SET && Timer_Enable==1  && RunWithOutBattery==true )
	{
	HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_RESET);
	SecondsRealTimePv_ReConnect_T2=0;
	CountSecondsRealTimePv_ReConnect_T2=0;
	}
	} // end match timer stop
	//-------------------------------------Timer 3-----------------------------------------------
	if (matched_timer_3_start==1)
		{
		Timer_3_isOn=1;
		TurnOffLoadsByPass=0;     // this variable just for if user shutdown loads and don't want to reactivated so it will be zeroed until next timer
		RunLoadsByBass=0;
			//-> when grid is available and timer is on after grid so access the condition to active timer after grid is off
		if (Ac_Available==GPIO_PIN_SET && Timer_Enable==1  && Vin_Battery >= StartLoadsVoltage_T3 && RunWithOutBattery==false)
		{
		HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_SET);
		}

		if (Ac_Available==GPIO_PIN_SET && Timer_Enable==1  && RunWithOutBattery==true)
		{
		HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_SET);
		}
		} // end if Ac_Available

		if (matched_timer_3_stop==1)
		{
		Timer_3_isOn=0;        // to continue the timer after breakout the timer when grid is available
		//-> when grid is available and timer is on after grid so access the condition to active timer after grid is off
		if (Ac_Available==GPIO_PIN_SET && Timer_Enable==1 && RunWithOutBattery==false )
		{
		HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_RESET);
		SecondsRealTimePv_ReConnect_T3=0;
		CountSecondsRealTimePv_ReConnect_T3=0;

		}

		if (Ac_Available==GPIO_PIN_SET && Timer_Enable==1  && RunWithOutBattery==true )
		{
		HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_RESET);
		SecondsRealTimePv_ReConnect_T3=0;
		CountSecondsRealTimePv_ReConnect_T3=0;
		}
		} // end match timer stop
	    } // end of run on battery voltage
    //------------------------Functions for reactiving timers------------------------
/*
 these function is used for reactiving timers when grid available in the same timer is on or off
*/
//-> if the  ac is shutdown and timer is steel in the range of being on  so reactive timer 1
if (Ac_Available==GPIO_PIN_SET && Timer_isOn==1 && Vin_Battery >= StartLoadsVoltage && RunWithOutBattery==false && TurnOffLoadsByPass==0
		&& RunOnBatteryVoltageMode ==0 )
{
CountSecondsRealTimePv_ReConnect_T1=1;
if (  SecondsRealTimePv_ReConnect_T1 > startupTIme_1)     HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_SET);

}
if (Ac_Available==GPIO_PIN_SET && Timer_isOn==1  && RunWithOutBattery==true && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==0 )
{

	CountSecondsRealTimePv_ReConnect_T1=1;
if (  SecondsRealTimePv_ReConnect_T1 > startupTIme_1)	HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_SET);

}

if (Ac_Available==GPIO_PIN_SET && Timer_2_isOn==1 && Vin_Battery >= StartLoadsVoltage_T2 && RunWithOutBattery==false && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==0)     //run with battery
{

	CountSecondsRealTimePv_ReConnect_T2=1;
if (  SecondsRealTimePv_ReConnect_T2 > startupTIme_2)
	HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_SET);
}

if (Ac_Available==GPIO_PIN_SET && Timer_2_isOn==1 &&  RunWithOutBattery==true && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==0)            //run without battery
{
	CountSecondsRealTimePv_ReConnect_T2=1;
if (  SecondsRealTimePv_ReConnect_T2 > startupTIme_2)
	HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_SET);

}

//-> timer 3
if (Ac_Available==GPIO_PIN_SET && Timer_3_isOn==1 && Vin_Battery >= StartLoadsVoltage_T3 && RunWithOutBattery==false && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==0)     //run with battery
{

	CountSecondsRealTimePv_ReConnect_T3=1;
if (  SecondsRealTimePv_ReConnect_T3 > startupTIme_3)
	HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_SET);
}

if (Ac_Available==GPIO_PIN_SET && Timer_3_isOn==1 &&  RunWithOutBattery==true && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==0)            //run without battery
{
	CountSecondsRealTimePv_ReConnect_T3=1;
if (  SecondsRealTimePv_ReConnect_T3 > startupTIme_3)
	HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_SET);

}
//*************************************BYPASS SYSTEM***********************************************
if(Ac_Available==GPIO_PIN_RESET  && UPSMode==0 )   // voltage protector is not enabled
{
TurnOffLoadsByPass=0;
RunLoadsByBass=0;
CountSecondsRealTime=1;
if(SecondsRealTime >= startupTIme_1 && Ac_Available==0)
{

HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_SET);
}
if(SecondsRealTime >= startupTIme_2 && Ac_Available==0)
{

HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_SET);
}
if(SecondsRealTime >= startupTIme_3 && Ac_Available==0)
{

HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_SET);
}
} // end
//-----------------------------------UPO MODE------------------------------------------------------
if(Ac_Available==GPIO_PIN_RESET && UPSMode==1 )
{
TurnOffLoadsByPass=0;
RunLoadsByBass=0;
CountSecondsRealTime=1;
if( Ac_Available==0 && LoadsAlreadySwitchedOFF==0)
{
LoadsAlreadySwitchedOFF=1;
HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_RESET);
}
if(SecondsRealTime >= startupTIme_1 && Ac_Available==0 && LoadsAlreadySwitchedOFF==1 )
{
	HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_SET);
}
if(SecondsRealTime >= startupTIme_2 && Ac_Available==0 && LoadsAlreadySwitchedOFF==1 )
{
HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_SET);
}
if(SecondsRealTime >= startupTIme_3 && Ac_Available==0 && LoadsAlreadySwitchedOFF==1 )
{
HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_SET);
}
}
//------------------------------------END BYPASS SYSTEM---------------------------------------------
//**************************************************************************************************
//************************************RUN ON BATTERY VOLTAGE MODE***********************************
if (Ac_Available==GPIO_PIN_SET  && Vin_Battery >= StartLoadsVoltage && RunWithOutBattery==false && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==1 )
{
CountSecondsRealTimePv_ReConnect_T1=1;
if (  SecondsRealTimePv_ReConnect_T1 > startupTIme_1)
HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_SET);
}

if (Ac_Available==GPIO_PIN_SET && Vin_Battery >= StartLoadsVoltage_T2 && RunWithOutBattery==false && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==1)     //run with battery
{
	CountSecondsRealTimePv_ReConnect_T2=1;
if (  SecondsRealTimePv_ReConnect_T2 > startupTIme_2)
HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_SET);
}
//-> timer 3
if (Ac_Available==GPIO_PIN_SET && Vin_Battery >= StartLoadsVoltage_T3 && RunWithOutBattery==false && TurnOffLoadsByPass==0 && RunOnBatteryVoltageMode ==1)     //run with battery
{
	CountSecondsRealTimePv_ReConnect_T3=1;
if (  SecondsRealTimePv_ReConnect_T3 > startupTIme_3)
HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_SET);
}
//----------------------------------END RUN ON BATTERY VOLTAGE---------------------------------------
//**************************************TURN OFF LOADS***********************************************
//--Turn Load off when battery Voltage  is Low and AC Not available and Bypass is enabled
if (Vin_Battery<=Mini_Battery_Voltage && Ac_Available==GPIO_PIN_SET  && RunWithOutBattery==false )
{
HAL_TIM_Base_Start_IT(&htim3); // give some time for battery voltage
}

//--Turn Load off when battery Voltage  is Low and AC Not available and Bypass is enabled
if (Vin_Battery<=Mini_Battery_Voltage_T2 && Ac_Available==GPIO_PIN_SET  && RunWithOutBattery==false )
{

HAL_TIM_Base_Start_IT(&htim3); // give some time for battery voltage
}

if (Vin_Battery<=Mini_Battery_Voltage_T3 && Ac_Available==GPIO_PIN_SET  && RunWithOutBattery==false )
{
HAL_TIM_Base_Start_IT(&htim3); // give some time for battery voltage
}
//-------------------------------------END OF TURN OFF LOADS----------------------------------
} // end check timers function function
//*************************************TurnLoadsOffWhenGridOff**************************************
void TurnLoadsOffWhenGridOff()
{
	//-> timer 1
	if(Ac_Available==GPIO_PIN_SET && Timer_isOn==0 && RunLoadsByBass==0  && RunOnBatteryVoltageMode==0)
	{
		SecondsRealTime=0;
		CountSecondsRealTime=0;
		SecondsRealTimePv_ReConnect_T1=0;
		CountSecondsRealTimePv_ReConnect_T1=0;

	HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_RESET);
	}
    //-> timer 2
	if (Ac_Available==GPIO_PIN_SET && Timer_2_isOn==0 && RunLoadsByBass==0 && RunOnBatteryVoltageMode==0)  // it must be   Timer_2_isOn==0    but because of error in loading eeprom value
	{
		SecondsRealTime=0;
		CountSecondsRealTime=0;
		SecondsRealTimePv_ReConnect_T2=0;
		CountSecondsRealTimePv_ReConnect_T2=0;
	HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_RESET);

	}

	//-> timer 3
	if (Ac_Available==GPIO_PIN_SET && Timer_3_isOn==0 && RunLoadsByBass==0 && RunOnBatteryVoltageMode==0)  // it must be   Timer_2_isOn==0    but because of error in loading eeprom value
		{
			SecondsRealTime=0;
			CountSecondsRealTime=0;
			SecondsRealTimePv_ReConnect_T3=0;
			CountSecondsRealTimePv_ReConnect_T3=0;
		HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_RESET);

		}

	//-> upo mode
	if (Ac_Available==GPIO_PIN_SET &&  RunLoadsByBass==0 && UPSMode==1 && LoadsAlreadySwitchedOFF==1)
	{
		LoadsAlreadySwitchedOFF=0;
		SecondsRealTime=0;
		SecondsRealTimePv_ReConnect_T1=0;
		SecondsRealTimePv_ReConnect_T2=0;
		SecondsRealTimePv_ReConnect_T3=0;
		CountSecondsRealTime=0;
		CountSecondsRealTimePv_ReConnect_T1=0;
		CountSecondsRealTimePv_ReConnect_T2=0;
		CountSecondsRealTimePv_ReConnect_T3=0;
	    HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_RESET);
      	HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_RESET);
	}
}
//***************************************CheckBatterySystem******************************************
void CheckSystemBatteryMode()
{
if (Vin_Battery>= 35 && Vin_Battery <= 60) SystemBatteryMode=48;
else if (Vin_Battery>=18 && Vin_Battery <=32) SystemBatteryMode=24;
else if (Vin_Battery >=1 && Vin_Battery<= 16 ) SystemBatteryMode=12;
else if(Vin_Battery==0) SystemBatteryMode=24;
else SystemBatteryMode=24; // take it as default
}
//*****************************************SCREEN****************************************************
void Screen_1()
{
	Read_Battery();
	if (RunLoadsByBass==0)   LCD_Clear(1,13,16);  else {


		LCD16X2_Set_Cursor(MyLCD,1,15);
		LCD16X2_Write_String(MyLCD,"B");
	}
	if (RunOnBatteryVoltageMode==0)
	{
		if (usedInsideRTC==1) display_time();        //reading from internal rtc
	 	if (usedInsideRTC==0) DisplayTimeDS1307();   //reading from ds1307
	}
	else
	{

		LCD16X2_Set_Cursor(MyLCD,1,1);
		LCD16X2_Write_String(MyLCD,"Voltage Mode  ");
	}



	if (Ac_Available== GPIO_PIN_RESET ) // in this if voltage protector is turned of no need for voltage read
	{

		LCD16X2_Set_Cursor(MyLCD,2,7);
		LCD16X2_Write_String(MyLCD," Grid ON   ");

	}

	if (Ac_Available== GPIO_PIN_SET ) // in this if voltage protector is turned of no need for voltage read
	{

		LCD16X2_Set_Cursor(MyLCD,2,7);
		LCD16X2_Write_String(MyLCD," Grid OFF  ");

	}




}

//---------------------------------Check For Set-----------------------------------------------------
void CheckForSet()
{
if (HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_RESET)
 {
	HAL_Delay(1500);
    SetupProgram();
 }

} //**********************************************************************************************
void CheckForParams()
{
//----------------Timer 1 ----------------
if (hours_lcd_1< 0  || hours_lcd_1 > 23)
{
hours_lcd_1=8;
Flash_Save();
}
if (minutes_lcd_1< 0  || minutes_lcd_1 > 59)
{
minutes_lcd_1=0;
Flash_Save();

}
if (hours_lcd_2< 0  || hours_lcd_2 > 23)
{
hours_lcd_2=18;
Flash_Save();
}
if (minutes_lcd_2< 0  || minutes_lcd_2 > 59)
{
minutes_lcd_2=0;
Flash_Save();
}
//----------------Timer 2 ------------------------
if (hours_lcd_timer2_start< 0  || hours_lcd_timer2_start > 23)
{
hours_lcd_timer2_start=9;
Flash_Save();
}
if (minutes_lcd_timer2_start< 0  || minutes_lcd_timer2_start > 59)
{
minutes_lcd_timer2_start=0;
Flash_Save();
}
if (hours_lcd_timer2_stop< 0  || hours_lcd_timer2_stop > 23)
{
hours_lcd_timer2_stop=17;
Flash_Save();
}
if (minutes_lcd_timer2_stop< 0  || minutes_lcd_timer2_stop > 59)
{
minutes_lcd_timer2_stop=0;
Flash_Save();
}
//---------------------------Timer 3 --------------------------------
if (hours_lcd_timer3_start< 0  || hours_lcd_timer3_start > 23)
{
hours_lcd_timer3_start=9;
Flash_Save();
}
if (minutes_lcd_timer3_start< 0  || minutes_lcd_timer3_start > 59)
{
minutes_lcd_timer3_start=0;
Flash_Save();
}
if (hours_lcd_timer3_stop< 0  || hours_lcd_timer3_stop > 23)
{
hours_lcd_timer3_stop=17;
Flash_Save();
}
if (minutes_lcd_timer3_stop< 0  || minutes_lcd_timer3_stop > 59)
{
minutes_lcd_timer3_stop=0;
Flash_Save();
}
//---------------------------LOW Voltage------------------------------
if (Mini_Battery_Voltage<= 0  || Mini_Battery_Voltage > 65.0 || isnan(Mini_Battery_Voltage))
{
if (SystemBatteryMode==12) Mini_Battery_Voltage=12.0;
if (SystemBatteryMode==24) Mini_Battery_Voltage=24.5;
if (SystemBatteryMode==48) Mini_Battery_Voltage=48.5;
Flash_Save();
}
if (Mini_Battery_Voltage_T2<= 0  || Mini_Battery_Voltage_T2 > 65.0 || isnan(Mini_Battery_Voltage_T2))
{
if (SystemBatteryMode==12) Mini_Battery_Voltage_T2=12.3;
if (SystemBatteryMode==24) Mini_Battery_Voltage_T2=25.0;
if (SystemBatteryMode==48) Mini_Battery_Voltage_T2=49.5;
Flash_Save();
}

if (Mini_Battery_Voltage_T3<= 0  || Mini_Battery_Voltage_T3 > 65.0 || isnan(Mini_Battery_Voltage_T3))
{
if (SystemBatteryMode==12) Mini_Battery_Voltage_T3=12.5;
if (SystemBatteryMode==24) Mini_Battery_Voltage_T3=25.5;
if (SystemBatteryMode==48) Mini_Battery_Voltage_T3=50.0;
Flash_Save();
}
//--------------------------Start Loads Voltage------------------------
if (StartLoadsVoltage<= 0  || StartLoadsVoltage > 65.0 || isnan(StartLoadsVoltage) )
{
if (SystemBatteryMode==12) StartLoadsVoltage=13.0;
if (SystemBatteryMode==24) StartLoadsVoltage=25.5;
if (SystemBatteryMode==48) StartLoadsVoltage=51.0;
Flash_Save();
}
if (StartLoadsVoltage_T2<= 0  || StartLoadsVoltage_T2 > 65.0 || isnan(StartLoadsVoltage_T2))
{
if (SystemBatteryMode==12) StartLoadsVoltage_T2=13.2;
if (SystemBatteryMode==24) StartLoadsVoltage_T2=26.0;
if (SystemBatteryMode==48) StartLoadsVoltage_T2=52.0;
Flash_Save();
}

if (StartLoadsVoltage_T3<= 0  || StartLoadsVoltage_T3 > 65.0 || isnan(StartLoadsVoltage_T3))
{
if (SystemBatteryMode==12) StartLoadsVoltage_T3=13.5;
if (SystemBatteryMode==24) StartLoadsVoltage_T3=26.5;
if (SystemBatteryMode==48) StartLoadsVoltage_T3=54.0;
Flash_Save();
}


if (startupTIme_1<0 || startupTIme_1> 900)
{
	startupTIme_1=90 ;
	Flash_Save();
}

if (startupTIme_2<0 || startupTIme_3> 900)
{
	startupTIme_2=120 ;
	Flash_Save();
}

if (startupTIme_3<0 || startupTIme_3> 900)
{
	startupTIme_3=180 ;
	Flash_Save();
}

if (delayTimerOff_1<0 || delayTimerOff_1> 240)
{
	delayTimerOff_1=25 ;
	Flash_Save();
}

if (delayTimerOff_2<0 || delayTimerOff_2> 240)
{
	delayTimerOff_2=20 ;
	Flash_Save();
}

if (delayTimerOff_3<0 || delayTimerOff_3> 240)
{
	delayTimerOff_3=15;
	Flash_Save();
}


if (RunOnBatteryVoltageMode<0 || RunOnBatteryVoltageMode > 1)
{
	RunOnBatteryVoltageMode=0;
	Flash_Save();
}

if (UPSMode<0 || UPSMode > 1)
{
	UPSMode=0;
	Flash_Save();
}

} // end function checkforparams
//--------------------------------WORKINING MODE------------------------------------------
//--------------------------Working Mode Flashing Led---------------------------
void WorkingMode()
{

	if (Ac_Available==GPIO_PIN_RESET)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(500);
	}

}
//-------------------------------------RUN TIMERS CHECK NOW ----------------------------------
//-> KEYS PRESSED
void RunTimersNowCheck()
{
	//-> EXIT IS PRESSED
if (HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET
		&& HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_RESET
		&& HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_RESET
		&& HAL_GPIO_ReadPin(Enter_GPIO_Port, Enter_Pin)==GPIO_PIN_SET)
		{
	  UpdateScreenTime=0;
      HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, GPIO_PIN_SET);
		}
   //-> BYPASS MODE UP IS PRESSED
if ( HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
		&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
		{
	        UpdateScreenTime=0;
		    HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, GPIO_PIN_SET);
	    	HAL_Delay(1500);
		if (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET && HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
		{
			RunLoadsByBass++;
			if (  RunLoadsByBass==1 ) HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_SET);
			if (RunLoadsByBass>=2 )
			{
				HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_SET);
			}
			if (RunLoadsByBass>=3 )
						{
							HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_SET);
						}
		} // end if
      } // end main if
  //-> BYPASS MODE DECREMENT IS PRESSED
if ( HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET
		&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
		{
	    UpdateScreenTime=0;
	    HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, GPIO_PIN_SET);
		HAL_Delay(1500);
		if (HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_SET && HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_RESET)
		{
			TurnOffLoadsByPass=1;
			RunLoadsByBass=0;
			HAL_GPIO_WritePin(RELAY_L_1_GPIO_Port, RELAY_L_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RELAY_L_2_GPIO_Port, RELAY_L_2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RELAY_L_3_GPIO_Port, RELAY_L_3_Pin, GPIO_PIN_RESET);
		} // end if
      } // end main if
    //-> RESET TO FACTORY SETTINGS
if      (HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
		&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET
		&& HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_RESET)
{
    UpdateScreenTime=0;
    HAL_GPIO_WritePin(BACKLIGHT_GPIO_Port, BACKLIGHT_Pin, GPIO_PIN_SET);
    if(HAL_GPIO_ReadPin(INCREMENT_GPIO_Port, INCREMENT_Pin)==GPIO_PIN_SET
    		&& HAL_GPIO_ReadPin(EXIT_GPIO_Port, EXIT_Pin)==GPIO_PIN_SET
    		&& HAL_GPIO_ReadPin(DECREMENT_GPIO_Port, DECREMENT_Pin)==GPIO_PIN_RESET)
    {
    	HAL_Delay(500);
    	Factory_Settings();
        LCD16X2_Set_Cursor(MyLCD,2,1);
        LCD16X2_Write_String(MyLCD,"Factory Settings");
        HAL_Delay(1500);
        LCD16X2_Clear(MyLCD);
       // lcd_clear();
    } // end if
} // end main if

} // END FUNCTION
//-------------------------------------FACTORY SETTINGS----------------------------------
void Factory_Settings()
{
	if(SystemBatteryMode==12)
	{
	Mini_Battery_Voltage=12.0;
	StartLoadsVoltage=13.0;
	Mini_Battery_Voltage_T2=12.3,
	StartLoadsVoltage_T2=13.2;
	Mini_Battery_Voltage_T3=12.5;
	StartLoadsVoltage_T3=13.5;
	}
	if(SystemBatteryMode==24)
	{
	Mini_Battery_Voltage=24.5;
	StartLoadsVoltage=25.5;
	Mini_Battery_Voltage_T2=25.0,
	StartLoadsVoltage_T2=26.0;
	Mini_Battery_Voltage_T3=25.5;
	StartLoadsVoltage_T3=26.5;
	}
	if(SystemBatteryMode==48)
	{
	Mini_Battery_Voltage=49.0;
	StartLoadsVoltage=52.0;
	Mini_Battery_Voltage_T2=50.0,
	StartLoadsVoltage_T2=53.0;
	Mini_Battery_Voltage_T3=51.0;
	StartLoadsVoltage_T3=54.0;
	}
	//-> global variables
	startupTIme_1 =90;
	startupTIme_2=120;
	startupTIme_3=180;
	hours_lcd_1=8;
	minutes_lcd_1=0;
	hours_lcd_2=17;
	minutes_lcd_2=0;
	hours_lcd_timer2_start=8;
	minutes_lcd_timer2_start=30;
	hours_lcd_timer2_stop=17;
	minutes_lcd_timer2_stop=0;
	hours_lcd_timer3_start=9;
	minutes_lcd_timer3_start=0;
	hours_lcd_timer3_stop=17;
	minutes_lcd_timer3_stop=0;
	RunOnBatteryVoltageMode=0; // default
	UPSMode=0;
	delayTimerOff_1=25;
	delayTimerOff_2=20;
	delayTimerOff_3=15;
	usedInsideRTC=1; // use ds1307 as defaut
	//-> write data to array
	flash_data[0]=hours_lcd_1;
	flash_data[1]=minutes_lcd_1;
	flash_data[2]=hours_lcd_2;
	flash_data[3]=minutes_lcd_2;
	flash_data[4]=hours_lcd_timer2_start;
	flash_data[5]=minutes_lcd_timer2_start;
	flash_data[6]=hours_lcd_timer2_stop;
	flash_data[7]=minutes_lcd_timer2_stop;
	flash_data[8]=hours_lcd_timer3_start;
	flash_data[9]=minutes_lcd_timer3_start;
	flash_data[10]=hours_lcd_timer3_stop;
	flash_data[11]=minutes_lcd_timer3_stop;
	flash_data[12]=Mini_Battery_Voltage;
	flash_data[13]=Mini_Battery_Voltage_T2;
	flash_data[14]=Mini_Battery_Voltage_T3;
	flash_data[15]=StartLoadsVoltage;
	flash_data[16]=StartLoadsVoltage_T2;
	flash_data[17]=StartLoadsVoltage_T3;
	flash_data[18]=startupTIme_1;
	flash_data[19]=startupTIme_2;
	flash_data[20]=startupTIme_3;
	flash_data[21]=RunOnBatteryVoltageMode;
	flash_data[22]=UPSMode;
	flash_data[23]=delayTimerOff_1;
	flash_data[24]=delayTimerOff_2;
	flash_data[25]=delayTimerOff_3;
	flash_data[26]=usedInsideRTC;
    //-> save data to flash
	Flash_Write_Data(Flash_Address, flash_data, 27);
}



//--------------------------------Check DS1307 is counting-------------------------------
void CheckDS1307WorkingState()
{
if (RunOnBatteryVoltageMode==0) // if timer is on as mode
{
ReadDS1307_Time();
if (time_ds1307.seconds>60) RunOnBatteryVoltageMode=1;
} // end if RunOnBatteryVoltageMode
}
//-------------------------------------MAIN LOOP-----------------------------------------

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_IWDG_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  Config();
  Flash_Load();
  HAL_TIM_Base_Start_IT(&htim15); // for counting real seconds and make lcd off
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  CheckForParams();  // done for timer 3
      CheckForSet();  // done for timer 3
	  RunTimersNowCheck(); // done for timer 3
	  WorkingMode();
	  CheckForTimerActivationInRange(); // done for timer 3
	  CheckForTimerActivationOutRange();  //done for timer 3
	  Screen_1();
	  Check_Timers();    // done for timer 3
	  TurnLoadsOffWhenGridOff();   // done for timer 3
	  CheckSystemBatteryMode();          // done for timer 3
	  HAL_Delay(200);
	  HAL_IWDG_Refresh(&hiwdg);
  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00901D23;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 3999;
  hiwdg.Init.Reload = 3999;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim3.Init.Prescaler = 16000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
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
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 16000;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1000;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|RELAY_L_3_Pin|RELAY_L_2_Pin|RELAY_L_1_Pin
                          |BACKLIGHT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : EXIT_Pin DECREMENT_Pin INCREMENT_Pin */
  GPIO_InitStruct.Pin = EXIT_Pin|DECREMENT_Pin|INCREMENT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin RELAY_L_3_Pin RELAY_L_2_Pin RELAY_L_1_Pin
                           BACKLIGHT_Pin */
  GPIO_InitStruct.Pin = LED_Pin|RELAY_L_3_Pin|RELAY_L_2_Pin|RELAY_L_1_Pin
                          |BACKLIGHT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Enter_Pin Ac_Available_Pin */
  GPIO_InitStruct.Pin = Enter_Pin|Ac_Available_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
