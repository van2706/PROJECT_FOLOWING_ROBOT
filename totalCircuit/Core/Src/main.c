/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//bien lay mau dong co
float n = 0.0;
int encoder_cnt_1 = 0, encoder_cnt_pre_1 = 0, encoder_cnt_2 = 0, encoder_cnt_pre_2 = 0;
float rate_1 = 0, rate_2 = 0;
int k = 0, kkkk = 0;
float setpoint = 100;
float rpm_tb1 = 0, rpm_tb2 = 0;
//biến sensorline
uint16_t AdcValues[5];
uint16_t ADCMinValue[5] = {247, 237, 192, 192, 204};
uint16_t ADCMaxValue[5] = {3203, 3118, 2849, 3005, 2974};
uint16_t sumADCmax = 0, sumADCmin = 0;
uint16_t Ymax = 3030, Ymin = 210;
uint16_t AdcValuesAfterCalib[5];
float A[5], e2 = 0, pre_e2 = 0, e3 = 0, pre_e3 = 0;
float sum_Adcvalues = 0;
float pwm = 0.0;
//biến colorsensor
uint32_t red_value = 0, blue_value = 0, green_value = 0;
int green = 0, red = 0, blue = 0, hangXanh = 0, hangDo = 0, empty = 1, outline = 0;

//biến động cơ
uint32_t current_time = 0, time_A = 0, time_B = 0;
int encoder_cnt_A = 0, encoder_cnt_pre_A = 0, encoder_cnt_B = 0, encoder_cnt_pre_B = 0;
float rpm_A = 0, rpm_B = 0, v_A = 0, v_B = 0, r = 0.0325, b = 0.26;
int stop = 0;
float duty_cycle_1 = 0, duty_cycle_pre_1 = 0, duty_cycle_2 = 0, duty_cycle_pre_2 = 0, duty_cycle_A = 0, duty_cycle_B = 0;

float P_part_A = 0, I_part_A = 0, D_part_A = 0;
float P_part_B = 0, I_part_B = 0, D_part_B = 0;

float er_A = 0, er_A_pre = 0, er_A_pre_pre = 0;
float er_B = 0, er_B_pre = 0, er_B_pre_pre = 0;
float error_rpm_A = 0, error_prev_A = 0, integral_error_A = 0, derivative_error_A = 0, error_rpm_B = 0, error_prev_B = 0, integral_error_B = 0, derivative_error_B = 0;
//he thong
float Ts = 0.01;
float vr = 0.7, vtb = 0, wr = 0, s = 0;
float v, w;
float k1 = 0, k2 = 280, k3 = 20;//180 16
float kp_A = 0.5, ki_A = 9.5, kd_A = 0.001; //kp_A = 0.56, ki_A = 11.2, kd_A = 0.0025
float kp_B = 0.3, ki_B = 10, kd_B = 0.001; //kp_B = 0.388, ki_B = 11.5, kd_B = 0.0016;
float e1_line = 0, e2_line = 0, e3_line = 0;
float setpoint_r = 0, setpoint_l = 0;
int i = 0, count = 0;
#define PI 3.1416
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
//ham tri tuyet doi
float abs_(float a) {
	if(a < 0) return -a;
	else return a;
}

//xe
void Follow_Tracking(void)
{
	wr = -e3;
	v = vr * cos(e3) + k1 * e1_line; //m/s
	w = wr + vr * k2 * e2/1000  + k3 * vr * sin(e3); //rad/s
	setpoint_r = (1 / r) * (v - b * w/2) * (60 / (2 * PI)); //rpm
	setpoint_l = (1 / r) * (v + b * w/2) * (60 / (2 * PI)); //rpm
	if(setpoint_r < 0){
		setpoint_r = 0;
	}
	if(setpoint_r >= 250){
		setpoint_r = 250;
	}
	if(setpoint_l < 0){
		setpoint_l = 0;
	}
	if(setpoint_l >= 250){
		setpoint_l = 250;
	}
}
//stop to get goods and no line no run
void stop_or_run(){
	if((empty == 1 && AdcValues[0] >= 1800 && AdcValues[1] >= 1800 && AdcValues[2] >= 1800 && AdcValues[3] >= 1800 && AdcValues[4] >= 1800)) {
		stop = 1;
	}
}
void no_line_no_run() {
	if((AdcValues[0] <= 700 && AdcValues[1] <= 700 && AdcValues[2] <= 700 && AdcValues[3] <= 700 && AdcValues[4] <= 700)){
		stop = 1;
		outline = 1;
	}
}
//check good?
void yes_or_no_good(){
	if(green == 1 && red == 0) {
		  hangXanh = 1;
		  hangDo = 0;
		  empty = 0;
	}
	if(green == 0 && red == 1){

		  hangXanh = 0;
		  hangDo = 1;
		  empty = 0;
	}
}
//sensorline
void CaculatorFuntionSensor(void)
   {
            A[0] = (float) ( Ymax - Ymin ) / ( ADCMaxValue[0] - ADCMinValue[0] );
            A[1] = (float) ( Ymax - Ymin ) / ( ADCMaxValue[1] - ADCMinValue[1] );
            A[2] = (float) ( Ymax - Ymin ) / ( ADCMaxValue[2] - ADCMinValue[2] );
            A[3] = (float) ( Ymax - Ymin ) / ( ADCMaxValue[3] - ADCMinValue[3] );
            A[4] = (float) ( Ymax - Ymin ) / ( ADCMaxValue[4] - ADCMinValue[4] );
      }
void CalibrationSenrsorValue(uint16_t *ADCValue)
{
		AdcValuesAfterCalib[0] = (uint16_t) (Ymin + A[0]*( ADCValue[0] - ADCMinValue[0]));
		AdcValuesAfterCalib[1] = (uint16_t) (Ymin + A[1]*( ADCValue[1] - ADCMinValue[1]));
		AdcValuesAfterCalib[2] = (uint16_t) (Ymin + A[2]*( ADCValue[2] - ADCMinValue[2]));
		AdcValuesAfterCalib[3] = (uint16_t) (Ymin + A[3]*( ADCValue[3] - ADCMinValue[3]));
		AdcValuesAfterCalib[4] = (uint16_t) (Ymin + A[4]*( ADCValue[4] - ADCMinValue[4]));
}

void CalibSensorDependOnEnvironment(void)
{
	for(uint8_t i = 0; i < 5; i++)
	{
		if(AdcValues[i] != 0)
		{
			if(AdcValues[i] > ADCMaxValue[i])
				ADCMaxValue[i] = AdcValues[i];
			if(AdcValues[i] < ADCMinValue[i])
				ADCMinValue[i] = AdcValues[i];
		}
		sumADCmax += ADCMaxValue[i];
		sumADCmin += ADCMinValue[i];
	}
	Ymax = sumADCmax/5;
	Ymin = sumADCmin/5;
	sumADCmax = 0;
	sumADCmin = 0;
}
float ErrorLine(void)
{
      CalibrationSenrsorValue(AdcValues);
      sum_Adcvalues = (AdcValuesAfterCalib[0] + AdcValuesAfterCalib[1] + AdcValuesAfterCalib[2] + AdcValuesAfterCalib[3] + AdcValuesAfterCalib[4]);
      float e2_line = (2*(AdcValuesAfterCalib[4] - AdcValuesAfterCalib[0]) + (AdcValuesAfterCalib[3] - AdcValuesAfterCalib[1]))*17/sum_Adcvalues;
      //e2_line = (e2_line + 0.2565)/0.7247;
      return -e2_line;
}
void find_e2_e3(void) {
	  CalibSensorDependOnEnvironment();
	  CaculatorFuntionSensor();
	  CalibrationSenrsorValue(AdcValues);
	  e2 = ErrorLine();
	  if(vtb != 0){
		  e3 = atan((e2 - pre_e2)/ (vtb*Ts*1000));
	  }
	  else e3 = 0;
}

//color sensor
void color_recognize(){
   for(int i = 0; i < 3; i++){
         switch (i){
            case 0:
               HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
               HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 0);
               red_value = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
               HAL_Delay(1);
               break;
            case 1:
               HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
               HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
               blue_value = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
               HAL_Delay(1);
               break;
            case 2:
               HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 1);
               HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
               green_value = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_1);
               HAL_Delay(1);
         }
      }
   //detect Red
      if ((red_value > 400 && red_value < 600) &&  (blue_value > 120 && blue_value < 250) && (green_value > 300 && green_value < 500)){
         green = 0;
         red = 1;
         blue = 0;
      }
      // Green
      else if ((red_value > 200 && red_value < 250) &&  (blue_value > 200 && blue_value < 250) && (green_value > 220 && green_value < 300)){
         green=1;
         red=0;
         blue = 0;
      }
      //Blue
      else if((red_value > 800 && red_value < 1000) &&  (blue_value > 1300 && blue_value < 1500) && ((blue_value > red_value)) && (red_value > green_value)){
		green=0;
		red=0;
		blue = 1;
      }
      else {
         green=0;
         red = 0;
         blue = 0;
      }
}

//giam toc do
void brake(void) {
	if(s >= 6500 && empty == 1) {
		vr = 0.2;
	}
	else if(empty == 0 && s <= 8800){
		vr = 0.7;
	}else if(empty == 0 && s > 8800 && s <= 9500){
		vr = 0.5;
	}else if(empty == 0 && s > 9500 && s <= 11600){
		vr = 0.7;
	}else if(empty == 0 && s > 11600){
		vr = 0.2;
	}
}

//motor
void find_speed(void) {
	//read encoder
	encoder_cnt_A = __HAL_TIM_GET_COUNTER(&htim1); //right
	encoder_cnt_B = __HAL_TIM_GET_COUNTER(&htim2); //left
	//cal rpm and speed (m/s)
	rpm_A = 60*((float)encoder_cnt_A - (float)encoder_cnt_pre_A)/(1496*Ts);
	rpm_B = -60*((float)encoder_cnt_B - (float)encoder_cnt_pre_B)/(1496*Ts);
	v_A = abs_(rpm_A*2*PI*r/60); //m/s
	v_B = abs_(rpm_B*2*PI*r/60); //m/s
	vtb = (v_A+v_B)/2; //m/s
}

float control_motor_A(float rpm_motor, float expected_rpm){ //dc1
	er_A_pre_pre = er_A_pre;
	er_A_pre = er_A;
	er_A = expected_rpm - rpm_motor;
	duty_cycle_pre_1 = duty_cycle_1;
	P_part_A = kp_A*(er_A - er_A_pre);
	I_part_A = 0.5*ki_A*Ts*(er_A + er_A_pre);
	D_part_A = kd_A/Ts*(er_A - 2*er_A_pre + er_A_pre_pre);
	duty_cycle_1 = duty_cycle_pre_1 + P_part_A + I_part_A + D_part_A;
	if(duty_cycle_1 < 0) duty_cycle_1 = 0;
	if(duty_cycle_1 > 100) duty_cycle_1 = 99;
	return duty_cycle_1;
}
float control_motor_B(float rpm_motor, float expected_rpm){ //dc2
	er_B_pre_pre = er_B_pre;
	er_B_pre = er_B;
	er_B = expected_rpm - rpm_motor;
	duty_cycle_pre_2 = duty_cycle_2;
	P_part_B = kp_B*(er_B - er_B_pre);
	I_part_B = 0.5*ki_B*Ts*(er_B + er_B_pre);
	D_part_B = kd_A/Ts*(er_B - 2*er_B_pre + er_B_pre_pre);
	duty_cycle_2 = duty_cycle_pre_2 + P_part_B + I_part_B + D_part_B;
	if(duty_cycle_2 < 0) duty_cycle_2 = 0;
	if(duty_cycle_2 > 100) duty_cycle_2 = 99;
	return duty_cycle_2;
}

//code lay mau cam bien mau
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	i++;
//	if(i == 1000){
//		char str[30];
//		snprintf(str, sizeof(str), "%.2f %.2f\r\n",(float)blue_value, Ts);
//		HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
//		i = 0;
//		Ts += 1;
//		k++;
//	}
//}

//code xem dap ung dong co
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
////	i++;
////	if(i == 95){
//		find_speed();
//		duty_cycle_A = control_motor_A(rpm_A, setpoint);
//		duty_cycle_B = control_motor_B(rpm_B, setpoint);
//		TIM3->CCR1 = (99 - (int)duty_cycle_A);
//		TIM3->CCR2 = (99 - (int)duty_cycle_B);
//		encoder_cnt_pre_A = encoder_cnt_A;
//		encoder_cnt_pre_B = encoder_cnt_B;
////		char str[30];
////		snprintf(str, sizeof(str), "%.2f %.2f\r\n",rpm_B, duty_cycle_B);
////		HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
////		i = 0;
////	}
//}

//code lay mau dong co tim ham truyen
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	i++;
//	if(i == 90){
//		uint32_t current_time = HAL_GetTick();
//		//n = 52.5 + 46.5 * sin(2 * 3.14 * 0.25 * current_time / 1000.0); //linear from 7-100%
//		n = 54 + 45 * sin(2 * 3.14 * 0.25 * (float)current_time / 1000.0); //linear from 10-100%
//		__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, (99-n));
////		n = 99;
////		TIM3->CCR1 = 0;
////		TIM3->CCR2 = 0;
//		encoder_cnt_2 =__HAL_TIM_GET_COUNTER(&htim2);
//		encoder_cnt_1 =__HAL_TIM_GET_COUNTER(&htim1);
//
//		rate_2 = -60*((float)encoder_cnt_2 - (float)encoder_cnt_pre_2)/(1496*0.01);
//		rate_1 = 60*((float)encoder_cnt_1 - (float)encoder_cnt_pre_1)/(1496*0.01);
//
//		encoder_cnt_pre_2 = encoder_cnt_2;
//		encoder_cnt_pre_1 = encoder_cnt_1;
//
//		char str[30];
//		snprintf(str, sizeof(str), "%.2f %.2f\r\n",rate_1, n+1);
//		HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
////		k++;
//		i = 0;
//	}
//}



//code chuong trinh chinh
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	i++;
	if(i == 85) { //tglm la 0.05s 85
		//no line no run and stop to get goods
		stop_or_run(); //den vach tai hang hoac ra khoi line se dung 'stop = 1'
		no_line_no_run();
		brake();
		//find error
		find_e2_e3();
		//find_speed
		find_speed();
		//fl tracking (lyapunov)
		Follow_Tracking();
		//find pwm to provide for motor
		duty_cycle_A = control_motor_A(rpm_A, setpoint_r);  //return pwm dc1
		duty_cycle_B = control_motor_B(rpm_B, setpoint_l);  //return pwm dc2
		//setup for next time
		encoder_cnt_pre_A = encoder_cnt_A;
		encoder_cnt_pre_B = encoder_cnt_B;
		pre_e2 = e2;
		pre_e3 = e3;
		//run fl pwm
		if(stop == 0) {
		  TIM3->CCR1 = (99-(int)duty_cycle_A);
		  TIM3->CCR2 = (99-(int)duty_cycle_B);
		}
		else {
		  TIM3->CCR1 = 99;
		  TIM3->CCR2 = 99;
		}
		s += vtb*Ts*1000;
		if((hangXanh == 1 || hangDo == 1) && outline == 0){
			stop = 0;
			if(empty == 0 && ((AdcValues[0] < 1000 && AdcValues[1] > 2000 && AdcValues[2] > 2000 && AdcValues[3] > 2000 && AdcValues[4] < 1000) //01110
					|| (AdcValues[0] > 2000 && AdcValues[1] < 1000 && AdcValues[2] < 1000 && AdcValues[3] < 1000 && AdcValues[4] > 2000) //10001
					|| (AdcValues[0] < 1000 && AdcValues[1] > 2000 && AdcValues[2] < 1000 && AdcValues[3] > 2000 && AdcValues[4] < 1000) //01010
					|| (AdcValues[0] > 2000 && AdcValues[1] > 2000 && AdcValues[2] < 1000 && AdcValues[3] > 2000 && AdcValues[4] < 1000) //11010
					|| (AdcValues[0] > 2000 && AdcValues[1] > 2000 && AdcValues[2] < 1000 && AdcValues[3] > 2000 && AdcValues[4] > 2000) //11011
					|| (AdcValues[0] > 2000 && AdcValues[1] > 2000 && AdcValues[2] < 1000 && AdcValues[3] < 1000 && AdcValues[4] > 2000) //11001
					|| (AdcValues[0] > 2000 && AdcValues[1] < 1000 && AdcValues[2] < 1000 && AdcValues[3] > 2000 && AdcValues[4] > 2000) //10011
					|| (AdcValues[0] > 2000 && AdcValues[1] < 1000 && AdcValues[2] > 2000 && AdcValues[3] > 2000 && AdcValues[4] < 1000) //10110
					|| (AdcValues[0] < 1000 && AdcValues[1] > 2000 && AdcValues[2] > 2000 && AdcValues[3] < 1000 && AdcValues[4] > 2000)
					|| (AdcValues[0] > 2000 && AdcValues[1] < 1000 && AdcValues[3] < 1000 && AdcValues[4] > 2000)
					|| (AdcValues[1] > 2000 && AdcValues[0] < 1000 && AdcValues[4] < 1000 && AdcValues[3] > 2000)
					|| (AdcValues[0] > 2000 && AdcValues[1] > 2000 && AdcValues[3] > 2000 && AdcValues[4] > 2000 && AdcValues[2] < 1000))){ //01101
				if(hangXanh == 1) {
					TIM3->CCR1 = 99;//99
					TIM3->CCR2 = 0;//0
				}
				else if(hangDo == 1){
					TIM3->CCR1 = 0;
					TIM3->CCR2 = 99;
				}
			}
		}
		//code lay mau sai so tra ve qua uart
//		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
//		char str[30];
//		snprintf(str, sizeof(str), "%.2f %.2f\r\n",e2, vtb);
//		HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
		i = 0;
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //cảm biến màu
  HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, 0);
  HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, 1);
  HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, 0);
  HAL_GPIO_WritePin(S3_GPIO_Port, S3_Pin, 1);
  HAL_TIM_IC_Start(&htim4, TIM_CHANNEL_1);

  //dò line
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)AdcValues,5);
  CalibSensorDependOnEnvironment();
  CaculatorFuntionSensor();

  //động cơ
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3);
  TIM3->CCR1 = 99;
  TIM3->CCR2 = 99;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//no line no run and stop to get goods
		stop_or_run(); //den vach tai hang hoac ra khoi line se dung 'stop = 1'
		no_line_no_run();
		//nhan dien mau hang
		color_recognize();
		//check is there good or not
		yes_or_no_good();
		//giam toc
		brake();
		if((hangXanh == 1 || hangDo == 1) && outline == 0){
			stop = 0;
			if(empty == 0 && ((AdcValues[0] < 1000 && AdcValues[1] > 2000 && AdcValues[2] > 2000 && AdcValues[3] > 2000 && AdcValues[4] < 1000) //01110
					|| (AdcValues[0] > 2000 && AdcValues[1] < 1000 && AdcValues[2] < 1000 && AdcValues[3] < 1000 && AdcValues[4] > 2000) //10001
					|| (AdcValues[0] < 1000 && AdcValues[1] > 2000 && AdcValues[2] < 1000 && AdcValues[3] > 2000 && AdcValues[4] < 1000) //01010
					|| (AdcValues[0] > 2000 && AdcValues[1] > 2000 && AdcValues[2] < 1000 && AdcValues[3] > 2000 && AdcValues[4] < 1000) //11010
					|| (AdcValues[0] > 2000 && AdcValues[1] > 2000 && AdcValues[2] < 1000 && AdcValues[3] > 2000 && AdcValues[4] > 2000) //11011
					|| (AdcValues[0] > 2000 && AdcValues[1] > 2000 && AdcValues[2] < 1000 && AdcValues[3] < 1000 && AdcValues[4] > 2000) //11001
					|| (AdcValues[0] > 2000 && AdcValues[1] < 1000 && AdcValues[2] < 1000 && AdcValues[3] > 2000 && AdcValues[4] > 2000) //10011
					|| (AdcValues[0] > 2000 && AdcValues[1] < 1000 && AdcValues[2] > 2000 && AdcValues[3] > 2000 && AdcValues[4] < 1000) //10110
					|| (AdcValues[0] < 1000 && AdcValues[1] > 2000 && AdcValues[2] > 2000 && AdcValues[3] < 1000 && AdcValues[4] > 2000) //01101
					|| (AdcValues[0] > 2000 && AdcValues[1] < 1000 && AdcValues[3] < 1000 && AdcValues[4] > 2000) //10_01
					|| (AdcValues[1] > 2000 && AdcValues[0] < 1000 && AdcValues[4] < 1000 && AdcValues[3] > 2000) //01_10
					|| (AdcValues[0] > 2000 && AdcValues[1] > 2000 && AdcValues[3] > 2000 && AdcValues[4] > 2000 && AdcValues[2] < 1000))){ //11011
				if(hangXanh == 1) {
					TIM3->CCR1 = 99;
					TIM3->CCR2 = 0;
					HAL_Delay(10);
				}
				else if(hangDo == 1){
					TIM3->CCR1 = 0;
					TIM3->CCR2 = 99;
					HAL_Delay(10);
				}
			}
		}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, S1_Pin|S0_Pin|S2_Pin|S3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : S1_Pin S0_Pin S2_Pin S3_Pin */
  GPIO_InitStruct.Pin = S1_Pin|S0_Pin|S2_Pin|S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
