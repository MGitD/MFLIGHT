/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "usbd_cdc_if.h"
#include "MPU6050.h"
#include "filter.h"
#include "stdbool.h"
#include "map.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// pid constants
#define Kp_x 0.0f
#define Kp_y 0.0f
#define Kp_z 0.0f
#define Ki_x 0.0f
#define Ki_y 0.0f
#define Ki_z 0.0f
#define Kd_x 0.0f
#define Kd_y 0.0f
#define Kd_z 0.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
MPU6050 gyro;
filter gyroX_filter;
filter gyroY_filter;
filter gyroZ_filter;
filter d_termX_filter;
filter d_termY_filter;
filter d_termZ_filter;
filter rc_roll;
filter rc_pitch;
filter rc_yaw;
filter rc_throttle;
filter rc_arm;
filter rc_prearm;

bool arm_ok = 0;
bool first_arm = 1;
bool failsafe = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */
#include "string.h"
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// micros delay function
uint16_t time_since_loop_start;
void delay_micros(uint16_t delay){
	__HAL_TIM_SET_COUNTER(&htim9, 0);
	while(__HAL_TIM_GET_COUNTER(&htim9) < delay);
}



#define RxBuf_SIZE 26
#define Main_Buf_SIZE 26

uint8_t RxBuf[RxBuf_SIZE];
uint8_t MainBuf[Main_Buf_SIZE];

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){

	if (huart->Instance == USART1){
		memcpy (MainBuf, RxBuf, RxBuf_SIZE);
		 HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuf, RxBuf_SIZE);
		 __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
}

// receiver IT test
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//    HAL_UART_Receive_IT(&huart1, RxBuf, RxBuf_SIZE);
		//memcpy (MainBuf, RxBuf, RxBuf_SIZE);

//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	filter_init(&gyroX_filter, 75.0f, 0.0005f);
	filter_init(&gyroY_filter, 75.0f, 0.0005f);
	filter_init(&gyroZ_filter, 75.0f, 0.0005f);
	filter_init(&rc_roll, 15.0f, 0.0001f);
	filter_init(&rc_pitch, 15.0f, 0.0001f);
	filter_init(&rc_yaw, 15.0f, 0.0001f);
	filter_init(&rc_throttle, 15.0f, 0.0001f);
	filter_init(&rc_arm, 5.0f, 0.0001f);
	filter_init(&rc_prearm, 4.0f, 0.0001f);
	filter_init(&d_termX_filter, 150.0f, 0.0005f);
	filter_init(&d_termY_filter, 150.0f, 0.0005f);
	filter_init(&d_termZ_filter, 150.0f, 0.0005f);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */

  // test of receive IT
  //HAL_UART_Receive_IT(&huart1, RxBuf, RxBuf_SIZE);


  // timers
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
   HAL_TIM_Base_Start(&htim9);
   // set duty to zero (oneshot125 uses 125us-250us) and the timer setup for 100mhz is 1250-2500
   uint32_t stop_motor = 1250;
   float idle = 1340.0;
   float MAX_MOTOR_OUT = 2500.0;

   HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin); // turn LED on to indicate delay
   HAL_Delay(7000);
   htim3.Instance->CCR1 = stop_motor; // when written zero throttle 125us the ESC makes the final beep
   htim3.Instance->CCR2 = stop_motor;
   htim3.Instance->CCR3 = stop_motor;
   htim3.Instance->CCR4 = stop_motor;
   HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
   HAL_Delay(2000);

  // receiver
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, RxBuf, RxBuf_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  char buf[16];
  uint16_t prearm = 0;
  uint16_t throttle = 0;
  uint16_t roll = 0;
  uint16_t pitch = 0;
  uint16_t yaw = 0;
  uint16_t beep = 0;
  uint16_t arm = 0;
  uint16_t wheel = 0;
  uint16_t time_passed = 0;
  float roll_filt = 0;
  float pitch_filt = 0;
  float yaw_filt = 0;
  float throttle_filt = 0;
  float arm_filt = 0;
  float prearm_filt = 0;
  float M1 = 1400.0;
  float M2 = 1400.0;
  float M3 = 1400.0;
  float M4 = 1400.0;
  float roll_PID = 0.0;
  float pitch_PID = 0.0;
  float yaw_PID = 0.0;
  float d_termX = 0.0;
  float d_termY = 0.0;
  float d_termZ = 0.0;
  float I_termX = 0.0;
  float I_termY = 0.0;
  float I_termZ = 0.0;

  float P_termX = 0.0;
  float P_termY = 0.0;
  float P_termZ = 0.0;

  uint8_t counting = 0;

  float D_termX_filtered;
  float D_termY_filtered;
  float D_termZ_filtered;
  float gyroX_filtered_prev;
  float gyroY_filtered_prev;
  float gyroZ_filtered_prev;




  // pid for tuning
  float Kp_X = 0.6f;
  float Kp_Y = 0.6f;
  float Kp_Z = 0.6f;
  float Ki_X = 0.0001f;
  float Ki_Y = 0.0001f;
  float Ki_Z = 0.0001f;
  float Kd_X = 0.001f;
  float Kd_Y = 0.001f;
  float Kd_Z = 0.001f;

  /*
  Kp_X = Kp_x;
  Kp_Y = Kp_y;
  Kp_Z = Kp_z;
  Ki_X = Ki_x;
  Ki_Y = Ki_y;
  Ki_Z = Ki_z;
  Kd_X = Kd_x;
  Kd_Y = Kd_y;
  Kd_Z = Kd_z;
*/

  // receiver end

  // gyro init
  uint8_t IMU_check;
  IMU_check = MPU6050_initialise(&gyro, &hi2c1);
  if (IMU_check == 0){
  HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
  HAL_Delay(2000);
  HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
  }


  float gyroX;
  float gyroY;
  float gyroZ;

  float gyroOffsetX = 0;
  float gyroOffsetY = 0;
  float gyroOffsetZ = 0;

  uint8_t cnt = 0;
  float offset_cnt = 100;
  while(cnt < offset_cnt){
	  MPU6050_read_gyro(&gyro);
	  gyroOffsetX += ((gyro.gyro_ds[0])/offset_cnt);
	  gyroOffsetY += ((gyro.gyro_ds[1])/offset_cnt);
	  gyroOffsetZ += ((gyro.gyro_ds[2])/offset_cnt);
	  cnt++;
  }

  float gyroX_filtered;
  float gyroY_filtered;
  float gyroZ_filtered;




 // gyro end


// due to rotating receiver packages, skip 10 package as soon as strange data arrives
  uint8_t cnt_bad_data_throttle = 0;
  uint16_t prev_throttle = 0;
  uint8_t cnt_bad_data_roll = 0;
  uint16_t prev_roll = 0;
  uint8_t cnt_bad_data_pitch = 0;
  uint16_t prev_pitch = 0;
  uint8_t cnt_bad_data_yaw = 0;
  uint16_t prev_yaw = 0;
  uint8_t cnt_bad_data_arm = 0;
  uint16_t prev_arm = 0;
  uint8_t cnt_bad_data_prearm = 0;
   uint16_t prev_prearm = 0;
    //HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
	  // MOTORS to zero
	   htim3.Instance->CCR1 = stop_motor;
	   htim3.Instance->CCR2 = stop_motor;
	   htim3.Instance->CCR3 = stop_motor;
	   htim3.Instance->CCR4 = stop_motor;

	  /*
	   htim3.Instance->CCR1 = idle;
	   htim3.Instance->CCR2 = idle;
	   htim3.Instance->CCR3 = idle;
	   htim3.Instance->CCR4 = idle;
*/

	  // before arm all ok
	  throttle = ((MainBuf[4] & 0x07) << 8) | MainBuf[3]; // 196-1792
	  prearm = ((MainBuf[11] & 0x03) << 9) | (MainBuf[10] << 1); // > 1000 prearm ok
	  prearm = ((MainBuf[9] & 0x80) >> 7) | prearm;
	  arm = ((MainBuf[9] & 0b01111111) << 4) | ((MainBuf[8] & 0b11110000) >> 4);

	  // due to rotating receiver data
	  if(throttle < 100 && cnt_bad_data_throttle <= 15){
		  cnt_bad_data_throttle++;
		  throttle = prev_throttle;
	  }else{
		  cnt_bad_data_throttle = 0;
		  prev_throttle = throttle;
	  }


	  if(arm < 100 && cnt_bad_data_arm <= 20){
			  cnt_bad_data_arm++;
			  arm = prev_arm;
		  }else{
			  cnt_bad_data_arm = 0;
			  prev_arm = arm;
		  }

	  if(prearm < 100 && cnt_bad_data_prearm <= 20){
			  cnt_bad_data_arm++;
			  prearm = prev_prearm;
		  }else{
			  cnt_bad_data_prearm = 0;
			  prev_prearm = prearm;
		  }
	  // filter rc commands

	  throttle_filt = filter_apply(&rc_throttle, (float) throttle);
	  arm_filt = filter_apply(&rc_arm, (float) arm);
	  prearm_filt = filter_apply(&rc_prearm, (float) prearm);


	  // throttle 196-1792
	  if (throttle < 1800 && throttle > 100){ // check actual value for failsafe
		  failsafe = 0;
	  } else failsafe = 1;

	  if (IMU_check == 0 && throttle < 300  && prearm > 500 && arm < 500 && (failsafe == 0)){
		  arm_ok = 1;
	  }

	  // check if gyro is not calibrated correctly upon first arm
	  if (first_arm){
		  first_arm = 0;
		  MPU6050_read_gyro(&gyro);
		  gyroX = gyro.gyro_ds[0] - gyroOffsetX;
		  gyroY = gyro.gyro_ds[1] - gyroOffsetY;
		  gyroZ = gyro.gyro_ds[2] - gyroOffsetZ;
		 if (gyroX > 4 || gyroY > 4 || gyroZ > 4){
			 arm_ok = 0;
		 }
	  }

      //arm_ok = 1; // for test
	  // flying loop
	  while(arm_ok) {
		  counting++;
	  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
	  __HAL_TIM_SET_COUNTER(&htim9, 0); // use tim9 for time-control (loop time)

	  //time_since_loop_start = __HAL_TIM_GET_COUNTER(&htim9);
	  // RC COMMANDS
	  arm = ((MainBuf[9] & 0b01111111) << 4) | ((MainBuf[8] & 0b11110000) >> 4); // if < 1000 = armed
	  throttle = ((MainBuf[4] & 0x07) << 8) | MainBuf[3]; // 196-1792
	  pitch = ((MainBuf[7] & 0x01) << 10) | (MainBuf[6] << 2);
	  pitch = ((MainBuf[5] & 0xC0) >> 6) | pitch;
	  yaw = ((MainBuf[8] & 0x0F) << 7) | ((MainBuf[7] & 0b11111110) >> 1);
	  roll = ((MainBuf[5] & 0x3F) << 5) | ((MainBuf[4] & 0xF8) >> 3); // 192-1792

	  // due to rotating receiver data
	 	  if(throttle < 100 && cnt_bad_data_throttle <= 15){
	 		  cnt_bad_data_throttle++;
	 		  throttle = prev_throttle;
	 	  }else{
	 		  cnt_bad_data_throttle = 0;
	 		  prev_throttle = throttle;
	 	  }

	 	  if(((roll-prev_roll < -300)||(roll-prev_roll > 300)) && cnt_bad_data_roll <= 15){
	 			  cnt_bad_data_roll++;
	 			  roll = prev_roll;
	 		  }else{
	 			  cnt_bad_data_roll = 0;
	 			  prev_roll = roll;
	 		  }

	 	  if(pitch < 170 && cnt_bad_data_pitch <= 15){
	 			  cnt_bad_data_pitch++;
	 			  pitch = prev_pitch;
	 		  }else{
	 			  cnt_bad_data_pitch = 0;
	 			  prev_pitch = pitch;
	 		  }

	 	  if(((yaw-prev_yaw < -300)||(yaw-prev_yaw > 300)) && cnt_bad_data_yaw <= 15){
	 			  cnt_bad_data_yaw++;
	 			  yaw = prev_yaw;
	 		  }else{
	 			  cnt_bad_data_yaw = 0;
	 			  prev_yaw = yaw;
	 		  }




	 	  if(arm < 100 && cnt_bad_data_arm <= 30){
	 			  cnt_bad_data_arm++;
	 			  arm = prev_arm;
	 		  }else{
	 			  cnt_bad_data_arm = 0;
	 			  prev_arm = arm;
	 		  }



	  // filter rc commands
	  roll_filt = filter_apply(&rc_roll, (float) roll); // this is wrong values, prior to filter
	  pitch_filt = filter_apply(&rc_pitch, (float) pitch);
	  yaw_filt = filter_apply(&rc_yaw, (float) yaw);
	  throttle_filt = filter_apply(&rc_throttle, (float) throttle);
	  arm_filt = filter_apply(&rc_arm, (float) arm);

	  // check if disarm, check failsafe // check value
		  if (arm_filt > 600){
			 arm_ok = 0;
		  }

		  if (throttle < 100 || throttle > 1800){
			  failsafe = 1;
			  arm_ok = 0;
		  }

	  // MAP and CONSTRAIN RC_COMMANDS
		  roll_filt = constrain(roll_filt, 206.0, 1776.0);
		  pitch_filt = constrain(pitch_filt, 194.0, 1790.0);
		  yaw_filt =  constrain(yaw_filt, 198.0, 1768.0);
		  throttle_filt = constrain(throttle_filt, 191.0, 1788.0);
		  roll_filt = map(roll_filt, -1000.0, 1000.0, 206.0, 1776.0);
		  pitch_filt = map(pitch_filt, -1000.0, 1000.0, 194.0, 1790.0);
		  yaw_filt =  map(yaw_filt, -1000.0, 1000.0, 198.0, 1768.0);
		  throttle_filt = map(throttle_filt, 1250.0, 2500.0, 191.0, 1788.0);
		  throttle_filt = constrain(throttle_filt, idle, 2000.0);

	  // GYRO READ
      MPU6050_read_gyro(&gyro);
	  gyroX = gyro.gyro_ds[0] - gyroOffsetX;
	  gyroY = gyro.gyro_ds[1] - gyroOffsetY;
	  gyroZ = gyro.gyro_ds[2] - gyroOffsetZ;

	  // filters
	  gyroX_filtered = filter_apply(&gyroX_filter, gyroX);
	  gyroY_filtered = filter_apply(&gyroY_filter, gyroY);
	  gyroZ_filtered = filter_apply(&gyroZ_filter, gyroZ);

	  // PID



	  // compute proportional term
	  P_termX = - pitch_filt - gyroX_filtered;
	  P_termY =   roll_filt -  gyroY_filtered;
	  P_termZ =  - yaw_filt -  gyroZ_filtered;

	  // compute derivative term
	  d_termX = gyroX_filtered - gyroX_filtered_prev;
	  d_termY = gyroY_filtered - gyroY_filtered_prev;
	  d_termZ = gyroZ_filtered - gyroZ_filtered_prev;

	  // filter derivative term
	  D_termX_filtered = filter_apply(&d_termX_filter, d_termX);
	  D_termY_filtered = filter_apply(&d_termY_filter, d_termY);
	  D_termZ_filtered = filter_apply(&d_termZ_filter, d_termZ);

	  // shift
	  gyroX_filtered_prev = gyroX_filtered;
	  gyroY_filtered_prev = gyroY_filtered;
	  gyroZ_filtered_prev = gyroZ_filtered;

	  // compute I term
	  I_termX = I_termX + P_termX * 0.0005;
	  I_termY = I_termY + P_termY * 0.0005;
	  I_termZ = I_termZ + P_termZ * 0.0005;

	  // constrain I term
	  if (I_termX > 100.0){
		  I_termX = 100.0;
	  }
	  if (I_termX < -100.0){
		  I_termX = -100.0;
	  }
	  if (I_termY > 100.0){
		  I_termY = 100.0;
	  }
	  if (I_termY < -100.0){
		  I_termY = -100.0;
	  }
	  if (I_termZ > 100.0){
		  I_termZ = 100.0;
	  }
	  if (I_termZ < -100.0){
		  I_termZ = -100.0;
	  }


	  // compute pids
	  roll_PID  = Kp_Y * P_termY + Ki_Y * I_termY - Kd_Y * D_termY_filtered;
	  pitch_PID = Kp_X * P_termX + Ki_X * I_termX - Kd_X * D_termX_filtered;
	  yaw_PID   = Kp_Z * P_termZ + Ki_Z * I_termZ;// - Kd_Z * D_termZ_filtered;

	  //time_since_loop_start = __HAL_TIM_GET_COUNTER(&htim9);

	  // MIXER
	   M1 = throttle_filt + pitch_PID + roll_PID + yaw_PID;
	   M2 = throttle_filt + pitch_PID - roll_PID - yaw_PID;
	   M3 = throttle_filt - pitch_PID - roll_PID + yaw_PID;
	   M4 = throttle_filt - pitch_PID + roll_PID - yaw_PID;
/*
	   if (M1 < idle || M2 < idle || M3 < idle || M4 < idle){
		   Kp_X = 0.8*Kp_X;
		   Kp_Y = 0.8*Kp_Y;
		   Kp_Z = 0.8*Kp_Z;
		   Ki_X = 0.8*Ki_X;
		   Ki_Y = 0.8*Ki_Y;
		   Ki_Z = 0.8*Ki_Z;
		   Kd_X = 0.8*Kd_X;
		   Kd_Y = 0.8*Kd_Y;
		   Kd_Z = 0.8*Kd_Z;
	 	   }else{
			   Kp_X = Kp_x;
			   Kp_Y = Kp_y;
			   Kp_Z = Kp_z;
			   Ki_X = Ki_x;
			   Ki_Y = Ki_y;
			   Ki_Z = Ki_z;
			   Kd_X = Kd_x;
			   Kd_Y = Kd_y;
			   Kd_Z = Kd_z;
	 	   }
	   if (M1 < MAX_MOTOR_OUT || M2 < MAX_MOTOR_OUT || M3 < MAX_MOTOR_OUT || M4 < MAX_MOTOR_OUT){
		   Kp_X = 0.8*Kp_X;
		   Kp_Y = 0.8*Kp_Y;
		   Kp_Z = 0.8*Kp_Z;
		   Ki_X = 0.8*Ki_X;
		   Ki_Y = 0.8*Ki_Y;
		   Ki_Z = 0.8*Ki_Z;
		   Kd_X = 0.8*Kd_X;
		   Kd_Y = 0.8*Kd_Y;
		   Kd_Z = 0.8*Kd_Z;
	 	   }else{
			   Kp_X = Kp_x;
			   Kp_Y = Kp_y;
			   Kp_Z = Kp_z;
			   Ki_X = Ki_x;
			   Ki_Y = Ki_y;
			   Ki_Z = Ki_z;
			   Kd_X = Kd_x;
			   Kd_Y = Kd_y;
			   Kd_Z = Kd_z;
	 	   }
	  // compute pids
	  roll_PID  = Kp_Y * P_termY + Ki_Y * I_termY - Kd_Y * D_termY_filtered;
	  pitch_PID = Kp_X * P_termX + Ki_X * I_termX - Kd_X * D_termX_filtered;
	  yaw_PID   = Kp_Z * P_termZ + Ki_Z * I_termZ - Kd_Z * D_termZ_filtered;


	  // MIXER
	   M1 = throttle_filt + pitch_PID + roll_PID + yaw_PID;
	   M2 = throttle_filt + pitch_PID - roll_PID - yaw_PID;
	   M3 = throttle_filt - pitch_PID - roll_PID + yaw_PID;
	   M4 = throttle_filt - pitch_PID + roll_PID - yaw_PID;
*/
	M1 = constrain(M1, idle, 2500.0);
	M2 = constrain(M2, idle, 2500.0);
	M3 = constrain(M3, idle, 2500.0);
	M4 = constrain(M4, idle, 2500.0);

			  // MOTOR OUTPUT
	//M1 = throttle_filt;
	// M1 -> M3
	//M2 -> M2
	// M3 -> M4
	//
	   //htim3.Instance->CCR1 = M4; //M1
	   //htim3.Instance->CCR2 = M1; //M3
	   //htim3.Instance->CCR3 = M2; //M2
	   //htim3.Instance->CCR4 = M3; //M4

	   htim3.Instance->CCR1 = M2;//M2;
	   htim3.Instance->CCR2 = M3;//M3;
	   htim3.Instance->CCR3 = M1;//M1;
	   htim3.Instance->CCR4 = M4;//M4;

	/*
	   htim3.Instance->CCR1 = idle;
	   htim3.Instance->CCR2 = idle;
	   htim3.Instance->CCR3 = idle;
	   htim3.Instance->CCR4 = idle;
 */

	   	  // for debuging usb
	      //sprintf(buf, "%.2f\r\n", roll_PID);
		  //CDC_Transmit_FS((uint8_t *) buf, strlen(buf));




	   // TIME-CONTROL
	  time_since_loop_start = __HAL_TIM_GET_COUNTER(&htim9); // one cnt == 1 micros
	  time_passed = time_since_loop_start;
	  if (time_since_loop_start < 500){
		  delay_micros(500 - time_since_loop_start);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 400000;
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
  htim3.Init.Prescaler = 10-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2500-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 100-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 420000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_STATUS_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_STATUS_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
