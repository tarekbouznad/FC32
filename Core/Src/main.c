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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../../USB_DEVICE/App/usbd_cdc_if.h"
#include "stdio.h"
#include <string.h>
#include "mpu6050.h"
#include <math.h>
#include "ibus.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define IBUS_UART				(&huart4)
#define IBUS_USER_CHANNELS		6		// Use 6 channels
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == IBUS_UART)
		ibus_reset_failsafe(huart);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// PID gain and limit settings
	// Roll PID
	float pid_p_gain_roll = 1.3; // P-gain for roll PID controller
	float pid_i_gain_roll = 0.04; // I-gain for roll PID controller
	float pid_d_gain_roll = 18.0; // D-gain for roll PID controller
	int pid_max_roll = 400; // Maximum output of the roll PID controller (+/-)

	// Pitch PID (using same gains and limits as roll for simplicity)
	float pid_p_gain_pitch = pid_p_gain_roll;
	float pid_i_gain_pitch = pid_i_gain_roll;
	float pid_d_gain_pitch = pid_d_gain_roll;
	int pid_max_pitch = pid_max_roll;

	// Yaw PID
	float pid_p_gain_yaw = 4.0; // P-gain for yaw PID controller
	float pid_i_gain_yaw = 0.02; // I-gain for yaw PID controller
	float pid_d_gain_yaw = 0.0; // D-gain for yaw PID controller
	int pid_max_yaw = 400; // Maximum output of the yaw PID controller (+/-)

	// Other variables
	int16_t esc_1, esc_2, esc_3, esc_4; // ESC pulse values
	int16_t throttle; // Throttle input
	double gyro_pitch, gyro_roll, gyro_yaw; // Gyroscope measurements

	// Loop timing variables
	uint32_t time1,time2,timex;
	uint32_t loop_timer, error_timer;
	uint32_t startTime, endTime;
	uint32_t loopExecutionTime = 0;
	uint8_t error, error_counter, error_led;
	// PID variable
	float pid_error_temp;
	float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll,
			pid_last_roll_d_error;
	float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input,
			pid_output_pitch, pid_last_pitch_d_error;
	float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw,
			pid_last_yaw_d_error;
	float roll_level_adjust, pitch_level_adjust;
	float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
	float acc_x, acc_y, acc_z, acc_total_vector;

	// MPU6050 struct for gyro data
	MPU6050_t GYRO;

	// ibus variable for receiver inputs
	uint16_t ibus_data[IBUS_USER_CHANNELS];
	// Buffer for UART transmission
	char buf[120];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	void calculate_pid(void) {
		//Roll calculations
		pid_error_temp = gyro_roll_input - pid_roll_setpoint;
		pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;

		//Limiting the I term
		if (pid_i_mem_roll > 100)
			pid_i_mem_roll = 100;
		else if (pid_i_mem_roll < 100 * -1)
			pid_i_mem_roll = 100 * -1;

		pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);

		//Limiting the output
		if (pid_output_roll > pid_max_roll)
			pid_output_roll = pid_max_roll;
		else if (pid_output_roll < pid_max_roll * -1)
			pid_output_roll = pid_max_roll * -1;

		pid_last_roll_d_error = pid_error_temp;

		//Pitch calculations
		pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
		pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;

		//Limiting the I term
		if (pid_i_mem_pitch > 100)
			pid_i_mem_pitch = 100;
		else if (pid_i_mem_pitch < 100 * -1)
			pid_i_mem_pitch = 100 * -1;

		pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);

		//Limiting the output
		if (pid_output_pitch > pid_max_pitch)
			pid_output_pitch = pid_max_pitch;
		else if (pid_output_pitch < pid_max_pitch * -1)
			pid_output_pitch = pid_max_pitch * -1;

		pid_last_pitch_d_error = pid_error_temp;

		//Yaw calculations
		pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
		pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;


		if (pid_i_mem_yaw > 100)
			pid_i_mem_yaw = 100;
		else if (pid_i_mem_yaw < 100 * -1)
			pid_i_mem_yaw = 100 * -1;

		pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);

		//Limiting the output
		if (pid_output_yaw > pid_max_yaw)
			pid_output_yaw = pid_max_yaw;
		else if (pid_output_yaw < pid_max_yaw * -1)
			pid_output_yaw = pid_max_yaw * -1;

		pid_last_yaw_d_error = pid_error_temp;
	}

	/**
	  * @brief Read sensor data from MPU6050 and perform gyro-based signal processing
	  * @retval None
	  */
	void gyro_signalen() {
	    // Read all sensor data from MPU6050
	    MPU6050_Read_All(&hi2c1, &GYRO);

	    // Calculate gyro signal adjustments
	    gyro_roll = (GYRO.Gx - GYRO.Gx_cal);
	    gyro_pitch = (GYRO.Gy - GYRO.Gy_cal);
	    gyro_yaw = (GYRO.Gz - GYRO.Gz_cal);
	    acc_x = GYRO.Ax;
	    acc_y = GYRO.Ay;
	    acc_z = GYRO.Az;

	    // Calculate pitch and roll angles using gyro readings
	    angle_pitch += (float)gyro_pitch / 250;  // Calculate traveled pitch angle
	    angle_roll += (float)gyro_roll / 250;    // Calculate traveled roll angle

	    // Correct for yaw effect on pitch and roll angles
	    angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.0000697778);  // Transfer roll angle to pitch angle if yawed
	    angle_roll += angle_pitch * sin((float)gyro_yaw * 0.0000697778);  // Transfer pitch angle to roll angle if yawed

	    // Calculate pitch and roll angles using accelerometer readings
	    acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));  // Calculate total accelerometer vector
	    if (abs(acc_y) < acc_total_vector) {
	        angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.295;  // Calculate pitch angle
	    }
	    if (abs(acc_x) < acc_total_vector) {
	        angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.295;   // Calculate roll angle
	    }

	    // Correct gyro drift using accelerometer readings
	    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;  // Correct gyro drift in pitch angle
	    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;     // Correct gyro drift in roll angle

	    // Calculate pitch and roll angle corrections
	    pitch_level_adjust = angle_pitch * 15;  // Calculate pitch angle correction
	    roll_level_adjust = angle_roll * 15;     // Calculate roll angle correction

	    // Transmit debug information
	    sprintf(buf, "angle_roll = %d angle_pitch = %d \n", (int)angle_roll, (int)angle_pitch);
	    int size = strlen(buf);
	    CDC_Transmit_FS((uint8_t*)buf, size);
	}


	//In this part the error LED signal is generated.
	void error_signal(void) {
		if (error >= 100)
			HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_SET);         //When the error is 100 the LED is always on.
		else if (error_timer < HAL_GetTick()) { //If the error_timer value is smaller that the millis() function.
			error_timer = HAL_GetTick() + 250; //Set the next error_timer interval at 250ms.
			if (error > 0 && error_counter > error + 3)
				error_counter = 0; //If there is an error to report and the error_counter > error +3 reset the error.
			if (error_counter < error && error_led == 0 && error > 0) { //If the error flash sequence isn't finisched (error_counter < error) and the LED is off.
				HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_SET);                                //Turn the LED on.
				error_led = 1; //Set the LED flag to indicate that the LED is on.
			} else { //If the error flash sequence isn't finisched (error_counter < error) and the LED is on.
				HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_RESET);                                //Turn the LED off.
				error_counter++; //Increment the error_counter variable by 1 to keep trach of the flashes.
				error_led = 0; //Set the LED flag to indicate that the LED is off.
			}
		}
	}

	//Variable Initialization
		ibus_data[0]=0;
		ibus_data[1]=0;
		ibus_data[2]=0;
		ibus_data[3]=0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_Init();
	ibus_init();
	HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_RESET);
	int size = 0;
	error = MPU6050_Init(&hi2c1);

	while (error != 0) {
		sprintf(buf, "GYRO NOT INIT\n");
		size = strlen(buf);
		CDC_Transmit_FS((uint8_t*) buf, size);
		error_signal();
		HAL_Delay(4);
	}

	sprintf(buf, "GYRO Inititialized\n");
	size = strlen(buf);
	CDC_Transmit_FS((uint8_t*) buf, size);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	TIM2->CCR1 = 800; // PIN
	TIM2->CCR2 = 800;
	TIM2->CCR3 = 800;
	TIM2->CCR4 = 800;
	sprintf(buf, "Calibration...\n");
	size = strlen(buf);
	CDC_Transmit_FS((uint8_t*) buf, size);
	MPU6050_Calibrate(&hi2c1, &GYRO, 0);
	HAL_Delay(50);
	//Read RC command
	ibus_read(ibus_data);
	ibus_soft_failsafe(ibus_data, 10); // if ibus is not updated, clear ibus data.
	//Wait until the receiver is active.
	  while (ibus_data[0] < 990 || ibus_data[1] < 990 || ibus_data[2] < 990 || ibus_data[3] < 990)  {
	    error = 3;                                                  //Set the error status to 3.
	    error_signal();                                             //Show the error via the red LED.
	    ibus_read(ibus_data);
	    ibus_soft_failsafe(ibus_data, 10); // if ibus is not updated, clear ibus data.
		HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_SET);
	    HAL_Delay(4);
	  }

	  while (ibus_data[2] < 990 || ibus_data[2] < 990)  {
	     error = 4;                                                  //Set the error status to 4.
	     error_signal();                                             //Show the error via the red LED.
	     HAL_Delay(4);
	     HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_SET);
	   }
	   error = 0;                                                    //Reset the error status to 0.

	   //When everything is done, turn off the led.
	HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_RESET);
	roll_level_adjust=0;
	pitch_level_adjust=0;
	HAL_Delay(100);
	sprintf(buf, "Quadcopter Functional\n");
	size = strlen(buf);
	CDC_Transmit_FS((uint8_t*) buf, size);
	pid_pitch_setpoint = 0;
	pid_yaw_setpoint = 0;
	pid_roll_setpoint = 0;

	gyro_roll_input = 0;
	gyro_pitch_input = 0;
	gyro_yaw_input = 0;

	pid_error_temp = 0;

	pid_i_mem_pitch = 0;
	pid_i_mem_roll = 0;
	pid_i_mem_yaw = 0;

	pid_last_pitch_d_error = 0;
	pid_last_yaw_d_error = 0;
	pid_last_roll_d_error = 0;

	pid_i_mem_roll = 0;
	pid_last_roll_d_error = 0;
	pid_i_mem_pitch = 0;
	pid_last_pitch_d_error = 0;
	pid_i_mem_yaw = 0;
	pid_last_yaw_d_error = 0;

	pid_roll_setpoint = 0;
	pid_yaw_setpoint = 0;
	pid_pitch_setpoint = 0;

	angle_pitch=0;
	angle_roll=0;
	HAL_TIM_Base_Init(&htim3);
	HAL_TIM_Base_Start(&htim3);              //initiate the loop timer counter

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		TIM3->CNT = 0;
		time1=TIM3->CNT;

		ibus_read(ibus_data);
		ibus_soft_failsafe(ibus_data, 10); // if ibus is not updated, clear ibus data.

		//Let's get the current gyro data and scale it to degrees per second for the pid calculations.
		gyro_signalen();


		//The PID set point in degrees per second is determined by the roll receiver input.
		//In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_roll_setpoint = 0;
		//We need a little dead band of 16us for better results.
		if(ibus_data[0]<1508 && ibus_data[0]>1492)
			pid_roll_setpoint = 0;
		else if (ibus_data[0] > 1508)
			pid_roll_setpoint = ibus_data[0] - 1508;
		else if (ibus_data[0] < 1492)
			pid_roll_setpoint = ibus_data[0] - 1492;

		pid_roll_setpoint -= roll_level_adjust;
		pid_roll_setpoint /= 3; //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.

		//The PID set point in degrees per second is determined by the pitch receiver input.
		//In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_pitch_setpoint = 0;
		//We need a little dead band of 16us for better results.
		if (ibus_data[1] < 1508 && ibus_data[1] > 1492)
			pid_pitch_setpoint = 0;
		else if (ibus_data[1] > 1508)
			pid_pitch_setpoint = ibus_data[1] - 1508;
		else if (ibus_data[1] < 1492)
			pid_pitch_setpoint = ibus_data[1] - 1492;

		pid_pitch_setpoint -= pitch_level_adjust; //Subtract the angle correction from the standardized receiver pitch input value.
		pid_pitch_setpoint /= 3; //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

		//The PID set point in degrees per second is determined by the yaw receiver input.
		//In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
		pid_yaw_setpoint = 0;
		//We need a little dead band of 16us for better results.
		if (ibus_data[3] > 1050) { //Do not yaw when turning off the motors.
			if (ibus_data[3] < 1508 && ibus_data[3] > 1492)
				pid_yaw_setpoint = 0;
			else if (ibus_data[3] > 1508)
				pid_yaw_setpoint = ibus_data[3] - 1508;
			else if (ibus_data[3] < 1492)
				pid_yaw_setpoint = ibus_data[3] - 1492;
		}

		gyro_roll_input = (gyro_roll_input * 0.7) + (((float) gyro_roll) * 0.3); //Gyro pid input is deg/sec.

		gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float) gyro_pitch) * 0.3); //Gyro pid input is deg/sec.

		gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float) gyro_yaw) * 0.3); //Gyro pid input is deg/sec.

		calculate_pid();
		throttle = ibus_data[2];
		if (throttle > 1800)
			throttle = 1800;



		/**
		 * Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
		 * by applying PID control.
		 *
		 * (2) (1)            x
		 *   \ /            z ↑
		 *    X              \|
		 *   / \        y <---+
		 * (3) (4)
		 *
		 * Motors 1 & 3 run CCW
		 * Motors 2 & 4 run CW
		 *
		 * Each motor output is considered as a servomotor. As a result, value range is about 1000µs to 2000µs
		 */


		esc_1 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW).
		esc_2 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW).
		esc_3 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW).
		esc_4 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW).

		if (esc_1 < 1100)
			esc_1 = 1100;                             //Keep the motors running.
		if (esc_2 < 1100)
			esc_2 = 1100;                             //Keep the motors running.
		if (esc_3 < 1100)
			esc_3 = 1100;                             //Keep the motors running.
		if (esc_4 < 1100)
			esc_4 = 1100;                             //Keep the motors running.

		if (esc_1 > 2000)
			esc_1 = 2000;                     //Limit the esc-1 pulse to 2000us.
		if (esc_2 > 2000)
			esc_2 = 2000;                     //Limit the esc-2 pulse to 2000us.
		if (esc_3 > 2000)
			esc_3 = 2000;                     //Limit the esc-3 pulse to 2000us.
		if (esc_4 > 2000)
			esc_4 = 2000;                     //Limit the esc-4 pulse to 2000us.

		TIM2->CCR1 = esc_1; //Set the throttle receiver input pulse to the ESC 1 output pulse.
		TIM2->CCR2 = esc_2; //Set the throttle receiver input pulse to the ESC 2 output pulse.
		TIM2->CCR3 = esc_3; //Set the throttle receiver input pulse to the ESC 3 output pulse.
		TIM2->CCR4 = esc_4; //Set the throttle receiver input pulse to the ESC 4 output pulse.


		HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_SET);

		timex=TIM3->CNT-time1;       //how much time did it take the code to come here
		while(TIM3->CNT-time1<=4000);
		HAL_GPIO_WritePin(LD_R_GPIO_Port, LD_R_Pin, GPIO_PIN_RESET);
		time2=(TIM3->CNT-time1);
		//how much time did the main loop take
		/* DATA SENDING */

		//sprintf(buf, "Ax = %d  Ay = %d\n", (int) GYRO.KalmanAngleX, (int) GYRO.KalmanAngleY);
		//sprintf(buf, " gyro_roll = %d gyro_pitch = %d  gyro_yaw = %d\n", (int) pid_output_roll, (int) pid_output_pitch, (int) pid_output_yaw);
		//sprintf(buf, " esc_1 = %d esc_2 = %d  esc_3 = %d esc_4 = %d\n", (int) ibus_data[0], (int) ibus_data[1], (int) ibus_data[2], (int) ibus_data[3]);
		//sprintf(buf, "roll = %d pitch = %d yaw = %d roll = %d pitch = %d yaw = %d\n",(int) gyro_roll_input,(int) gyro_pitch_input,(int) gyro_yaw_input, (int) pid_output_roll, (int) pid_output_pitch, (int) pid_output_yaw);
		//size = strlen(buf);
		//CDC_Transmit_FS((uint8_t*) buf, size);



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
	while (1) {
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
